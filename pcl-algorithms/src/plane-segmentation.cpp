#include <iostream>
#include <thread>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/copy_point.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std::chrono_literals;
ros::Publisher pose_pub;

pcl::visualization::PCLVisualizer::Ptr
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->initCameraParameters();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr
simpleVisColor(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(1, 1, 1);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->initCameraParameters();
  return (viewer);
}

void planeSegmentation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
}

pcl::PointCloud<pcl::Normal>::Ptr normalComputation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.03);
  ne.compute(*normals);
  return normals;
}

//function to calculate cross product of two vectors
void cross_product(std::vector<float> &vector_a, std::vector<float> &vector_b, std::vector<float> &cp)
{
  cp[0] = vector_a[1] * vector_b[2] - vector_a[2] * vector_b[1];
  cp[1] = -(vector_a[0] * vector_b[2] - vector_a[2] * vector_b[0]);
  cp[2] = vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0];
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColor(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloudColor);

  // Plane segementation for removing major plane
  pcl::ModelCoefficients::Ptr coefficients_major_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_major_plane(new pcl::PointIndices);
  planeSegmentation(cloud, coefficients_major_plane, inliers_major_plane);

  if (inliers_major_plane->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }

  std::cerr << "Model coefficients: " << coefficients_major_plane->values[0] << " "
            << coefficients_major_plane->values[1] << " "
            << coefficients_major_plane->values[2] << " "
            << coefficients_major_plane->values[3] << std::endl;

  // std::cerr << "Model inliers_major_plane: " << inliers_major_plane->indices.size() << std::endl;
  // for (std::size_t i = 0; i < inliers_major_plane->indices.size(); ++i)
  //   for (const auto &idx : inliers_major_plane->indices)
  //     std::cerr << idx << "    " << cloud->points[idx].x << " "
  //               << cloud->points[idx].y << " "
  //               << cloud->points[idx].z << std::endl;

  std::vector<int> ids;
  for (const auto &idx : inliers_major_plane->indices)
  {
    ids.push_back(idx);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr major_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, ids, *major_plane_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_major_plane);
  extract.setNegative(true);
  extract.filter(*object_cloud);

  // Plane segementation for getting object plane
  pcl::ModelCoefficients::Ptr coefficients_object_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_object_plane(new pcl::PointIndices);
  planeSegmentation(object_cloud, coefficients_object_plane, inliers_object_plane);

  pcl::PointCloud<pcl::PointXYZ>::Ptr object_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  extract.setInputCloud(object_cloud);
  extract.setIndices(inliers_object_plane);
  extract.setNegative(false);
  extract.filter(*object_plane_cloud);

  // Centroid Calculation
  float x{0}, y{0}, z{0};
  for (auto &p : object_plane_cloud->points)
  {
    x += p.x;
    y += p.y;
    z += p.z;
  }

  x = x / object_plane_cloud->points.size();
  y = y / object_plane_cloud->points.size();
  z = z / object_plane_cloud->points.size();

  pcl::PointXYZ p1;
  float min_dist = 200;
  for (auto &p : object_plane_cloud->points)
  {
    float dist = std::abs(p.x - x);
    if (dist < min_dist)
    {
      min_dist = dist;
      p1 = p;
    }
  }

  //object frame calculation
  std::vector<float> obj_frame_z(3, 0.0), obj_frame_y(3, 0.0), obj_frame_x(3, 0.0);

  float norm_obj_normal = std::sqrt(std::pow(coefficients_major_plane->values[0], 2) + std::pow(coefficients_major_plane->values[1], 2) + std::pow(coefficients_major_plane->values[2], 2));
  if (coefficients_major_plane->values[2] < 0)
  {
    obj_frame_z[0] = -coefficients_major_plane->values[0]/norm_obj_normal;
    obj_frame_z[1] = -coefficients_major_plane->values[1]/norm_obj_normal;
    obj_frame_z[2] = -coefficients_major_plane->values[2]/norm_obj_normal;
  }
  else
  {
    obj_frame_z[0] = coefficients_major_plane->values[0]/norm_obj_normal;
    obj_frame_z[1] = coefficients_major_plane->values[1]/norm_obj_normal;
    obj_frame_z[2] = coefficients_major_plane->values[2]/norm_obj_normal;
  }

  float distance = std::sqrt(std::pow(p1.x - x, 2) + std::pow(p1.y - y, 2) + std::pow(p1.z - z, 2));
  obj_frame_y[0] = (p1.x - x) / distance;
  obj_frame_y[1] = (p1.y - y) / distance;
  obj_frame_y[2] = (p1.z - z) / distance;
  cross_product(obj_frame_y, obj_frame_z, obj_frame_x);

  Eigen::Matrix4f TCam2Obj;
  TCam2Obj << obj_frame_x[0], obj_frame_y[0], obj_frame_z[0], x,
      obj_frame_x[1], obj_frame_y[1], obj_frame_z[1], y,
      obj_frame_x[2], obj_frame_y[2], obj_frame_z[2], z,
      0.0, 0.0, 0.0, 1.0;

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    listener.waitForTransform("/world", "/panda_camera_optical_link", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("/world", "/panda_camera_optical_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  tf::Matrix3x3 R(transform.getRotation());
  Eigen::Matrix4f TWorld2Cam;
  TWorld2Cam << R[0][0], R[0][1], R[0][2], transform.getOrigin().x(),
  R[1][0], R[1][1], R[1][2], transform.getOrigin().y(),
  R[2][0], R[2][1], R[2][2], transform.getOrigin().z(),
  0.0, 0.0, 0.0, 1.0;


  Eigen::Matrix4f TWorld2Obj;
  TWorld2Obj = TWorld2Cam * TCam2Obj;

  std::cout << TWorld2Obj << std::endl;
  
//  
  tf::TransformListener listener2;
  tf::StampedTransform transform2;
  try
  {
    listener2.waitForTransform("/panda_rightfinger", "/panda_link7", ros::Time(0), ros::Duration(1.0));
    listener2.lookupTransform("/panda_rightfinger", "/panda_link7", ros::Time(0), transform2);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  tf::Matrix3x3 R2(transform2.getRotation());
  Eigen::Matrix4f TObj2link7;
  TWorld2Cam << R2[0][0], R2[0][1], R2[0][2], transform2.getOrigin().x(),
  R2[1][0], R2[1][1], R2[1][2], transform2.getOrigin().y(),
  R2[2][0], R2[2][1], R2[2][2], transform2.getOrigin().z(),
  0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix4f TWorld2link7;
  TWorld2link7 = TWorld2Obj * TObj2link7;

  std::cout << TWorld2link7 << std::endl;
//

  geometry_msgs::PoseStamped target_pose;  
  target_pose.pose.position.x = (double) TWorld2link7(0,3);
  target_pose.pose.position.y = (double) TWorld2link7(1,3);
  target_pose.pose.position.z = (double) TWorld2link7(2,3);
  Eigen::Matrix3f rot;
  rot << TWorld2link7(0,0), TWorld2link7(0,1), TWorld2link7(0,2),
         TWorld2link7(1,0), TWorld2link7(1,1), TWorld2link7(1,2),
         TWorld2link7(2,0), TWorld2link7(2,1), TWorld2link7(2,2);
        
  std::cout << rot << std::endl;
  Eigen::Quaternionf q(rot);
  target_pose.pose.orientation.x = q.x();
  target_pose.pose.orientation.y = q.y();
  target_pose.pose.orientation.z = q.z();
  target_pose.pose.orientation.w = q.w();

  target_pose.header.frame_id = "world";
  target_pose.header.stamp = ros::Time::now();
  pose_pub.publish(target_pose);
  
  pcl::visualization::PCLVisualizer::Ptr viewer;
  // viewer = simpleVis(object_plane_cloud);

  // while (!viewer->wasStopped())
  // {
  //   viewer->spinOnce(100);
  //   std::this_thread::sleep_for(1ms);
  // }

  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planeSegmentation");
  ros::NodeHandle n;
  ros::Subscriber pcl_sub = n.subscribe("/panda_camera/depth/points", 1, cloud_cb);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/target_pose", 1);
  ros::spin();
  return 0;
}
