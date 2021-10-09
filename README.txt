Running the simulation: roslaunch panda_simulation panda_simulation.launch
Installing potentially missing dependencies (run outside the src folder of the workspace): rosdep install --from-paths --ignore-src ./ -y

To view the camera output, look for topics (rostopic list) under the /panda_camera namespace:
RGB PointCloud: /panda_camera/depth/points
Image:          /panda_camera/depth/image_raw
NOTE: Both depth and rgb data are published to the above Image topic, to figure out which is which look for the "encoding" in the received sensor_msgs/Image msg
      "bgr8" is the rgb image, while "32FC1" is the depth image

Note that the simulation starts paused to give the controllers time to connect to the robot, which prevents the robot from potentially falling over once the simulation starts

Tutorials:
C++: https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html
Python: https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
