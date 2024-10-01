# Activities for the ROS module of TE3003B (Robotics and Intelligent Systems Integration)

Code by: Francisco José Ramírez Aldaco \
Teaching professor: Dr. Arturo Eduardo Cerón López \
ROS version: ROS 2 Jazzy Jalisco

## ROS Activity \#1: Talker and listener nodes, car moving in a circle (written in C++)

### Talker and listener nodes (written following [this](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) tutorial)

To build the example:\
`colcon build --packages-select cpp_pubsub`

To run talker node:\
`ros2 run cpp_pubsub talker`

To run listener node:\
`ros2 run cpp_pubsub listener`

### Moving car example (written following [this](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html) tutorial)

To build the example:\
`colcon build --packages-select urdf_test`

To launch and visualize static robot with manually movable wheels:\
`ros2 launch urdf_test display.launch.py`

To launch robot moving in a circle:\
`ros2 launch urdf_test circle.launch.py`

To visualize robot moving in a circle:\
`rviz2 -d install/urdf_test/share/urdf_test/config/rob_pub.rviz`