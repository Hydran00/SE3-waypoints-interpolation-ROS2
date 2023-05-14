# SE3-waypoints-interpolation-ROS2
A ROS2 package for creating a 3d smooth trajectory that interpolates the position togeter with the orientation. It takes in input a list of waypoint in a ``geometry_msgs::msg::PoseArray`` reading a topic called ``/desired_waypoints`` and compute a **smooth trajectory** that connect every points in the same order.  
The trajectory then will be published under ``/computed_trajectory`` using another ``geometry_msgs::msg::PoseArray``.  
Waypoint are added to the array in a way such that a **maximum velocity threshold will never be overcomed**. 
You can change the maximum velocity parameters in the c++ file:  
  ```
  #define MAX_VELOCITY <max_velocity_value>
  ```

In order to compute trajectories that starts from the robot current configuration it is needed that the tool center point of the robot is published, that's why there is also a tf_broadcaster that is launched together with the interpolator node. If you are not interested in this feature you can comment the relative control inside ``interpolate.cpp``.  

1. Install [pinocchio](https://github.com/stack-of-tasks/pinocchio)  
    ```
    sudo apt install ros-$ROS_DISTRO-pinocchio
    ```
2. Change the launch file parameter with the name of the last link of your robot:
    - Go in ``/launch`` and open ``interpolate.launch.py``
    - Change frame_name with the name of the link that is used for the inverse kinematics/dynamics   
    ``frame_name = "wrist_3_link"``
3. Clone this repo in your workspace src folder:   
    ```
    git clone https://github.com/Hydran00/SE3-waypoints-interpolation-ROS2.git
    ```
4. Build the package:  
    ```
    colcon build --packages-select interpolator
    ```
5. Run the node:  
    ```
    ros2 launch interpolator interpolate.launch.py
    ```
    



