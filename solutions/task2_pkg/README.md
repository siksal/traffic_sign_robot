**Dependencies used are:**

*  `geometry_msgs`
*  `roscpp`
*  `rospy`
*  `std_msgs`
*  `tf`
*  `cv_bridge`
*  `sensor_msgs`
*  `nav_msgs`


**Packages needed are:**

* `ros_control`: ROS packages including controller interfaces, controller managers, transmissions, etc.

    * `$ sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers`

* `opencv`: Open-source computer vision library.
    * `$ sudo apt-get install python-opencv`


## Task 2

While there are more than one approach that can be used for task two, we went for this one which uses colour and shape detection. The camera scene from the robot was passed through a colour filter to get the HSV (Hue Saturation Value) of the traffic sign. This information was then used to detect the colour, shape, and state of the traffic sign. The robot remains in its initial position when the red sign is active and upon a go-state (or the green sign detection) it keeps moving and checking whether it has reached the goal location using the pose information from the odometry, and stops immediately it gets to the goal point.



Command required to launch task two solution: <br>
` roslaunch task2_pkg task2_solution.launch `

## Challenges Faced

The major challenge from task two is keeping the robot moving as it approaches the goal location. This was a challenge because when the robot got close to the traffic sign during the go-state, it started detecting the red sign again which made the robot stop along the way. It would've been better if the traffic sign acted as a light that goes on-and-off rather than an object that docks.

Also while the robot was trying to detect the traffic sign the colour of the goal location interfered and was detected as a traffic stop sign too. This was addressed in the solution, though.