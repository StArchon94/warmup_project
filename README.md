# Warmup Project
## Driving in a Square
* Description

    The goal is to write a ROS node that commands the Turtlebot3 robot to drive a square path. I accomplish this by using timed moving, i.e., move forward for a fixed amount of time then make a 90 degree turn four times.
* Code explanation

    `DriveSquare` class is the ROS node that drives the robot in a square. Its `__init__` method starts the rospy node, gets a publisher to the `/cmd_vel` topic and defines several `Twist` variables that will be used to control the robot movement. The `run` method is called once the node is up. It implements the behavior as detailed in Description. To make it more stable, the robot is also commanded to stop for 1 sec between forward movement and turning.
* GIF

    ![drive_square](gifs/drive_square.gif)
## Person Follower
* Description

    The goal is to write a ROS node that commands the Turtlebot3 robot to follow a person (or its closets object) while maintaining a safe distance. I accomplish this by
    1. computing the distance & azimuth of the closest point
    2. computing the error between the desired distance & azimuth and the current ones
    3. applying a P-controller to set the robot's linear and angular velocity
    4. additionally, the robot's linear velocity is set to 0 when the azimuth error is not small enough
* Code explanation

    `PersonFollower` class is the ROS node that drives the robot to follow a person. Its `__init__` method starts the rospy node, gets a subscriber to the `/scan` topic, a publisher to the `/cmd_vel` topic and defines several variables that will be used to control the robot movement. The `scan_callback` method is called every time the node receives new data from `/scan`. It implements the behavior as detailed in Description. To make it more stable, the `ranges` data from `/scan` is mean-filtered, as in `process_range_data` method in `utils.py`. Extra care has been taken to make sure invalid ranges are properly handled.
* GIF

    ![person_follower](gifs/person_follower.gif)

## Wall Follower
* Description

    The goal to write a ROS node that commands the Turtlebot3 robot to
    1. navigate close to a wall
    2. drive alongside the wall at an approximately fixed distance
    3. handle corners

    I accomplish this by
    1. computing the distance & azimuth of the closest point
    2. computing the error between the desired distance & azimuth and the current ones. In this case, the desired azimuth is set to -90 degrees so that the robot always drives counter-clockwise
    3. setting the robot's linear velocity to a constant. However, it is only enabled when the azimuth error is small enough
    4. applying a P-controller to set the robot's angular velocity. Specifically, it is affected by both distance and azimuth error
* Code explanation

    `WallFollower` class is the ROS node that drives the robot to follow a wall. Its `__init__` method starts the rospy node, gets a subscriber to the `/scan` topic, a publisher to the `/cmd_vel` topic and defines several variables that will be used to control the robot movement. The `scan_callback` method is called every time the node receives new data from `/scan`. It implements the behavior as detailed in Description. `process_range_data` method in `utils.py` is reused from above.
* GIF

    ![wall_follower](gifs/wall_follower.gif)
# Challenges
1. `ranges` data from `/scan` can be a bit noisy, which makes the robot behavior unstable. I overcame it by applying a mean-filter to the data.
2. The gain coefficients for P-controllers are not provided. The magnitudes of them have a large impact on the actual performance of the robot. I set them by experimenting with different combinations and then chose the empirically best-performing ones.
# Future work
The *Drive in a Square* part is done without any active control. It turns out that the robot cannot drive in perfect squares, and will drift over time. If a map is given, the robot may use measurement data to update its state, and thus remedying its square trajectory. Even more ambitiously, SLAM may be first used to create such a map.
# Takeaways
* Check ROS network.
    
    To connect to the robot from another machine, always make sure that machine and the robot can reach each other through networking. This usually requires them to use the same WiFi. Also make sure that environment variables such as `ROS_MASTER_URI` and `ROS_HOSTNAME` are properly set.
* Utilize the Gazebo simulator. 

    Although robot behavior usually will not exactly transfer from simulator to real-world, I find that Gazebo is still a great tool when I am prototyping the code framework. It saves a lot of trouble of having to physically grab a robot or move oneself. After passing the test in a simulator, all that is left is tuning the parameters to make it work in real-world.
