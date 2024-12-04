I have prepared a package that creates and combines 2D lidar and encoder odoms in order to create odom,
which is the most important theory in robot operating systems.
In this way, position estimation, which is a must for SLAM algorithm,
will be provided with better and more consistent data.

To start the package: ros2 launch odom_merge odom_merge_launch.py

In the package, it is preferred to read Rplidar s2 model and encoder data via serial port.
For questions or any problems, you can reach mirackurtak7@gmail.com.
