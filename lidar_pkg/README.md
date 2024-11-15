
In this package the Turtlebot moves without touching an obstacle. It keeps moving forward untill the lidar infront has reached the safe distance. 
Then the Turtlebot decides wich way to go. It compares the minimal values of the left and right side with eachother, the one that is bigger is the direction the Turtlebot will go to. If the Turtlebot is is close to a wall on it's left side, it will turn right a bit. Same for the right side.

## Branchstructure
The code is found uner lidar_pkg > lidar.py

- lidar_pkg
    - lidar.py

## before running the code
To run the code you need to have Ubuntu 22.04, Gazebo and Turtlebot installed. 
Make sure all your terminals you are using have the same ros domain id. You can check this by using the following command:

  ``` echo $ROS_DOMAIN_ID ```

### Make sure all the terminals have the same id !

## run the program
steps to follow: 
1) Open an Ubuntu terminal
2) Source your Terminal so it knows about the ros2 workspace and it's packages
   
   ```source ~/.bashrc```
4) Run the folowing command to open the simulation area

   ```ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py```
   
5) Open a second Ubuntu terminal
6) Run the following command to prepare your code for execution
   
   ```colcon build --packages-select lidar_pkg```
8) After the colcon build is complete you should get a message like this:
   ```
    Starting >>> lidar_pkg
    Finished <<< lidar_pkg [1.27s]

    Summary: 1 package finished [2.05s]
    ```
9) Make sure to source again after the colcon build

    ``` source install/setup.bash```
10) Run this code

    ```ros2 launch lidar_pkg lidar_pkg_launch_file.launch.py```

Now you should see the Turtlebot moving in the Gazebo environment. If not, make sure to check the terminals for any error's.

   
   
      






