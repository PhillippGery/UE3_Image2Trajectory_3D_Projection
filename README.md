# Image to Robot Trajectory

Transform an arbitrary image into coordinates and finally into a robot trajectory. Using a real robot with an attached LED on the TPC, a 3D projection is created.

[IMG_0093](https://github.com/user-attachments/assets/0b7f7970-f63b-44d8-a445-03f283068bc2)
![IMG_0095](https://github.com/user-attachments/assets/c37b4a65-cec6-4e4c-952a-15d80a2b6adb)


```bash
ros2 launch msee22_description view_room.launch.py
```!


```bash
ros2 launch msee22_description move_robot.launch.py
```

add your csv file to the `resource` folder, build and source your workspace. Then, run 
```bash
ros2 launch msee22_description move_robot.launch.py
```
and enable the tool0 trail, too see Trajectory
