<!--
 * @Author: Jingsheng Lyu
 * @Date: 2021-01-16 09:22:31
 * @LastEditors: Jingsheng Lyu
 * @LastEditTime: 2021-01-16 11:46:18
 * @FilePath: /workspace_assignment4/README.md
 * @Github: https://github.com/jingshenglyu
 * @Web: https://jingshenglyu.github.io/
 * @E-Mail: jingshenglyu@gmail.com
-->

## How to use this folder?
1. Create a new workspace and make
    > cd workspace_assignment4
    > catkin_make
* If you have some error like the followed error, please 'sudo apt-get install 
```
...
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'localization'
-- ==> add_subdirectory(localization)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- localization: 3 messages, 1 services
-- +++ processing catkin package: 'mapping'
-- ==> add_subdirectory(mapping)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- +++ processing catkin package: 'rbo_create'
-- ==> add_subdirectory(rbo_create)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- rbo_create: 1 messages, 3 services
CMake Warning at /opt/ros/noetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'python-serial' but neither
  'python-serial_INCLUDE_DIRS' nor 'python-serial_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/noetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  rbo_create/CMakeLists.txt:40 (catkin_package)


-- +++ processing catkin package: 'create_gui'
-- ==> add_subdirectory(create_gui)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
CMake Error at /usr/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake:146 (message):
  Could NOT find wxWidgets (missing: wxWidgets_LIBRARIES
  wxWidgets_INCLUDE_DIRS) (Required is at least version "2.8")
Call Stack (most recent call first):
  /usr/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake:393 (_FPHSA_FAILURE_MESSAGE)
  /usr/share/cmake-3.16/Modules/FindwxWidgets.cmake:1008 (find_package_handle_standard_args)
  create_gui/CMakeLists.txt:31 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/jingsheng/workspace_assignment4/build/CMakeFiles/CMakeOutput.log".
See also "/home/jingsheng/workspace_assignment4/build/CMakeFiles/CMakeError.log".
Invoking "cmake" failed
```

    After that, if you need
    > sudo apt-get update

2. cmake again
    > catkin_make
    * If you have also this error, please 'catkin_make -j1'
```
...
/home/jingsheng/workspace_assignment4/src/create_gui/src/create_move.cpp:6:10: fatal error: rbo_create/Tank.h: error no such file or directory
    6 | #include "rbo_create/Tank.h"
      |          ^~~~~~~~~~~~~~~~~~~
compilation terminated.
make[2]: *** [create_gui/CMakeFiles/button_move.dir/build.make:76：create_gui/CMakeFiles/button_move.dir/src/create_move.o] error 1
...
```

3. finished 'build'
```
...
[ 69%] Built target rbo_create_generate_messages_py
[ 77%] Built target rbo_create_generate_messages_eus
[ 83%] Built target rbo_create_generate_messages_lisp
[ 83%] Built target rbo_create_generate_messages
[ 88%] Built target map_planner
[ 93%] Built target map_view
[ 95%] Building CXX object create_gui/CMakeFiles/button_move.dir/src/create_move.o
[ 96%] Building CXX object create_gui/CMakeFiles/button_move.dir/src/create_status.o
[ 98%] Linking CXX executable /home/jingsheng/workspace_assignment4/devel/lib/create_gui/button_move
[100%] Built target button_move
```

4. begin to roslaunch, don't forget to source
  > source devel/setup.bash

## Run the simulation

***NOTE: The below instructions are NOT applicable to WS20/21 because it is online. But in case you were to record data to be run on the Create robots, you would follow the below instructions.***

All of the below commands need to be opened in separate terminals. Also make sure sources are built, in the workspace (`catkin_make`)

Start ROS master
```
roscore
```

Start lasers
Make sure the lasers are plugged in to the netbook (silver cables)
```
roslaunch rbo_create laser.launch
```

Start driver
```
rosrun rbo_create driver.py
```
The base will make a few beeps. The base is ready to go only after it plays an "I'm ready" song

Publish static transforms
```
roslaunch rbo_create publish_static_tfs.launch
```

Launch teleoperation node
You are expected to use a keyboard plugged into the USB hub on the robot. You can also accomplish teleoperation with another PC running ROS. But this is involved. Ask supervisors for details
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
The controls are printed on the command line. It basically allows you to use 9 keys to go forward/backward, spin, go in a curved path and halt. Remember, using the default speeds might be too fast. You can change that setting too.

Record bagfile
Change to a directory with reasonable disk space and use the following commands. If the netbook does not have much space left, you can use a thumb drive via the USB hub
```
# To record all topics
rosbag record -a

# To record only the required topics e.g., for making small files
rosbag record /cmd_vel /odom /scan /sensorPacket /tf
```

Now you can use the recorded bagfile for mapping.


    