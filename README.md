<!--
 * @Author: Jingsheng Lyu
 * @Date: 2021-01-15 10:59:13
 * @LastEditors: Jingsheng Lyu
 * @LastEditTime: 2021-02-19 20:50:32
 * @FilePath: /robotics/Robotics/README.md
 * @Github: https://github.com/jingshenglyu
 * @Web: https://jingshenglyu.github.io/
 * @E-Mail: jingshenglyu@gmail.com
--> 


# Robotics
A Robotics course of TU Berlin WiSe20/21

From the chapter 1 to chapter 3 we use pumasim to implement the simulation. 
Pumasim is not a open source software. If you want to use it, please contact with the owner. 
Our software is from https://www.robotics.tu-berlin.de/menue/home/.

--------------------------------------------------------------------------------------------

# RBO Lab's Puma simulator based on Stanford's Puma simulator


This directory contains the precompiled simulator

Before you can use the simulator, you have to install it.


## Installing the simulator on Ubuntu:
If you want to install the simulator instead of using the VM image, you can try the following instructions:

* Install simulator dependencies:
        sudo apt-get install libnewmat10-dev libqhull-dev libopencv-dev cmake libsoil1 libglew2.1
* copy the simulator archive file to the computer
* unpack the simulator: 
        tar -xzvf pumasimulator.tgz
* run the script:
        cd pumasimulator
        ./install.sh 
  to install the simulator in /opt/pumasim and provide the command "pumasim"

## Running the simulator

You can call the simulator anywhere using the command ```pumasim```
       
When started, the simulator looks for two libraries: one contains controllers (libcontrolDLL.so, mandatory), the other the vision loop (libcv_main.so, optional).

The simulator first checks the current directory for these libraries and loads them. 
If loading fails, the simulator loads the example libcontrolDLL.so provided in /opt/pumasim instead.


## Compiling a libcontrolDLL.so

You will find the code to create the controller and vision loop libraries in the exercise packages.
Each exercise is set using cmake and compiled using make.
The compilation system setup follows the canonical use of cmake:

```
cd 1/  #change into the exercise 1 directory
mkdir build && cd build
cmake ..
```

Once that is done, you can compile a controller library with:

```
make
```

To load the controller library into the puma simulator, simply run:

```
pumasim
```

in the build directory.
