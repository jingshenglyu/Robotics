<!--
 * @Author: Jingsheng Lyu
 * @Date: 2021-02-09 22:07:49
 * @LastEditors: Jingsheng Lyu
 * @LastEditTime: 2021-02-09 22:07:50
 * @FilePath: /Robotics/AI_for_Robotics/README.md
 * @Github: https://github.com/jingshenglyu
 * @Web: https://jingshenglyu.github.io/
 * @E-Mail: jingshenglyu@gmail.com
-->
# Install python 3.8.5 virtual environment

* OS: Ubuntu 20.04
* IDE: VS Code

## Here I wanna to show

* How to install python 3.8.5 virtual environment on Ubuntu 20.04.
* How to create
* Activate and deactivate

1. Install the tools

    * Check the python version with `python3 --version`
  
        ```bash
        Python 3.8.5
        ```

    * Install python-dev with `sudo apt-get install python3.8-dev`

    * Install python-venv with `sudo apt-get install python3.8-venv`

2. Create a virtual env

    ```bash
    mkdir env/
    python3.8 -m venv /home/jingsheng/robotics/Robotics/AI_for_Robotics/env/
    ```

3. Activate the virtual env

    ```bash
    source env/bin/activate
    ```

4. Deactivate

    ```bash
    deactivate
    ```