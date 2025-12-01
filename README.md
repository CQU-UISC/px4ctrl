<a id="readme-top"></a>
<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/CQU-UISC/px4ctrl">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>
  <h3 align="center">UISC Lab Px4Ctrl</h3>
  <p align="center">
    Px4Ctrl
  </p>
  <img align="center" src=https://img.shields.io/badge/license-GPL--3.0-blue  alt="license"/>
  
</div>

<!-- ABOUT -->
## About
ROS2 Wrapper for The Px4 Autopilot

<!-- GETTING STARTED -->
### Prerequisites
To use this repository, you are required to install both  [px4ctrl_client](https://github.com/Luxru/px4ctrl_client.git) and [px4ctrl_msgs](https://github.com/Luxru/px4ctrl_msgs.git).

### Dependencies
- [format](https://github.com/fmtlib/fmt) 
- [C++ 20](https://en.cppreference.com/w/cpp/compiler_support)
  

### Architecture
```
+-----------------------+
|   Flight Controller   |
|     (PX4 Firmware)    |
+-----------+-----------+
            ^
            | MAVLink (UART/TCP)
            v
+-----------+-------------------------------------------------------+
|             Onboard Computer (ROS 2 Environment)                  |
|                                                                   |
|   +--------------+         ROS 2         +-------------------+    |
|   | mavlink_node | <===================> |      px4ctrl      |    |
|   +--------------+       (Odom/Ctrl)     +---------+---------+    |
|                                                    ^              |
|                                              ROS 2 |              |
|   +--------------------+                           |              |
|   | custom_controllers | <=========================+              |
|   +--------------------+                                          |
|                                                                   |
|   .............................................................   |
|                                                                   |
|                      ZMQ (IPC/TCP)                                |
|                            |                                      |
|                            v                                      |
|                  +------------------+                             |
|                  | Zmq Proxy Server |                             |
|                  | (Current Deployment)                           |
|                  +---------+--------+                             |
+----------------------------+--------------------------------------+
                             ^
                             | ZMQ (Network/WiFi)
                             v
+----------------------------+--------------------------------------+
|                       Laptop (Client)                             |
|                                                                   |
|                  +------------------+                             |
|                  |  px4ctrl_client  |                             |
|                  +------------------+                             |
+-------------------------------------------------------------------+
```


### Installation
```
git clone https://github.com/CQU-UISC/px4ctrl.git
cd px4ctrl
git submodule update --init --recursive
mkdir build && cd build
cmake ..
make -j4
```

<!-- CONTACT -->
## Contact
Xu Lu - lux@cqu.edu.cn

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments
* [ZJU FastLab](https://github.com/ZJU-FAST-Lab)
* [UZH Robotics and Perception Group](https://github.com/uzh-rpg)
