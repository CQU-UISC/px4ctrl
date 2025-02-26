<a id="readme-top"></a>
<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/CQU-UISC/px4ctrl_client">
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
基于Mavros实现的Px4控制器

<!-- GETTING STARTED -->
## Getting Started
TODO

### Prerequisites
- [spdlog](https://github.com/gabime/spdlog) >=v1.14.1 
- [GLFW](https://github.com/glfw/glfw)  == 3.4 
- OpenGL >= 3.3
- [format](https://github.com/fmtlib/fmt) 
- [C++ 20](https://en.cppreference.com/w/cpp/compiler_support)

### Installation
```
git clone https://github.com/CQU-UISC/px4ctrl.git
cd px4ctrl
git submodule update --init --recursive
mkdir build && cd build
cmake ..
make -j4
```

<!-- USAGE EXAMPLES -->
## Usage
TODO

<!-- ROADMAP -->
## Roadmap

- [ ] 添加MPC控制器
- [ ] 添加安全控制方法（odom timeout等状况，紧急；降落等）
- [ ] move zmq proxy
- [x] 存在Bug， forece hover 姿态控制会出问题
- [x] 重新起飞时，重设油门估计


<!-- CONTACT -->
## Contact
Xu Lu - lux@cqu.edu.cn

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments
* [ZJU FastLab](https://github.com/ZJU-FAST-Lab)
* [UZH Robotics and Perception Group](https://github.com/uzh-rpg)
