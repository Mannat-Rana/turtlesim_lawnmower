# Lawnmower Application for the ROS Turtlesim Simulator ![SHIELD](https://img.shields.io/badge/Project%20Status%3A-Complete-green?style=for-the-badge) ![ros](https://camo.githubusercontent.com/4c117e738ecff5825b1031d601ac04bc70cc817805ba6ce936c0c556ba8e14f0/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d524f5326636f6c6f723d323233313445266c6f676f3d524f53266c6f676f436f6c6f723d464646464646266c6162656c3d) ![c++](https://camo.githubusercontent.com/6301a47e098ea0b84260920a75b5a71f121c5a0b55965dff8ad80bd60db208c7/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d4325324225324226636f6c6f723d303035393943266c6f676f3d43253242253242266c6f676f436f6c6f723d464646464646266c6162656c3d)  ![PYTHON](https://camo.githubusercontent.com/3df944c2b99f86f1361df72285183e890f11c52d36dfcd3c2844c6823c823fc1/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d507974686f6e26636f6c6f723d333737364142266c6f676f3d507974686f6e266c6f676f436f6c6f723d464646464646266c6162656c3d) ![ROBOTICS](https://camo.githubusercontent.com/b8e2732eda54a502cb34a56c1ea83747134ce98754e6c49a3177cd89f411bc97/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d526f626f742b4672616d65776f726b26636f6c6f723d303030303030266c6f676f3d526f626f742b4672616d65776f726b266c6f676f436f6c6f723d464646464646266c6162656c3d)

## About
The goal of this application is to develop a lawnmower pattern with the Turtlesim Simulator that takes a starting (x, y) coordinate, a height, and a width for the pattern. This application is developed as part of the first homework assignment in the course SES 598: Autonomous Exploration Systems at Arizona State University.

## Demonstration
Starting location (x, y), height, width, and speed limits for the turtlesim can be adjusted in the [lawnmower_params.yaml](https://github.com/Mannat-Rana/turtlesim_lawnmower/blob/main/config/lawnmower_params.yaml) file.

### C++ Implementation
![cpp_lawnmower_demo](https://user-images.githubusercontent.com/82643627/151105107-15c2c6ee-359a-4eed-9277-90da1dec74b0.gif)

### Python Implementation
![py_lawnmower_demo](https://user-images.githubusercontent.com/82643627/151104975-5d24e1f1-977d-4906-ade1-e607d774e9ac.gif)

## To Install
```bash
cd ~/catkin_ws/src
git clone https://github.com/Mannat-Rana/turtlesim_lawnmower.git
catkin build turtlesim_lawnmower
cd ~/catkwin_ws
source devel/setup.bash
```
## To Run
For C++:
```bash
roslaunch turtlesim_lawnmower lawnmower_cpp.launch
```

For Python:
```bash
roslaunch turtlesim_lawnmower lawnmower_py.launch
```
