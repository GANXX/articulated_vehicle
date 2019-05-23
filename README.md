# 铰接式铲运机在ROS系统下的建模与导航控制系统

该项目旨在创建一辆铰接式铲运机模型，在gazebo中进行建模仿真，并设计相应的环境感知，路径规划，路径跟踪控制模块。

系统使用Ubuntu 16.04 ，ros框架采用ROS-kinetic

---

## 下载和编译

1. 下载src目录至本地电脑的ros工作空间根目录下。

2. 删除本地系统的move_base, navigation （如果已经安装）

   ``` shell
   $ sudo aptitude purge ros-kinetic-move-base
   $ sudo aptitude purge ros-kinetic-navigation
   ```

   

3. 安装所需要的依赖

   ```shell
   $ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
   ```

4. 更新gazebo版本，建议版本号为 gazebo 7.15以上

   1. 查看版本方法：

      ```shell
      $ gazebo -v
      ```

   2. 升级方法：

      ```shell
      $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
      $ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
      $ sudo apt-get update
      $ sudo apt-get install gazebo7
      ```

5. 安装ros_control控制器

   ```shell
   $ sudo apt-get install ros-kinetic-velocity-controllers 
   $ sudo apt-get install ros-kinetic-position-controllers 
   ```

6. 进行编译 

   ```shell
   $ catkin_make
   ```

   编译过程中可能会缺少相应的依赖或者是系统文件，可能会碰到的问题和解决方案如下：

   >
>
   >**出现（missing: BULLET_DYNAMICS_LIBRARY...）**
   >
   >CMake Error at /usr/share/cmake-2.8/Modules/FindPackageHandleStandardArgs.cmake:108 (message):
   >Could NOT find Bullet (missing: BULLET_DYNAMICS_LIBRARY
   >BULLET_COLLISION_LIBRARY BULLET_MATH_LIBRARY BULLET_SOFTBODY_LIBRARY
   >BULLET_INCLUDE_DIR)
>Call Stack (most recent call first):
   >/usr/share/cmake-2.8/Modules/FindPackageHandleStandardArgs.cmake:315 (_FPHSA_FAILURE_MESSAGE
>
   >如果编译时遇到上诉错误，进行如下操作
   >
   >```
   >rosdep where-defined bullet
>sudo apt-get install libbullet-dev
   >```
>
   > **出现 Could not find a package configuration file provided by "tf2_sensor_msgs"**
>
   >办法：
   >
   >```
>git clone https://github.com/ros/geometry2.git
   >```
>
   >
>
   > **出现 Could NOT find SDL (missing: SDL_LIBRARYSDL_INCLUDE_DIR)**
   >
   >```
>sudo aptitude install libsdl1.2-dev
   >```
>
   > **出现 Could NOT find SDL_image (missing:SDL_IMAGE_LIBRARIES SDL_IMAGE_INCLUDE_DIRS)**
   >
   >```
>sudo apt-get install libsdl-image1.2-dev 
   >```
>
   > **出现  Could not find a package configuration fileprovided by "move_base_msgs"**
   >
   >```
>git clone https://github.com/ros-planning/navigation_msgs.git
   >```
   >
   >

   