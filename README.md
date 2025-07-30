# VRX in Docker

ROS: humble

Gazebo: garden

## 准备

1. 安装rocker工具，参考[教程](https://github.com/HonuRobotics/dockwater/wiki/Install-Dependencies)
1. 安装pymap3

```bash
pip3 install pymap3d
```

## 使用

1. 构建镜像

```bash
docker build -t vrx_sim:humble .
```

2. 进入容器

```bash
rocker --x11 --user --home  --network host vrx_sim:humble bash
```

3. （容器中）编译代码

```bash
source /opt/ros/humble/setup.bash
colcon build --merge-install
. install/setup.bash
```

4. （容器中）启动vrx的仿真环境，更详细的教程可参考[vrx](https://github.com/osrf/vrx/wiki/tutorials)

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```



