# VRX in Docker

ROS: humble

Gazebo: garden

## 一、准备

### 1. 安装docker，[官方教程](https://docs.docker.com/engine/install/ubuntu/)

```bash
# 添加docker官方公钥:
udo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# 添加apt源:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 安装docker
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### 2. 安装rocker工具，[官方教程](https://github.com/HonuRobotics/dockwater/wiki/Install-Dependencies#step-3-install-rocker)

```bash
# 运行成功的前提是添加过ROS的公钥和软件源，成功安装过ROS也行
sudo apt install python3-rocker 
```

### 3. 构建镜像

```bash
cd ~
git clone XXX 
cd ~/vrx_ws
docker build -t vrx_sim:humble .
```

## 二、使用

### 1. 创建并进入容器

#### 1.1. 集成核显卡

```bash
rocker --devices /dev/dri/ --group-add video --group-add 110 --x11 --user --home  --network host --name vrx_sim vrx_sim:humble bash
```

#### 1.2. Nvidia独立显卡

```bash
rocker --nvidia --x11 --user --home  --network host --name vrx_sim vrx_sim:humble bash
```

### 2. （容器中）编译代码

```bash
cd ~/vrx
source /opt/ros/humble/setup.bash
colcon build --merge-install
. install/setup.bash
```

### 3. 启动vrx的仿真环境，更详细的教程可参考[vrx](https://github.com/osrf/vrx/wiki/tutorials)

#### 3.1. 定点控制

##### 3.1.2. 启动仿真环境（容器中）

```bash
cd ~/vrx
source /opt/ros/humble/setup.bash
. install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=stationkeeping_task
```

##### 3.1.2. 打开新终端进入容器，运行inverse_kinematics

```bash
docker exec -it vrx_sim bash
source /opt/ros/humble/setup.bash
cd ~/vrx
. install/setup.bash
ros2 run wamv_ctl inverse_kinematics
```

##### 3.1.3. 打开新终端进入容器，运行station_keeping

```bash
docker exec -it vrx_sim bash
source /opt/ros/humble/setup.bash
cd ~/vrx
. install/setup.bash
ros2 run wamv_ctl station_keeping
```

#### 3.2. 跟踪路径-PID

##### 3.2.1. 启动仿真环境（容器中）

```bash
cd ~/vrx
source /opt/ros/humble/setup.bash
. install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=wayfinding_task
```

##### 3.2.2. 打开新终端进入容器，运行inverse_kinematics

```bash
docker exec -it vrx_sim bash
source /opt/ros/humble/setup.bash
cd ~/vrx
. install/setup.bash
ros2 run wamv_ctl inverse_kinematics
```

##### 3.2.3. 打开新终端进入容器，运行station_keeping

```bash
docker exec -it vrx_sim bash
source /opt/ros/humble/setup.bash
cd ~/vrx
. install/setup.bash
ros2 run wamv_ctl wayfinding
```

## 三、调整环境参数

### 1. 调整风力

修改仿真世界中~/Everything/vrx_ws/src/vrx/vrx_gz/worlds/sydney_regatta.sdf中libUSVWind.so插件

```xml
  <plugin
    filename="libUSVWind.so"
    name="vrx::USVWind">
    <wind_obj>
      <name>wamv</name>
      <link_name>wamv/base_link</link_name>
      <coeff_vector>.5 .5 .33</coeff_vector>
    </wind_obj>
    <!-- Wind -->
    <wind_direction>240</wind_direction> <!-- 调节风向 -->
    <!-- in degrees -->
    <wind_mean_velocity>2.0</wind_mean_velocity> <!-- 调节风的强度 -->
    <var_wind_gain_constants>0</var_wind_gain_constants>
    <var_wind_time_constants>2</var_wind_time_constants>
    <random_seed>10</random_seed>
    <!-- set to zero/empty to randomize -->
    <update_rate>10</update_rate>
    <topic_wind_speed>/vrx/debug/wind/speed</topic_wind_speed>
    <topic_wind_direction>/vrx/debug/wind/direction</topic_wind_direction>
  </plugin>
```

### 2. 调整波浪

修改仿真世界中~/Everything/vrx_ws/src/vrx/vrx_gz/worlds/sydney_regatta.sdf中libPublisherPlugin.so插件

```xml
 <plugin filename="libPublisherPlugin.so" name="vrx::PublisherPlugin">
   <message type="gz.msgs.Param" topic="/vrx/wavefield/parameters"
            every="2.0">
     params {
       key: "direction"
       value {
         type: DOUBLE
         double_value: 0.0
       }
     }
     params {
       key: "gain"
       value {
         type: DOUBLE
         double_value: 0.3 <!-- 调节波浪的强度 -->
       }
     }
     params {
       key: "period"
       value {
         type: DOUBLE
         double_value: 5
       }
     }
     params {
       key: "steepness"
       value {
         type: DOUBLE
         double_value: 0
       }
     }
   </message>
 </plugin>
```

