### Description


ROS 2 package for receiving CRSF (RC channels values) packets over serial port (UART).

Topics
    rc/channels - received rc channels values
    rc/link - connection statistics information


### Installation


Let's assume that your ros 2 workspace localized at `~/row2_ws/`.


#### 1. Clone package from git:

```bash
cd ~/row2_ws/src
git clone
```

#### 2. Build

```bash
cd ~/row2_ws
colcon build --packages-select ros2_crsf_receiver
```

#### 3. Re-source

```bash
source ~/row2_ws/install/setup.bash
```


### Running


#### Set up params:

1. Serial device name: `device`, default is `/dev/ttyUSB0`
2. Baud rate: `baud_rate`, default is `425000`
3. Enable / Disable link statistics info: `link_stats`, default is `false`
4. Receiver rate (hz): `receiver_rate`, default is `100`


#### Run ros node:

```bash
# Run Node with default parameters
ros2 run crsf_receiver crsf_receiver_node

# Or setup and run Node with custom parameters values:
ros2 run crsf_receiver crsf_receiver_node --ros-args -p "device:=/dev/ttyUSB1" -p baud_rate:=420000  -p link_stats:=true
```