# ROS 2 ICM42688 driver

节点自动查找`icm42688-accel`、`icm42688-gyro`和`icm42688-imu`。前两个设备
配置采样频率、滤波并启动硬件FIFO；节点只读取组合设备并发布消息。

`poll(2)`只等待内核IIO buffer可读；传感器数据由
`ICM42688 INT2 -> GPIO0_PC3 -> Linux IRQ -> IIO kfifo`产生，不轮询寄存器。

默认`timestamp_clock=monotonic`，ROS消息强制使用内核在ICM42688硬IRQ上半部
取得、再由FIFO/ODR模型反推到各样本的IIO时间戳。不会使用用户态`read()`或ROS
发布时刻；无效时间戳的样本会被丢弃。该时间轴与SC035 V4L2一致，秒数应接近
`/proc/uptime`，而不是Unix时间。

内核只为同一个硬件FIFO packet中的accel XYZ和gyro XYZ生成一条24字节组合
scan。ROS不再维护双队列或进行用户态时间配对。scan中的唯一时间戳以硬IRQ
上半部取时为锚点，由内核FIFO/ODR模型反推到样本；两路ODR必须相同。

```bash
cd /root/icm42688_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch icm42688_driver icm42688.launch.py
ros2 topic hz /imu/data_raw
```
