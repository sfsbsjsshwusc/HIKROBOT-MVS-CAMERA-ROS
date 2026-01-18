# HIKROBOT-MVS-CAMERA-ROS
The ros driver package of Hikvision Industrial Camera SDK. Support configuration parameters, the parameters have been optimized, and the photos have been transcoded to rgb format.
Please install mvs, https://blog.csdn.net/weixin_41965898/article/details/116801491

# Install
```
mkdir -p ~/ws_hikrobot_camera/src
git clone https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS.git ~/ws_hikrobot_camera/src/hikrobot_camera
cd ~/ws_hikrobot_camera
catkin_make
```
# launch run
```
source ./devel/setup.bash 
roslaunch hikrobot_camera hikrobot_camera.launch
```
# launch run
use rviz subscribe topic： /hikrobot_camera/rgb
```
source ./devel/setup.bash 
roslaunch hikrobot_camera hikrobot_camera_rviz.launch
```

## GigE 2.0 / IEEE1588 (PTP) 同步建议
建议使用支持 IEEE1588（PTP 同步）的 GigE 2.0 相机，配合 linuxptp 可以让多台设备与主机时钟同步。
使用 GigE 2.0 的优势是无需修改驱动或增加外部触发硬件；相机只需一根网线即可完成供电、数据传输与时钟同步。
