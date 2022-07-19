# simrobo
## 説明
本パッケージは、差動二輪ロボットをROS2でシミュレートしたいときに使用できるものである。

## 動作環境
 - ROS2 Foxy
 
## 準備
```bash
sudo apt install ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui ros-foxy-xacro ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gazebo-ros2-control
```

## 実行方法
### RViz2描画
```bash
ros2 launch simrobo_description display.launch.py
```
