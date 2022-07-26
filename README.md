# simrobo
[![BuildAndTest](https://img.shields.io/github/workflow/status/YumaMatsumura/simrobo/build%20and%20test/master)](https://github.com/YumaMatsumura/simrobo/actions/workflows/build.yml)

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

### ジョイコン操作
```bash
ros2 launch simrobo_driver driver.launch.py
```
