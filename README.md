```bash
git submodule add -b humble https://github.com/ros-perception/image_transport_tutorials.git
git submodule add https://github.com/RobotWebTools/rosbridge_suite.git
```

```bash
cd ~/ros2_ws/src
git clone https://github.com/tstaisyu/video_stream_publisher.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Terminal 1
```bash
ros2 run image_transport_tutorials publisher_from_video 0
```

Terminal2
```bash
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

ビデオストリームの確認
sample_browser_subscriber.htmlをブラウザで開く