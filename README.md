```bash
git submodule add -b humble https://github.com/ros-perception/image_transport_tutorials.git
git submodule add https://github.com/RobotWebTools/rosbridge_suite.git
```

# How to setup

```bash
cd ~/ros2_ws/src
git clone https://github.com/tstaisyu/video_stream_publisher.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-${ROS_DISTRO}-image-transport-plugins
colcon build
source install/setup.bash
```
参考：
https://zenn.dev/katsuitoh/articles/3585925a819b7d
https://index.ros.org/r/v4l2_camera/

## Implementation

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

# rosbridge_suiteとReact

参考：https://zenn.dev/tasada038/articles/90530ccec33619

* 必要な準備
```bash
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

sudo apt install nodejs
nodejs -v
npm -v # もしなければ sudo apt install npm

curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash # nvmのインストール
nvm node install # Node.jsのアップデート

# React app作成
sudo npm i -g create-react-app
create-react-app ros2_data_pubsub_app
cd ros2_data_pubsub_app
npm start

# ライブラリインストール
npm install roslib
npm install react-bootstrap bootstrap
npm install --save chart.js react-chartjs-2
npm install three @types/three @react-three/fiber
```

# 画像形式変換

参考：https://qiita.com/shoichi4411/items/e0aa33d9ce286485c5a8