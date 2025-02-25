# ROS2+Choreonoid使った移動ロボットの勉強用リポジトリ
## 環境セットアップ
- 検証済みのホストはUbuntu24.04.02LTSのみ
- CLI版dockerを公式の方法に沿って入れている
### 立ち上げ
- 1. `docker compose up -d`
- 2. コンテナ内に入る
- 3. `cd ros2_ws/src`
- 4. `chmod 777 run.sh`
- 5. `./run.sh`

### 備考
- 上記の操作で、GUI利用可能なコンテナでROS2とchoreonoidが動く環境がセットアップできる
    - `ros2 run  choreonoid_ros choreonoid`でchoreonoidが立ち上がれば成功
- hostのros_src以下はコンテナ内のros2_ws/srcと共有されている
### 参考
- [ros_handson](https://ouxt-polaris.github.io/ros_handson/how_to_setup/)
- [s-nakaokaさんのチュートリアル](https://github.com/choreonoid/choreonoid_ros2_mobile_robot_tutorial/tree/main)