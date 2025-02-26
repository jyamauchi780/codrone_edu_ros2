# ROS2 Driver of CoDrone Edu

[English Version README](https://github.com/jyamauchi780/CoDroneEdu-ROS2/blob/main/README.md)

## 概要
- Robolink から発売されている CoDrone Edu をROS2で使用するための非公式パッケージです．
- ROS2 Humble用に開発しており，その他のバージョンでの動作は確認していません．
- 安全には十分配慮して利用してください．

## システム要件
- Robolink CoDrone Edu
- Ubuntu 22.04
- ROS2 Humble 
- モーションキャプチャシステム（velocity observerを使う場合）
- 免責事項: 本プロジェクトはRobolinkとは一切関係がなく、公式のサポートは受けていません。利用は自己責任で行ってください。

## インストール
### 実行に必要なPythonパッケージのインストール
```sh
    $ sudo apt install ros-humble-tf-transformations
    $ pip install -r requirements.txt
```
### ソースのビルド
```sh
    $ mkdir -p ~/ros2_ws/src/
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/jyamauchi780/CoDroneEdu-ROS2.git
    $ cd ~/ros2_ws
    $ colcon build --symlink-install
    $ . ~/ros2_ws/install/setup.bash
```

## CoDrone Edu スマートコントローラのデバイス名の固定
複数台のCoDrone Eduを１台のPCに接続して同時に制御することを想定しているため，スマートコントローラのデバイス名を異なる名前で指定して固定する必要がある．
もしCoDrone Eduを１台しか使わず，以下の設定をすることを避けたければ，使用するlaunchファイルで
```python
    'robot_dev', default_value='CoDrone1',
```
を
```python
    'robot_dev', default_value='/dev/ttyACM0',
```
と修正する．

### デバイス名の固定方法
1. CoDrone Eduに指令を送るUbuntu PCにスマートコントローラを接続し，ターミナルで以下を実行．
```sh
    $ sudo dmesg
```
SerialNumberの行に表示されるスマートコントローラのシリアル番号を確認する．
2. `/etc/udev/rules.d/` に `99-seed-serial.rules` というファイルを作成し，以下を記述する．
```sh 
    SUBSYSTEM=="tty", ATTRS{serial}=="***", SYMLINK+="CoDrone1", MODE="0666"
```
`***` には1. で調べたシリアル番号を入力し，`SYMLINK+`には適当な名前を入力する．
`SYMLINK+="CoDrone1"`とした場合は次の手順は不要
3. 使用するlaunchファイルで，
```python
    'robot_dev', default_value='CoDrone1',
```
の default_valueにSYMLINK+に指定した名前を入力する．

## デモンストレーション
1. Ubuntu PCにMicro USBケーブルでスマートコントローラを接続し，CoDrone Eduにバッテリーを挿入する．
2. 以下の実行によりスマートコントローラでCoDrone Eduを操作できる．
```sh
    $ ros2 launch codrone_ros2_driver joy_teleop.launch.py robot_dev:=CoDrone1
```
`robot_dev`には固定したデバイス名を指定する．
デバイス名を固定していない，もしくは`CoDrone1`にしている場合は省略可．
また，CoDrone Eduを積分器として扱えるように設計した制御系を利用する場合は以下を実行．
```sh
    $ ros2 launch codrone_ros2_driver joy_teleop_as_integrator.launch.py robot_dev:=CoDrone1
```

## スマートコントローラのキー設定
### ジョイスティック
#### 左ジョイスティック
- 上下：前後方向の並進（x軸）
- 左右：左右方向の並進（y軸）

#### 右ジョイスティック
- 上下：上下方向の並進（z軸）
- 左右：ヨー回転

### 方向ボタン
- 上：離陸
- 下：着陸
- 右：自律飛行モード（`cmd_vel`に従って飛行）
- 左：スマートコントローラモード

#### それ以外のボタン
- H：緊急停止
- 電源ボタン：ROSから切り離してスマートコントローラのみでCoDrone Eduを制御

## License
This software is released under the MIT License, see [LICENSE](https://github.com/jyamauchi780/CoDroneEdu-ROS2/blob/main/LICENSE).

