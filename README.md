# pico-pwm

Raspberry Pi Pico で ROS 2 トピック読んで PWM 信号を吐き出す

!!! 開発ちぅ !!!

# 環境構築

ファイル構造の図を載せたい

pico-sdk の環境構築方法を書くか公式サイトに任せるか

micro-ROS Agent をビルドする話を書く

このリポジトリをクローン

サブモジュールする話を書く

このリポジトリのビルド方法を書く

ROS_LOCALHOST_ONLY と ROS_DOMAIN_ID の設定値の話も書かないとだめかも

# Usage
1. micro-ROS Agent の立ち上げ
    ```
    ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyACM0 -v6
    ```

2. デューティ比トピックの確認
    デューティ比の値を確認できる（単位は `%` ）
    ```
    ros2 topic echo /pico/duty
    ```
3. デューティ比の変更
    ```
    ros2 topic pub /pico/pwm std_msgs/msg/Float32 "data: -100"  # 逆転 100% Duty
    ros2 topic pub /pico/pwm std_msgs/msg/Float32 "data: -50"   # 逆転 50% Duty
    ros2 topic pub /pico/pwm std_msgs/msg/Float32 "data: 0"     # 空転
    ros2 topic pub /pico/pwm std_msgs/msg/Float32 "data: 50"    # 正転 50% Duty
    ros2 topic pub /pico/pwm std_msgs/msg/Float32 "data: 100"   # 正転 100% Duty
    ```
    使用する ESC は `50%` で空転

# Node List
```
/pico_node
  Subscribers:
    /pico/pwm: std_msgs/msg/Float32
  Publishers:
    /pico/duty: std_msgs/msg/Float32
  Service Servers:

  Service Clients:

  Action Servers:

  Action Clients:
```

# ライセンス
libmicroros は [micro-ROS/micro_ros_raspberrypi_pico_sdk](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk/tree/humble) のライセンスに依存します。

結構うやむやになってます。（ [pico_micro_ros_example.c](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk/blob/humble/pico_micro_ros_example.c) からパクってるコードが一部あるので Apache 2.0 になる？）

いずれは MIT で公開したいつもりなので MIT の LICENSE ファイルをいっちょ前においてあります（だめ）

# メモ書き
最初に submodule を追加するコマンド
```
git submodule add https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk external/microros_pico
```

submodule 付きで clone
```
git clone --recursive https://github.com/yazawakenichi/pico-pwm
```

submodule 付きで clone し忘れたとき
```
git submodule update --init --recursive
```

なんかファイル色々あるけど実際必要なのは CMakeLists.txt と main.cpp と main.hpp ( あとは libmicroros の中身 ）

