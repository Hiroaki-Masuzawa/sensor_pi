# sensor_pi

本リポジトリはRaspberry piやUSB-I2c bridgeを用いてGroveセンサ（主にM5 STACKのセンサ)をROSで扱えるようにするためのプログラム群である。

# 使用可能機器
- raspberry pi 4B
- tiny-usb [このソフト](https://github.com/Hiroaki-Masuzawa/rp2040-i2c-interface)をraspberry pi picoに書き込むことで可能。
- FT232H pyftdiを持ちいて実現。
- CP2112 [サンハヤトの変換ボード](https://www.sunhayato.co.jp/material2/ett09/item_1052)で確認。
- MCP2221A [みんなのラボ](http://minnanolab.net/product/pro_USB-I2C_BRIDGEBOARD-V2-GROVE/pro_USB-I2C_BRIDGEBOARD-V2-GROVE.html)で確認。[USB HIDライブラリ](https://github.com/nonNoise/PyMCP2221A)を用いて制御する。


# 使用するI2C通信ライブラリ
| |smbus / smbus2|pyftdi|USB HID|
|:-:|:-:|:-:|:-:|
|raspberry pi|〇|||
|tiny-usb|〇|||
|FT232H||〇||
|CP2112|〇|||
|MCP2221A|||〇|

# 使用可能なセンサ

- [超音波センサ](https://www.switch-science.com/products/7631)
注)CP2112では使用不可

- [カラーセンサ](https://www.marutsu.co.jp/pc/i/26616123/)
- [ToFセンサ](https://www.switch-science.com/products/5219)
- [AD変換](https://www.marutsu.co.jp/pc/i/574269/)+[リフレクタセンサ]()
- [I2Cハブ](https://www.marutsu.co.jp/pc/i/1631561/)


# 使用方法
## configの書き方
`catkin_ws/src/sensor_pi/config`
以下にJSON形式で記載する。
記載は以下の通りに行う.
~~~
{
    "センサ名1":{
        "パラメータ1": "値1",
        "パラメータ2": "値2"
    },
    "センサ名2":{
        "パラメータ1": "値3",
        "パラメータ2": "値4"
    }
}
~~~

ただしI2Cハブについては記載が特殊であり、以下の通りに記載する。
~~~
{
    "I2CHubPublisher": {
        "接続ポート番号" : {
            "name": "センサ名1",
            "パラメータ1": "値1",
            "パラメータ2": "値2"
        },
        "接続ポート番号" : {
            "name": "センサ名2",
            "パラメータ1": "値3",
            "パラメータ2": "値4"
        }
        "パラメータ1": "値0",
    },
    "センサ名3": {
        "パラメータ1": "値5",
        "パラメータ2": "値6"
    }
}
~~~

### config内のセンサ名称
|センサ|config内名称|パラメータ|
|:-:|:-:|:-:|
|超音波センサ|UltraSonicPublisher|address, topic_name|
|カラーセンサ|ColorSensorPublisher|address, topic_name|
|ToFセンサ|TOFPublisher|address, topic_name|
|AD変換+リフレクタセンサ|ReflectorPublisher|address, topic_name, pins|
|I2Cハブ|I2CHubPublisher|address, 接続センサ(上記参照)|

- address : I2Cアドレスを記載(16進数文字列)
- topic_name :　ROS出力時の名称を記載(文字列)
- pins : AD変換で使用するピンを記載(整数値リスト)

## 起動方法
### raspberry piの場合
~~~
cd docker_RaspberryPi
./build.sh
./run.sh configfile
~~~
### FT232Hの場合
~~~
cd docker_ubuntu
./build.sh
./run_ft232h.sh configfile bus_id (おおむね1)
~~~
### Tiny-usb & CP2112の場合
~~~
cd docker_ubuntu
./build.sh
./run_smbus.sh configfile bus_id (場合によって変わるのでdmesg等で確認する)
~~~

### MCP2221Aの場合
~~~
cd docker_ubuntu
./build.sh
./run_mcp2221.sh configfile
~~~