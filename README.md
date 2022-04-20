# RSIM_KMT

## 起動方法
### PLYデータの準備
~~~
cd model
./stl2ply.py
~~~
ring.plyが作成される

### launch
~~~
roslaunch rsim_ktm start.launch
~~~
または起動アイコンで起動


## vrobo  
vroboはcameraを移動させる仮想ロボットです。
### TF
|TF名|親TF|用途|
|:----|:----|:----|
|base|world|ロボットの基準座標系|
|tool0_controller|base|ロボットの機械エンド(フランジ)|
|uf0|base|ユーザ座標系0はbaseと同じにします|
|uf1|base|ユーザ座標系1はマスターワークの設置座標系に合わせ、マスターの作成の際に用います|
|uf2|base|ユーザ座標系2はVTで用います|

