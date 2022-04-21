# RSIM_KMT

## 準備
### PLYデータの準備
~~~
cd model
./stl2ply.py
~~~
ring.plyが作成される

### バーチャルカメラ

~~~
git clone https://github.com/lucasw/rviz_camera_stream.git
~~~

### 照明

~~~
git clone https://github.com/mogumbo/rviz_lighting
~~~

### Rviz設定  
ファイルmain.rvizにカメラ、照明の設定が追加されているので、viewer.launchを起動するだけで バーチャルカメラが有効になります。設定の変更はDisplayパネルにて行います。  
参考にしたURL  
- https://roboticsknowledgebase.com/wiki/tools/stream-rviz/



## 起動
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

