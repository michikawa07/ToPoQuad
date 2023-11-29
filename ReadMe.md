# ToPoQuad

## メモ
dynamixel_handlerは将来リポジトリごと独立させ，依存パッケージとしてのみ記述する（realsenseのSDKみたいに）

## 起動方法
```
$ roscore
```
言わずもがな

```
$ rosrun dynamixel_handler dynamixel_handler_node 
```
dynamixelとのやり取り, 1から{指定したid}までのdynamixelを自動で見つけてくれる．
{指定したid}の指定方法はros paramにしてあるがlaunchファイルを書く元気がない．
角度指令をsubして，dynamixelを位置制御する．

```
$ rosrun topoquad_master leg_node
```
ロボットの関節にどのIDのdynamixelがどんな向きでついているかを知っているノード
関節角の指令をsubして，dynamixelへの角度指令に直してpubしている．

```
$ rosrun topoquad_control sample_control_spider.py
```
ロボットの制御を行うためのノード．関節角の指令をpublishし続ける．

## dynamixel id map
topoquad_master pkg の leg_node が 持っている.
 - 後右 :  4  3  2
 - 前右 : 14 13 12
 - 前左 : 24 23 22
 - 後左 : 34 33 32
　　（根元 <--> 足先）

コード内の表現は以下の様

```cpp
    //                    dyn_ID, ギア比, 初期姿勢, 関節位置 (, 関節姿勢 未実装)
    leg_BR.initialize( Joint{  4, -1.0,   0.0 , Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{  3, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{  2, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) }  );
    leg_FR.initialize( Joint{ 14, -1.0,   0.0 , Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 13, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 12, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) }  ); 
    leg_FL.initialize( Joint{ 24, +1.0,   0.0 , Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 23, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 22, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) }  );
    leg_BL.initialize( Joint{ 34, +1.0,   0.0 , Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 33, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 32, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) }  );     
```
