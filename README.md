# waypoint_manager

<img width="736" alt="Screen Shot 2022-01-19 at 18 19 39" src="https://user-images.githubusercontent.com/18626482/150101087-60d64f9e-ca0b-46d5-82cf-49a279f38a61.png">

## Releases
https://github.com/masakifujiwara1/waypoint_manager/releases

## Usage

```shell
roslaunch waypoint_server waypoint_server.launch
```
or 
```shell
roslaunch waypoint_server waypoint_server.launch config_file:=<your confg>
```
### when using options
```
roslaunch waypoint_server waypoint_server_options.launch config_file:=<your confg> use_check_robot_moving:=<true or false> use_waypoint_reconfigure:=<true or false>
```

## Options
### check_robot_moving
https://github.com/masakifujiwara1/waypoint_manager/releases/tag/v2.0.1

### waypoint_reconfigure
https://github.com/masakifujiwara1/waypoint_manager/releases/tag/v2.1.1-beta

## TODO

+ Add an edit panel for waypoint properties
+ Support actionlib
+ Support ros2
+ Refactor code

## 列待ち制御（box2）

制御は既存の `waypoint_to_posestamped_node` に組み込まれています。
新しいパッケージ・制御ノード・設定ファイルは追加せず、
`waypoint_reconfigure` が同じYAMLの設定を配信します。
`waypoint_server_options.launch` の `reconfig_file` に指定したファイルを使います。
`kobayashi` の既定は `waypoint_reconfigure/config/reconfigure_list.yaml` です。

使用する地図・初期位置・ウェイポイント・ルートの既定は、列待ち追加前の
`tsukuba0926` 用設定です。3Fの地図とルートは試験用データとして残しています。
コストマップ用の地図は `map_file` に対応する `_for_costmap.yaml` を読み込みます。
既定で必要な `tsukuba0926_for_costmap.yaml` はこのリポジトリに含まれていないため、
使用する進入禁止領域の地図を配置してから起動してください。

`waypoint_reconfigure_config.areas` に次の区間を追加してください。
下の例のID・座標は説明用です。既定ファイルには3F試験用の区間
`crossing_queue_3F` を設定しています。この区間ID・停止線は `tsukuba0926` のルートでは
使えないため、走行前に使用するルートに合わせて設定してください。

```yaml
waypoint_reconfigure_config:
  # 既存のdefault_*設定は保持する
  queue_wait:
    max_speed: 0.3
    detection_distance: 1.0  # 赤枠と停止判定。車体前端からの距離[m]
    detection_half_width: 0.8
    resume_distance: 1.3
    clear_duration: 2.0
  areas:
    - name: crossing_queue
      start_id: REPLACE_WITH_APPROACH_ID
      end_id: REPLACE_WITH_STOP_ID
      queue_wait: true
      stop_line:
        point: [10.0, 0.0]  # 実測した停止線のmap座標へ置き換える
        yaw: 0.0           # 停止線の法線。横断方向を向く角度[rad]
      properties: []
```

列待ちの範囲は開始IDから終了IDまでの両端を含みます。終了IDは停止線手前の
停止ウェイポイントで、ウェイポイント側に `stop: true` が必要です。
開始IDは迂回が始まるより十分手前に置いてください。変換済みのウェイポイント列を
固定経路として使い、区間途中から起動した場合も列待ちに入ります。
古いID・区間重複・逆順・停止線を越えた停止目標は拒否します。

ビルド後、各端末でこのワークスペースの `devel/setup.bash` を読み込みます。

```bash
catkin build waypoint_server waypoint_reconfigure check_robot_moving
source devel/setup.bash
# terminal 1
roslaunch orne_box_bringup orne_box.launch
# terminal 2
roslaunch orne_box_navigation_executor play_waypoints_nav_box.launch
# terminal 3
roslaunch waypoint_server waypoint_server_options.launch
```

別の区間設定を使う場合はterminal 3に `reconfig_file:=/absolute/path/sections.yaml` を追加します。
`use_queue_wait` の既定は両launchとも `true` です。通常区間と列待ち区間はYAMLで切り替えます。
設定が届かない場合、区間IDがルートと一致しない場合、`queue_wait: true` の区間がない場合は
自律速度を出しません。両launchに `use_queue_wait:=false` を付けると従来の構成を使います。
列待ち有効時には
`waypoint_reconfigure` も起動し、ゴールの重複を避けるため `fulanghua_waypoints_nav` を起動しません。

通常区間はmove_baseの速度指令を中継します。列待ち区間ではmove_baseのゴールをキャンセルし、
専用の前進制御に切り替えます。迂回・旋回リカバリー・後退脱出を使いません。
前方の経路周辺にある障害物を検出し、車体前端から1mで待機、
1.3m以上の空きが2秒続くと最大0.3m/sで前詰めします。ロボットの識別・追跡は行いません。
欠測や古いscan・TF・local costmapでは停止し、待機には時間制限を設けません。

車体全体と予測軌道を `/map_for_costmap` とlocal costmapの両方で確認します。
未知領域・地図外・経路逸脱も進行不可です。制約地図は通常のmapと同じ物理座標系を
前提とし、有効時のlaunchでヘッダーをmapに揃えます。車体の多角形と地図セル全体の
重なりを確認し、車体内部の障害物やセルの一部分だけの接触も進行不可とします。
local costmapは実障害物セル（配信値100）と未知セルを車体全体に照らして確認します。
配信値99は車体の内接半径ぶん膨張したコストなので、車体全体へ重ねて二重に膨張させません。
制約地図は従来どおり配信値99以上と未知セルを進行不可として扱います。

`Switch Cancel` で停止・再開できます。停止保持中のNext/Prev/Resetと自動ウェイポイント更新、
区間内の停止監視によるコストマップ消去・Prev→Nextは抑止します。
停止線では自動再開せず、前方が継続して空いた後、オペレータが次を実行して
RVizのNext Waypointを押します。

```bash
rosservice call /queue_wait/release
```

`/queue_wait/state`、`/queue_wait/active`、`/queue_wait/hold` で状態を確認できます。
設定・ルート変更中は停止を保持するため、設定を見直してノードを再起動してください。

RVizの `nav.rviz` には `Queue Stop Range` 表示を追加しています。
列待ち区間では `/queue_wait/detection_range` に赤い停止判定範囲の枠を配信します。
枠は経路上の車体基準位置から、車体の最前端に `detection_distance` を加えた位置までです。
枠内で前方2D LiDARの点を検知すると待機します。3F設定は車体前端から1m、左右0.4mです。
`detection_distance` は停止距離で、`stop_distance` は廃止しました。
減速と再開確認のため、赤枠より先の範囲も内部で調べます。
この先読み範囲は `resume_distance` と、最高速度・減速度・予測時間・scan更新の余裕から
自動計算します。現在の3F設定では、車体前端から約1.80m先までを確認します。
先読みの外端は赤枠に含めず、赤枠は停止判定の範囲だけを表示します。
直線では四角、曲がり角では経路に沿った枠になります。経路末端の先も判定と同じく延長します。
曲がり角の境界は最大5cm刻みで近似した表示です。
枠は障害物の有無に関係なく表示し、列待ち区間外・自己位置欠測・設定異常では消えます。
これは `/scan` の停止条件を表示し、LiDARが実際に観測できる範囲を保証する表示ではありません。
`QUEUE_WAIT`に入った後は、赤枠の外へ出ても、車体前端から
`resume_distance`（3Fは1.3m）以上の空きが`clear_duration`（2秒）続くまで待機します。欠測・衝突予測・停止線などでも停止します。
別のRViz設定を使う場合は `Add` → `Marker` を追加し、
`Marker Topic` を `/queue_wait/detection_range`、`Fixed Frame` を `map` に設定してください。
赤枠の大きさは同じYAMLの `queue_wait.detection_distance` と
`queue_wait.detection_half_width` を変更して調整します。
`resume_distance` は `detection_distance` より大きい値が必要です。
旧設定を使う場合は `stop_distance` の値を `detection_distance` に移し、
`stop_distance` の項目を削除してください。

列待ち有効時にWP座標・停止属性・ルートが変更されると停止を保持します。
これは実機でも同じです。固定経路と停止線の設定が食い違うため、確認して再起動してください。

3Fの保存済みルートはRVizの番号で `0 → 1 → 2 → 3 → 4` です。
番号0→2は通常走行、番号2へ到達して次の目標が番号3になると、番号2→4を列待ちで走行します。
そのため区間設定の `start_id` には番号3のIDを指定しています。
番号2の `goal_radius` は0.25mで、手前から列待ちへ切り替わることを防ぎます。
停止線は番号4の停止WPから進行方向へ0.8m先に置いた3F用の境界です。
実コースではWPと停止線を実測した位置へ設定してください。
