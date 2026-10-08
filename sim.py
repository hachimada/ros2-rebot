# SPDX-License-Identifier: Apache-2.0
# ROS graph adapted from NVIDIA Isaac Sim 6.1 moveit.py sample.
# Original: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES.
"""Spawn B601-DM and expose ROS 2 joint state, command and clock topics."""
import argparse
from pathlib import Path

# 起動オプションは Isaac Sim 起動前に解析する。モデルが存在しない場合も先に検出する。
parser = argparse.ArgumentParser()
# ウィンドウなしの検証用。物理シミュレーションと ROS 通信は通常どおり実行する。
parser.add_argument("--headless", action="store_true")
# 0 は終了操作まで継続。正の値は指定ステップ数で終了する簡易起動検証用。
parser.add_argument("--frames", type=int, default=0, help="Stop after N frames; 0 runs until closed")
# パスは作業ディレクトリではなく本ファイルから解決する。モデル一式は reBot-Isaacsim からプロジェクト内 usd/ にコピーしたもの。
parser.add_argument("--asset", type=Path, default=Path(__file__).resolve().parent / "usd/reBot_B601_DM/reBot_B601_DM.usda")
args = parser.parse_args()
if not args.asset.is_file():
    parser.error(f"USD asset missing: {args.asset}")

# Kit / Omniverse 関連モジュールは、SimulationApp を起動した後で import する。
from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless})

try:
    import numpy as np
    import omni.graph.core as og
    import usdrt.Sdf
    from pxr import UsdLux, UsdPhysics
    from isaacsim.core.api import World
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
    from isaacsim.core.utils.viewports import set_camera_view

    # ROS ノードを提供する拡張をロードし、次の update で初期化を進める。
    enable_extension("isaacsim.ros2.bridge")
    app.update()
    # 長さの基準はメートル。物理と描画をシミュレーション時間 1/60 秒の刻みに揃える。
    # 実時間の実行速度は GPU 負荷等に依存するため、60 FPS を保証する設定ではない。
    world = World(stage_units_in_meters=1.0, physics_dt=1/60, rendering_dt=1/60)
    # ロボットの周囲に衝突可能な標準地面を追加する。
    world.scene.add_default_ground_plane()
    # Prim は USD シーン内のオブジェクト。ここではモデルの配置先を指定する。
    ROBOT_PATH = "/World/reBotArm"
    # USD を参照として配置する。元ファイルをコピー・変更せず、関節や物理設定も読み込む。
    add_reference_to_stage(str(args.asset.resolve()), ROBOT_PATH)
    stage = get_current_stage()
    # 全方向から照らす照明を追加してモデルを見やすくする。
    light = UsdLux.DomeLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(1000)
    # Use the actual articulation root authored in the USD.
    # 配置先 Xform と物理的な関節系の起点は同じとは限らない。
    # モデル配下を探索し、ArticulationRootAPI を持つ Prim を制御対象として特定する。
    roots = [p.GetPath().pathString for p in stage.Traverse()
             if p.GetPath().pathString.startswith(ROBOT_PATH + "/") and p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    # 起点がない／複数あるモデルを誤って制御しないよう、想定外の構造は明示的に止める。
    if len(roots) != 1:
        raise RuntimeError(f"Expected one articulation root, found {roots}")
    # 以降の ROBOT_PATH は配置先ではなく、検出した Articulation Root のパス。
    ROBOT_PATH = roots[0]
    # eye はカメラ位置、target は注視点。どちらもワールド座標のメートル単位。
    set_camera_view(eye=np.array([0.8, 0.8, 0.6]), target=np.array([0, 0, 0.25]))

    # GUI の Action Graph と同じ処理ネットワークを Python で生成する。
    # CREATE_NODES: ノード生成、CONNECT: ポート接続、SET_VALUES: 固定値設定。
    # execIn / execOut は実行のきっかけ、その他のポートは数値や名前などのデータを渡す。
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                # シミュレーション再生中の更新を処理の起点にする。
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                # OS の時計ではなく、シミュレーションの経過時刻を取得する。
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                # 物理エンジンから関節名、位置、速度、力／トルクを読み出す。
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                # Publisher / Subscriber が共通で使う ROS コンテキスト。
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                # sensor_msgs/msg/JointState として現在状態を配信する。
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                # 同じ JointState 型を目標値の入力に利用する。軌道アクションではない。
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                # 関節名と目標値を物理シミュレーションの articulation 制御へ渡す。
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                # /clock を配信。利用側ノードがシミュレーション時刻を使うには use_sim_time を設定する。
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                # 状態を読んだ後に配信するよう、ReadJointState の execOut で Publisher を起動する。
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                # 各配列の対応関係は jointNames の順序で決まる。指令値ではなく計測状態を渡す。
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                # 関節の種類とステージの長さ単位も渡し、ROS 側への単位変換に必要な情報を揃える。
                ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                # 状態メッセージのタイムスタンプに利用する時刻情報。
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                # 受信・時刻配信・制御を更新ごとに実行する。
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                # 全 ROS ノードで同じコンテキストを使い、同じ ROS_DOMAIN_ID で通信する。
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                # /clock の時刻はシミュレーションの進行に追従する。
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                # 名前で対象関節を指定できるので、joint1 だけの部分指令も受け付けられる。
                # 位置は回転関節なら rad、直動関節なら m。従来の UDP sender の符号反転は加えない。
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                (
                    # 位置目標を渡す。追従動作は USD の駆動設定に依存し、ここでは軌道を生成しない。
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    # 速度指令の経路も接続しているが、今回の動作確認では位置指令のみを使用。
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                # 力／トルク指令の経路。適切な駆動モード・ゲイン設定は別途必要。
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Setting the robot target prim to Articulation Controller node
                # 制御側と状態取得側の両方を同じ Articulation Root に向ける。
                ("ArticulationController.inputs:robotPath", ROBOT_PATH),
                # prim 入力は USD パスの配列を要求するため、文字列を Sdf.Path に変換する。
                ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(ROBOT_PATH)]),
                # 名前空間を付けていないため、実際のトピックは /joint_states と /joint_command。
                ("PublishJointState.inputs:topicName", "joint_states"),
                ("SubscribeJointState.inputs:topicName", "joint_command"),
            ],
        },
    )

    app.update()
    # 作成したシーンの物理状態を初期化してから再生を開始する。
    world.reset()
    world.play()
    # この表示はローカル初期化完了を示す。相手への到達は ros2 topic echo 等で別途確認する。
    print("[rebot-ros2] Ready: /joint_states, /joint_command, /clock", flush=True)
    frames = 0
    # ウィンドウを閉じるか、検証用の指定フレーム数に到達するまで実行する。
    while app.is_running() and (not args.frames or frames < args.frames):
        # 物理と描画を1ステップ進める。再生中の Action Graph もこの更新に伴って処理される。
        world.step(render=True)
        frames += 1
# 初期化後の処理で例外が発生しても、Kit アプリケーションの終了処理を呼ぶ。
finally:
    app.close()
