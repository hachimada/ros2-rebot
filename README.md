# B601-DM: Isaac Sim + ROS 2 Humble

Isaac Sim 6.1 用の最小プロジェクト。ホスト上で USD ロボットを配置し、
Distrobox の ROS 2 Humble から関節位置を指令します。
既存 UDP sender/receiver は終了してから開始してください。

## 実装ドキュメント

[実装解説（構成・ROS 2 通信・関節制御）](docs/architecture.md)

## ホストで起動

```bash
cd /home/nao/ws/ros2_rebot
bash run_sim.sh
```

既定モデルはプロジェクト内の `usd/reBot_B601_DM/reBot_B601_DM.usda`。
モデル一式は [reBot-Isaacsim](https://github.com/hopcan/reBot-Isaacsim)（MIT License、commit `1958511`）の `usd/reBot_B601_DM/` からコピーしたものです。別モデルは `--asset /absolute/path/model.usda`。
Isaac Sim の場所は `ISAACSIM_PATH`（既定 `~/isaacsim`）。

## 別ターミナルで ROS 2 を確認

```bash
humble
cd /home/nao/ws/ros2_rebot
source ros_env.bash
ros2 topic list
ros2 topic echo /joint_states --once
```

期待するトピック: `/joint_states`、`/joint_command`、`/clock`。
両側の ROS_DOMAIN_ID は 0。Distrobox は host network を使用。
Fast DDS は両側とも同梱 XML で UDP 通信に設定し、コンテナ間共有メモリを使いません。
以前の ROS CLI daemon の設定が異なる場合は `ros2 daemon stop` の後に再確認してください。

## 第1関節の位置指令

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{name: ['joint1'], position: [0.35]}"
```

角度はラジアン、グリッパの直動関節はメートル。名前は `/joint_states` を参照。
USD の関節座標に直接指令するため、以前の sender の符号反転は不要です。
Isaac Sim の articulation drive が目標位置を追従します。
この段階では軌道アクション、MoveIt、ros2_control は未実装です。

## 検証

```bash
bash run_sim.sh --headless --frames 180
```

ROS graph はインストール済み NVIDIA Isaac Sim 6.1 の
`standalone_examples/api/isaacsim.ros2.bridge/moveit.py`（Apache-2.0）を基にしています。
