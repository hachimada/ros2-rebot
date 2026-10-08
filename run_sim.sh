#!/usr/bin/env bash
# コマンド失敗・未定義変数・パイプ途中の失敗を見逃さず終了する。
set -euo pipefail
# 呼び出し元のカレントディレクトリに依存せず、スクリプト自身の場所を求める。
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# ISAACSIM_PATH があれば優先し、未設定または空なら ~/isaacsim を利用する。
isaac_dir="${ISAACSIM_PATH:-$HOME/isaacsim}"
# このランチャーはホスト用。Distrobox 内では ros_env.bash を source する。
if [[ -n "${CONTAINER_ID:-}" ]]; then
    echo 'Run this launcher on the host, outside Distrobox.' >&2
    exit 1
fi
# ホストに ROS を追加インストールせず、Isaac Sim 同梱の Humble ライブラリを選ぶ。
export ROS_DISTRO=humble
# ROS の通信グループ。ホスト側で上書きした場合は Distrobox 側も同じ値に揃える。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# localhost 限定を解除する。通信をこのPCのみに制限する設定ではない点に注意。
export ROS_LOCALHOST_ONLY=0
# ROS ミドルウェアは両側とも Fast DDS を使用する。
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 同梱 XML で UDP 通信を選び、ホスト／コンテナ間の共有メモリの違いを避ける。
export FASTRTPS_DEFAULT_PROFILES_FILE="$project_dir/fastdds.xml"
# Isaac Sim 6.1 の同梱ライブラリを優先する。既存のライブラリ探索パスも後ろに保持する。
export LD_LIBRARY_PATH="$isaac_dir/exts/isaacsim.ros2.core/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# シェルを Isaac Sim のランチャーに置き換える。"$@" で --headless 等の引数をそのまま渡す。
exec "$isaac_dir/python.sh" "$project_dir/sim.py" "$@"
