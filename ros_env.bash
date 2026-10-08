# Source this file inside the Humble Distrobox.
# bash ros_env.bash ではなく source ros_env.bash で実行し、現在の端末へ設定を反映する。
# コンテナ内にインストール済みの ROS 2 Humble のコマンド・ライブラリを有効化する。
source /opt/ros/humble/setup.bash
# ホスト側と同じ通信グループにする。ホストで変更した場合はここも揃える。
export ROS_DOMAIN_ID=0
# ホスト側と同じ探索設定。ネットワークは Distrobox の host network を利用する。
export ROS_LOCALHOST_ONLY=0
# Isaac Sim 側と同じ Fast DDS 実装を使う。
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# source されたファイルの位置から XML の絶対パスを求めるため、実行場所は問わない。
# XML はホスト側と共通。既存の ros2 daemon が古い設定なら ros2 daemon stop 後に再確認する。
export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/fastdds.xml"
