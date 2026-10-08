# reBot B601-DM の関節と駆動

このページでは、プロジェクト内の USD（`usd/reBot_B601_DM/`）で、reBot B601-DM の関節と駆動がどう定義されているかをまとめます。あわせて、USD で駆動を定義する一般的な方法も紹介します。

関節の型や API の一般的な説明は [usd_joints.md](usd_joints.md)、スキーマの概念は [usd_schemas.md](usd_schemas.md) を参照してください。

内容は Isaac Sim 6.1 と、USD の Physics バリアント `physx`（`reBot_B601_DM.usda:50` で選択されているもの）を前提にしています。

---

## 1. 関節の定義

定義元は `usd/reBot_B601_DM/payloads/Physics/physics.usda` です。

reBot B601-DM は、**6 軸の回転関節アームの先に、直動 2 本の平行グリッパーが付いた構成** です。関節は全部で 10 個あり、そのうち指令できるのは 8 個です。

### 1.1 構造（運動学ツリー）

```
ワールド
 └ root_joint ── 固定 ── base_link            ← Articulation Root
     └ joint1 ── 回転 ── link1
         └ joint2 ── 回転 ── link2
             └ joint3 ── 回転 ── link3
                 └ joint4 ── 回転 ── link4
                     └ joint5 ── 回転 ── link5
                         └ joint6 ── 回転 ── link6
                             └ gripper_joint ── 固定 ── gripper_link
                                 ├ gripper_joint1 ── 直動 ── gripper_left
                                 └ gripper_joint2 ── 直動 ── gripper_right
```

閉じたループはなく、一本の木構造です。そのため全体を 1 つの Articulation として解けます。

### 1.2 関節一覧

「回転軸（初期姿勢）」は、各関節の軸を全関節 0 のときのロボット基準座標で表したものです。`localRot0` を根元から順にかけ合わせて計算しました。

| 関節 | USD の型 | 回転軸（初期姿勢） | 役割 | 可動範囲 |
|---|---|---|---|---|
| joint1 | `PhysicsRevoluteJoint` | +Z（鉛直） | 根元の旋回（yaw） | ±2.80 rad（±160.4°） |
| joint2 | `PhysicsRevoluteJoint` | −Y（水平） | 肩の上下（pitch） | −3.14 〜 0 rad |
| joint3 | `PhysicsRevoluteJoint` | −Y | 肘（pitch） | −3.14 〜 0 rad |
| joint4 | `PhysicsRevoluteJoint` | −Y | 手首の曲げ（pitch） | −1.87 〜 1.57 rad |
| joint5 | `PhysicsRevoluteJoint` | +Z | 手首の旋回（yaw） | ±1.57 rad |
| joint6 | `PhysicsRevoluteJoint` | +X | 手先のロール | ±3.14 rad |
| gripper_joint1 | `PhysicsPrismaticJoint` | +Y 方向にスライド | 左の指 | 0 〜 0.0715 m |
| gripper_joint2 | `PhysicsPrismaticJoint` | −Y 方向にスライド | 右の指 | 0 〜 0.0715 m |
| root_joint | `PhysicsFixedJoint` | — | base_link をワールドに固定 | — |
| gripper_joint | `PhysicsFixedJoint` | — | グリッパー本体を link6 に固定 | — |

### 1.3 定義のされ方の特徴

- **USD 上の軸指定は、ほとんどが `axis = "Z"`**：回転関節 6 つはすべて Z 軸で定義されています。実際の向きの違いは、`localRot0`（関節座標系の回転）で作られています。たとえば joint2 は X 軸まわりに 90° 回した座標系の Z 軸なので、水平軸になります。
- **アームの配置**：yaw → pitch → pitch → pitch → yaw → roll の並びです。肩・肘・手首の曲げが平行な 3 軸で並ぶ、卓上 6 軸アームによくある構成です。
- **グリッパー**：2 本の指は、互いに逆向き（±Y）にスライドする独立した直動関節です。連動（mimic）の設定はないので、開閉するには 2 本とも指令する必要があります。
- **可動範囲の値**：USD には度で書かれています（例：`lowerLimit = -160.42818`）。rad に直すと 2.80、3.14、1.87、1.57 と切りのいい値になるので、元の URDF に rad で書かれていたものが変換で度になったと考えられます。
- **固定関節 2 つ**：URDF の `fixed` 関節がそのまま変換されたものです。指令の対象にはなりません。

注：「回転軸（初期姿勢）」と「役割」の列は、子リンクの座標系が関節座標系と一致している（`localPos1 = 0`、`localRot1 = 単位回転`。全関節でこうなっています）という前提で計算しました。指の開閉の向き（0 が閉じ側かどうか）は、USD の数値だけでは確定できません。

---

## 2. 駆動の定義

### 2.1 定義の置き場所

駆動は、各関節に付けた `PhysicsDriveAPI` で定義されています。値は 2 つのファイルに分かれて書かれています。

| ファイル | 内容 |
|---|---|
| `payloads/Physics/physics.usda` | `PhysicsDriveAPI` と `PhysicsJointStateAPI` を付け、`maxForce` だけを設定（stiffness と damping は既定値の 0） |
| `payloads/Physics/physx.usda` | physics.usda を下敷き（subLayer）にして、`stiffness`、`damping`、`targetPosition`、`type`、`PhysxJointAPI` の `maxJointVelocity` を上乗せ |

**physx バリアントでなければ、位置指令に追従しません。** physics バリアントではゲインが 0 になります。mujoco バリアントでは Drive が削除され、代わりにトルク指令型の `MjcActuator` が使われます。

### 2.2 設定値（physx バリアントで合成した結果）

| 関節 | Drive の自由度 | type | stiffness | damping | maxForce | maxJointVelocity |
|---|---|---|---|---|---|---|
| joint1 | `angular` | force | 500 | 60 | 27 N·m | 2864.8 °/s（= 50 rad/s） |
| joint2 | `angular` | force | 1500 | 96 | 27 N·m | 2864.8 °/s |
| joint3 | `angular` | force | 1000 | 76 | 27 N·m | 2864.8 °/s |
| joint4 | `angular` | force | 150 | 18 | 7 N·m | 11459.2 °/s（= 200 rad/s） |
| joint5 | `angular` | force | 80 | 10 | 7 N·m | 11459.2 °/s |
| joint6 | `angular` | force | 50 | 7 | 7 N·m | 11459.2 °/s |
| gripper_joint1/2 | `linear` | force | 5000 | 41.28 | 100 N | 859.4（§2.6 を参照） |

全関節で `targetPosition = 0` です。そのため、指令が来るまでは全関節 0 の姿勢を保とうとします。

### 2.3 駆動の仕組み

各関節は、トルク上限付きの PD 制御で動きます [1]。

```
トルク = clamp( stiffness × (目標位置 − 位置) + damping × (目標速度 − 速度),  ±maxForce )
```

`type = "force"` は、この式の結果を「力・トルク」として加えるという意味です。もう一方の `acceleration` を選ぶと、リンクの質量に関係なく加速度として扱われます。

`/joint_command` の値は、`sim.py` の `ArticulationController` を通して、毎ステップこの式の目標値に書き込まれます。

| `JointState` の欄 | 反映先 |
|---|---|
| `position` | 目標位置（rad から度への変換は Isaac Sim 側で行われる） |
| `velocity` | 目標速度 |
| `effort` | この式とは別に加わる追加の力・トルク |

### 2.4 単位と、実際の動き方

回転関節の stiffness と damping は、**1 度あたり** の値です [1]。rad あたりに直すと、57.3 倍の値になります。

| 関節 | stiffness（rad あたりに換算） | トルク上限に達する位置誤差（maxForce ÷ stiffness） | 追従の時定数（damping ÷ stiffness） |
|---|---|---|---|
| joint1 | 約 28,600 N·m/rad | 0.054° | 0.12 s |
| joint2 | 約 85,900 N·m/rad | 0.018° | 0.064 s |
| joint3 | 約 57,300 N·m/rad | 0.027° | 0.076 s |
| joint4 | 約 8,600 N·m/rad | 0.047° | 0.12 s |
| joint5 | 約 4,600 N·m/rad | 0.088° | 0.13 s |
| joint6 | 約 2,900 N·m/rad | 0.14° | 0.14 s |
| gripper | 5,000 N/m | 20 mm | 0.008 s |

この表から、アームの動き方は次のようになります。
- アームの関節は、目標から 0.1° 前後ずれるだけでトルクが上限に達するほど硬いゲインです。
- そのため、大きく動かすときは **ほぼ上限トルク（27 N·m または 7 N·m）で加速** します。
- 目標に近づくと、時定数 0.1 秒前後で収束します。
- 実質的に、動きの速さを決めているのはゲインではなく **maxForce（トルク上限）** です。

### 2.5 ゲインの出どころ

元リポジトリ（[reBot-Isaacsim](https://github.com/hopcan/reBot-Isaacsim)）の履歴を確認しました。

- コミット `458752b`（2026-08-13、「change physx.usda to set joint can be control」）で、physx.usda に Drive とゲインが追加されています。それ以前は、`maxJointVelocity` しか書かれていませんでした。
- 値は、同じリポジトリの別モデル `usd/RS-rebot-dev-arm/scripts/prep_asset.py` の `GAINS` と完全に一致します。そのファイルのコメントには、次のように書かれています。
  - アームの 6 関節：「Validated 2026-07-07（8/8 pass, errors ~1e-5 rad, Newton/PhysX parity）」
  - 指のゲイン 5000 / 41.28：実機のモーター（RobStride、14 N·m）とピニオン径から求めた剛性を、ソルバーが安定する範囲に抑えたもの。damping は指の有効質量 0.0852 kg に対して減衰比 ζ = 1 になるように決めた。

つまり、**このゲインは開発用アーム（RS-rebot-dev-arm）で検証された値を、B601-DM にそのまま流用したもの** と考えられます。B601-DM 自体で調整した記録は見つかりませんでした。

### 2.6 注意点

1. **グリッパーの `maxJointVelocity = 859.4` は単位の取り違えとみられます**
   - URDF では速度上限が 15 です。15 × 57.3 = 859.4 なので、度への換算がかかっています。
   - しかし直動関節のこの値の単位は「距離/秒」なので [3]、859 m/s を意味してしまいます。
   - 上限としては事実上無制限なので、動作に問題は出ません。ただ、値としては正しくありません。
2. **指 2 本の連動がない**
   - 流用元の dev-arm では、2 本の指を mimic で連動させたうえで、それぞれにドライブも付けていました。PhysX は直動関節の mimic を強制しないためです（同ファイルのコメントより）。
   - B601-DM には mimic がなく、2 本の指が独立したドライブで動きます。そのため、両方に同じ目標値を送る必要があります。
3. **関節摩擦と armature は未設定です**（既定値 0）。実機の減速機の摩擦や慣性は、モデルに入っていません。
4. **純粋なトルク制御にしたい場合**：`effort` だけを送っても、PD の項が同時に効きます。stiffness と damping を 0 にする必要があります。

---

## 3. 駆動を定義する一般的な方法

USD で関節を駆動する書き方は、`PhysicsDriveAPI` 以外にもあります。どれが効くかは、**使う物理エンジン（ソルバー）** によって決まります。Isaac Sim のゲイン調整ツールの説明にも、「ゲインは PhysX の `PhysicsDrive`、Newton の `NewtonActuator`、MuJoCo の `mjc:*` の 3 か所のどれかに書かれる」とあります [7]。

### 3.1 全体像

| 系統 | 書く場所 | 主な API | 読むエンジン |
|---|---|---|---|
| ① UsdPhysics（標準） | USD | `PhysicsDriveAPI` | PhysX、Newton など共通 |
| ② PhysX の拡張 | USD | `PhysxDrivePerformanceEnvelopeAPI`、腱、ギア | PhysX |
| ③ Newton のアクチュエータ | USD | `NewtonActuator` と制御則・制限の API | Newton |
| ④ MuJoCo のアクチュエータ | USD | `MjcActuator`、`MjcTendon` | MuJoCo（Newton の MuJoCo ソルバーを含む） |
| ⑤ USD の外で計算 | Python など | Isaac Lab のアクチュエータモデル、自作の制御 | トルクだけをエンジンに渡す |

### 3.2 ① UsdPhysics：`PhysicsDriveAPI`

reBot が使っている方式です（§2）。トルク上限付きの PD を、自由度ごとに付けます [1]。標準仕様なので、エンジンに依存しない共通の書き方です。

### 3.3 ② PhysX の拡張（PhysxSchema）

`PhysicsDriveAPI` を置き換えるのではなく、**補う** ものです [3][4]。

- **`PhysxDrivePerformanceEnvelopeAPI`**：モーターの速度－トルク特性を足します。「回転が速くなるほど出せるトルクが減る」といった挙動を表せます。同じ軸に `PhysicsDriveAPI` が付いていることが前提です。
- **`PhysxJointAxisAPI`**：静止摩擦、動摩擦、粘性摩擦、armature を軸ごとに設定します。駆動そのものではなく、駆動される側の損失や慣性を表すものです。
- **腱**（`PhysxTendonAxisAPI` など）：複数の関節を、ばね・ダンパでつながった「腱」で連動させます。ワイヤー駆動の指などに使います。スキーマ上はまだ Draft API の扱いです。
- **ギア関節・ラック＆ピニオン**（`PhysxPhysicsGearJoint`、`PhysxPhysicsRackAndPinionJoint`）：2 つの関節の動きを機械的に結びつけます。

### 3.4 ③ Newton のアクチュエータ（実験的）

`PhysicsDriveAPI` とは考え方が違い、**アクチュエータを関節とは別の Prim として作り、部品を組み合わせて定義** します [5][6]。

```usda
def NewtonActuator "joint1_actuator" (
    prepend apiSchemas = ["NewtonPDControlAPI", "NewtonDCMotorClampingAPI", "NewtonActuatorDelayAPI"]
)
{
    rel newton:targets = </.../joint1>   # 駆動する関節（回転または直動）
    float newton:kp = 500                 # rad あたりのゲイン（Drive API は度あたりなので注意）
    float newton:kd = 10
    float newton:saturationEffort = 27    # 停止時の最大トルク
    float newton:velocityLimit = 50       # 無負荷時の最高速度
    int newton:delaySteps = 2             # 指令が届くまでの遅れ
}
```

（数値は書き方を示すための例で、reBot 用に調整した値ではありません。）

組み合わせられる部品は次のとおりです。

| 種類 | API | 内容 |
|---|---|---|
| 制御則（どれか 1 つ） | `NewtonPDControlAPI` | PD 制御 |
| | `NewtonPIDControlAPI` | 積分項付きの PID 制御 |
| | `NewtonNeuralControlAPI` | 学習済みのニューラルネット（TorchScript）で出力を決める |
| 出力の制限（いくつでも） | `NewtonMaxEffortClampingAPI` | 単純な上限 |
| | `NewtonDCMotorClampingAPI` | DC モーターの 4 象限の速度－トルク曲線 |
| | `NewtonPositionBasedClampingAPI` | 関節角に応じた上限の表 |
| 遅れ | `NewtonActuatorDelayAPI` | 通信や処理の遅れをステップ数で指定 |

注意点：
- スキーマ自体に **「EXPERIMENTAL: … may change without notice」** と書かれていて、今後名前や仕様が変わる可能性があります。
- 単位は rad で、`PhysicsDriveAPI`（度）とは違います。
- 受動的な特性（摩擦、減衰、armature、速度上限）は、別の `NewtonJointAPI` で関節側に付けます。

### 3.5 ④ MuJoCo のアクチュエータ（MjcPhysics）

reBot の `mujoco.usda` が使っている方式です [8]。

- **`MjcActuator`**：`gainPrm`、`biasPrm`、`gainType`、`biasType` の組み合わせで、トルクモーター、位置サーボ、速度サーボなどを表します。既定値（gainPrm = 1、biasType = none）のままだと、指令値がそのままトルクになるモーター型になります。reBot の mujoco.usda はこの既定値のままで、`forceRange` だけを設定しています。
- 駆動対象（`mjc:target`）には関節だけでなく、腱（`MjcTendon`）や site も指定できます。`mjc:gear` で減速比を、`crankLength` などでクランク機構も表せます。
- **`MjcEqualityJointAPI`** などで、関節どうしの連動（等式拘束）も書けます。

### 3.6 ⑤ USD の外で駆動を計算する

USD 側のドライブは stiffness = damping = 0 にして（または付けずに）、**トルクを外で計算して毎ステップ渡す** 方法です。

- **Isaac Lab のアクチュエータモデル** [9]
  - `ImplicitActuator`：USD の Drive をそのままソルバーに任せます。reBot の今の構成と同じ方式です。
  - 外で計算する方式（explicit）：`IdealPDActuator`、`DCMotor`、`DelayedPDActuator`、`RemotizedPDActuator`、`ActuatorNetMLP` / `ActuatorNetLSTM`（実機データで学習したネットワーク）
  - 強化学習で、実機との差（sim-to-real gap）を縮めたいときによく使われます。
- **自作の制御**：ROS 2 側で計算したトルクを、`/joint_command` の `effort` で送る方法です。`sim.py` でも配線はできています（`effortCommand` の経路）。ただし §2.6 の 4 のとおり、PD ゲインを 0 にする必要があります。

### 3.7 選び方の目安

| やりたいこと | 向いている方式 |
|---|---|
| Isaac Sim（PhysX）で位置制御すればよい（reBot の今の構成） | ① `PhysicsDriveAPI` |
| PhysX でモーターの速度－トルク特性も再現したい | ① と ② の `PhysxDrivePerformanceEnvelopeAPI` |
| 指令の遅れや DC モーター特性を USD に書いておき、Newton で動かしたい | ③ `NewtonActuator` |
| MuJoCo と同じモデルを共有したい | ④ `MjcActuator`（physics.usda を共通の土台にし、エンジン別のファイルを重ねる。reBot の USD と同じ構成） |
| 強化学習や実機同定に合わせ込みたい | ⑤ Isaac Lab の explicit actuator |

---

## 参照

1. OpenUSD, [UsdPhysicsDriveAPI](https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html)：ドライブの式、`type`、インスタンス名、角度ドライブの単位（度あたり）
2. OpenUSD, [Overview of UsdPhysics](https://openusd.org/release/api/usd_physics_page_front.html)：関節の型、関節座標系、Articulation
3. Isaac Sim 6.1 同梱の PhysxSchema 定義（`~/isaacsim/extscache/omni.usd.schema.physx-*/plugins/PhysxSchema/resources/schema.usda`）：`PhysxJointAPI`（`maxJointVelocity` の単位）、腱、ギア関節
4. NVIDIA Omniverse Physics, [PhysX Joint Schema](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/joints/physx_joint_schema.html)：`PhysxJointAxisAPI`、`PhysxDrivePerformanceEnvelopeAPI`
5. Isaac Sim 6.1 同梱の Newton スキーマ定義（`~/isaacsim/exts/omni.usd.schema.newton/usd/schema/newton/newton_usd_schemas/generatedSchema.usda`）：`NewtonActuator` 系、`NewtonJointAPI`
6. NVIDIA Omniverse Physics, [Newton Schema](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/schemas/newtonSchema.html)
7. Isaac Sim 6.1, [isaacsim.robot_setup.gain_tuner](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.robot_setup.gain_tuner/docs/index.html)：エンジンごとのゲインの置き場所
8. Isaac Sim 6.1 同梱の MjcPhysics スキーマ定義（`~/isaacsim/exts/omni.usd.schema.mujoco/plugins/mjcPhysics/resources/generatedSchema.usda`）：`MjcActuator` の既定値、`MjcTendon`、`MjcEqualityJointAPI`
9. Isaac Lab, [isaaclab.actuators](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.actuators.html)：implicit / explicit アクチュエータ
10. [hopcan/reBot-Isaacsim](https://github.com/hopcan/reBot-Isaacsim)：コミット `458752b`、`usd/RS-rebot-dev-arm/scripts/prep_asset.py` の `GAINS` とコメント
