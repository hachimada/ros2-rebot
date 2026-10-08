USD では関節を **「2 つの剛体をどう拘束するか」** として書きます。仕組みは次の 3 層に分かれています。

1. **関節の型**（Prim の型）：どの自由度を残すかを決める。
2. **付加する API スキーマ**：可動範囲、駆動（モーター）、摩擦、連動などを後から足す。
3. **Articulation**：関節でつながった剛体群をまとめて 1 台のロボットとして扱う。

標準仕様は OpenUSD に含まれる **UsdPhysics** です [1]。ギア、腱、関節状態など PhysX 固有の機能は、NVIDIA の拡張である **PhysxSchema** で補います [6]。mimic（関節の連動）は、最新の Isaac Sim では **Newton スキーマ** の側にあります [8]。

## 1. 関節の基本形：このプロジェクトの joint1

`usd/reBot_B601_DM/payloads/Physics/physics.usda:131` の定義です。

```usda
def PhysicsRevoluteJoint "joint1" (
    prepend apiSchemas = ["PhysicsDriveAPI:angular", "PhysicsJointStateAPI:angular"]
)
{
    rel physics:body0 = </reBot_B601_DM/Geometry/base_link>        # 親側の剛体
    rel physics:body1 = </reBot_B601_DM/Geometry/base_link/link1>  # 子側の剛体
    point3f physics:localPos0 = (-0.00008416, 0, 0.08465)  # body0 から見た関節位置
    quatf   physics:localRot0 = (1, 0, 0, 0)               # body0 から見た関節座標系の向き
    point3f physics:localPos1 = (0, 0, 0)                  # body1 から見た関節位置
    quatf   physics:localRot1 = (1, 0, 0, 0)
    uniform token physics:axis = "Z"         # 関節座標系のどの軸で回るか
    float physics:lowerLimit = -160.42818    # 角度の単位は「度」
    float physics:upperLimit = 160.42818
}
```

どの関節も、次の要素で表します。

- **body0 / body1**：拘束する 2 つの剛体 [1][2]。
- **localPos / localRot**：各剛体から見た関節座標系の位置と向き。
- **axis**：その関節座標系のどの軸に沿って動くか。軸は X/Y/Z のどれかしか選べません（`allowedTokens = ["X", "Y", "Z"]`）[3]。斜めの軸にしたいときは localRot で座標系ごと回します（gripper_joint1 の `localRot0` が 90° 回転になっているのはこのためです）。

## 2. 関節の型

UsdPhysics の関節型は次のとおりです [1][2]。

| USD の型 | 残る自由度 | 現実の例 |
|---|---|---|
| `PhysicsFixedJoint` | 0 | 固定ボルト、ツールフランジ（このモデルの `gripper_joint`） |
| `PhysicsRevoluteJoint` | 回転 1 | サーボや減速機付きモーターの軸（joint1〜6） |
| `PhysicsPrismaticJoint` | 直動 1 | リニアアクチュエータ、平行グリッパー（gripper_joint1/2） |
| `PhysicsSphericalJoint` | 回転 3（`axis` まわりの円錐状の範囲制限を付けられる） | ボールジョイント、肩のモデル |
| `PhysicsDistanceJoint` | 2 点間の距離を最小値・最大値で拘束 | ワイヤーやロッドの簡易モデル |
| `PhysicsJoint`（汎用、いわゆる D6） | 6 自由度を軸ごとに選ぶ | ユニバーサルジョイント、円筒関節、平面関節など |

### 専用の型がない関節は汎用 `PhysicsJoint` で作る
汎用 `PhysicsJoint` は、そのままでは 6 自由度がすべて自由です。`PhysicsLimitAPI:<自由度>` で軸ごとにロックするか、範囲を付けて組み立てます。low が high より大きいと、その自由度はロックされます [1][4]。

```usda
def PhysicsJoint "universal_joint" (
    prepend apiSchemas = ["PhysicsLimitAPI:transX", "PhysicsLimitAPI:transY", "PhysicsLimitAPI:transZ",
                          "PhysicsLimitAPI:rotX", "PhysicsLimitAPI:rotY"]
)
{
    # low > high にするとその自由度はロックされる
    float limit:transX:physics:low = 1
    float limit:transX:physics:high = -1
    # transY, transZ も同様にロック
    float limit:rotX:physics:low = 1
    float limit:rotX:physics:high = -1     # rotX をロック → 残りは rotY・rotZ の 2 軸回転 = ユニバーサルジョイント
    float limit:rotY:physics:low = -45
    float limit:rotY:physics:high = 45     # rotY は ±45° まで
    # rotZ は Limit API を付けないので自由
}
```

自由度を表す名前は `transX/Y/Z`、`rotX/Y/Z`、`distance` です [4]。Limit API を付けても値を書かなければ既定値の low = -inf、high = inf のままなので、自由度は制限されません [4]。

## 3. 動かし方と特性は API で後から足す

`<自由度>` の部分は API スキーマのインスタンス名です。1 つの関節に自由度ごとに複数付けられます（multiple-apply）。

| API | 役割 | このモデルでの使い方 |
|---|---|---|
| `PhysicsDriveAPI:<自由度>`（UsdPhysics） | モーター（PD ドライブ）。`stiffness·(targetPosition − 位置) + damping·(targetVelocity − 速度)` を `maxForce` で制限して加える。`type` は force / acceleration [5] | 自由度名は回転関節なら `angular`、直動関節なら `linear`、D6 なら `rotX` など [5] |
| `PhysicsJointStateAPI:<自由度>`（PhysxSchema） | Articulation の関節軸の位置と速度を読み書きする。名前は `Physics` で始まるが、定義は UsdPhysics ではなく PhysxSchema [6] | 初期姿勢の指定に使う |
| `PhysxJointAPI` | PhysX 固有の設定で、Articulation 内の関節にだけ効く。`maxJointVelocity`（回転は度/秒）、`jointFriction`（無次元の摩擦係数で、静止摩擦・クーロン摩擦に近い）、`armature`（アクチュエータの慣性を関節空間に足す値）[6] | `physx.usda` で付加 |
| `PhysxJointAxisAPI:<自由度>` | 軸ごとの PhysX 設定。`armature`、`maxJointVelocity`、`staticFrictionEffort`、`dynamicFrictionEffort`、`viscousFrictionCoefficient`。`PhysxJointAPI` と重なる設定はこちらが優先される [7] | 未使用 |
| `PhysxDrivePerformanceEnvelopeAPI:<自由度>` | アクチュエータの速度－トルク特性（`maxActuatorVelocity`、`speedEffortGradient`、`velocityDependentResistance`）。同じ軸に `PhysicsDriveAPI` が必要 [7] | 未使用 |
| `NewtonMimicAPI`（Newton スキーマ） | ある関節（follower）を別の関節（leader）に連動させる。`newton:mimicJoint` で leader を指し、`follower = mimicCoef0 + mimicCoef1 × leader` の関係を保つ。2 つの関節は同じ Articulation 内に置く必要がある [8] | 未使用 |
| `PhysxMimicJointAPI:<軸>` | 旧来の mimic。今回の Isaac Sim 同梱版では **非推奨（DEPRECATED）** とされ、新しく作るなら `NewtonMimicAPI` を使うよう案内されている [6][8] | 未使用 |
| `PhysxPhysicsGearJoint` / `PhysxPhysicsRackAndPinionJoint` | 2 つの回転関節をギア比で、または回転関節と直動関節をラック＆ピニオンでつなぐ [6] | 未使用 |
| `PhysxTendonAxisAPI` など | 腱による連動（腱駆動ハンドなど）。スキーマ上はまだ Draft API [6] | 未使用 |

このモデルでは、関節の幾何（型、軸、範囲）は `physics.usda` にあります。PhysX 用のゲインと速度上限は `physx.usda` から `over` で重ねています。USD のレイヤー合成のおかげで、同じ関節に物理エンジンごとの設定を差し替えて載せられます（`mujoco.usda` も同じ仕組みです）。

## 4. 現実の機構との対応

- **減速機付きモーター**：`RevoluteJoint` と `DriveAPI` で表します。アクチュエータの慣性は `armature`、摩擦は `jointFriction`（軸ごとに静止・動・粘性摩擦を分けるなら `PhysxJointAxisAPI`）、トルク上限は `maxForce` に入れます。速度－トルク特性は `PhysxDrivePerformanceEnvelopeAPI` で近似できます [7]。モーターの電流制御や減速機のバックラッシュそのものはスキーマで表現できないので、近似になります。
- **連動するグリッパー（2 本の指が同時に開閉するもの）**：
  - このモデルでは gripper_joint1/2 が独立した 2 本の直動関節で、連動の設定はありません。前回お伝えしたとおり、両方に指令する必要があります。
  - 実機のように 1 つのモーターで両指を動かすなら、片方の関節に `NewtonMimicAPI` を付けて、もう片方を leader として参照させる形になります [8]。
- **閉リンク（平行リンク、4 節リンク）**：Articulation は木構造しか扱えません。ループを閉じる 1 本の関節に `physics:excludeFromArticulation = true` を付け、Articulation の外にある通常の拘束（maximal joint）として解かせます [1][9]。
- **ベルトやワイヤー駆動**：ギアジョイントや腱で連動させるか、関節空間でまとめて 1 関節として扱うのが一般的です。
- **弾性関節（SEA など）**：`DriveAPI` の stiffness と damping をバネ・ダンパとみなして近似します。

## 5. Articulation：関節をまとめて 1 台のロボットにする

`PhysicsArticulationRootAPI` を剛体かその祖先の Prim に付けると、その下の関節ツリーが **reduced-coordinate articulation**（関節角を変数とする多体系）として一括で解かれます。個々の拘束を別々に解く方法に比べて、関節誤差が原理的にゼロ（関節がずれたり伸びたりしない）で、剛体間の質量比が大きくても安定するため、ロボットに向いています [1][9]。このモデルでは `physics.usda:28` で `base_link` に付いています。`sim.py` が探している `ArticulationRootAPI` はこれです。

## 注意点

- 角度の単位は USD では **度**、ROS では rad です [1]。距離の単位はステージ単位で、このモデルはメートルです。
- このモデルは URDF から変換されています。そのため `urdf:limit:velocity` などの `custom` 属性が残っていますが、物理エンジンは読みません。実際に効いている速度上限は `physxJoint:maxJointVelocity`（度/秒）です [6]。
- URDF の関節型は revolute、continuous、prismatic、fixed、floating、planar の 6 種類です。`<mimic>`（`joint`、`multiplier`、`offset`）もあります [10]。Isaac Sim の URDF インポーターは mimic を `NewtonMimicAPI` に変換します [11]。このモデルの元 URDF には mimic がないので、USD 側にも連動の設定がありません。ギアや閉リンクは URDF で表現できないため、USD に変換してから付け足すことになります。

## 参照

1. OpenUSD, [Overview of UsdPhysics](https://openusd.org/release/api/usd_physics_page_front.html)：関節の型、関節座標系、D6 とロック条件、角度の単位（度）、Articulation と `excludeFromArticulation`
2. OpenUSD, [UsdPhysicsJoint](https://openusd.org/release/api/class_usd_physics_joint.html)：`body0/1`、`localPos0/1`、`localRot0/1`、`excludeFromArticulation`
3. OpenUSD, [UsdPhysicsRevoluteJoint](https://openusd.org/release/api/class_usd_physics_revolute_joint.html)：`axis` の取りうる値（X/Y/Z）
4. OpenUSD, [UsdPhysicsLimitAPI](https://openusd.org/release/api/class_usd_physics_limit_a_p_i.html)：自由度のトークン、low > high でロック、既定値 ±inf
5. OpenUSD, [UsdPhysicsDriveAPI](https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html)：`angular` / `linear` / `rotX` などのインスタンス名、ドライブの式、`type`
6. PhysxSchema（Isaac Sim 6.1 同梱の `omni.usd.schema.physx` 110.3、`~/isaacsim/extscache/omni.usd.schema.physx-*/plugins/PhysxSchema/resources/schema.usda`）：`PhysxJointAPI`、`PhysicsJointStateAPI`、`PhysxMimicJointAPI`（DEPRECATED の記載）、ギア・ラック＆ピニオン関節、腱。Web 版: [Omniverse Physics Schema API](https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/latest/index.html)
7. NVIDIA Omniverse Physics, [PhysX Joint Schema](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/joints/physx_joint_schema.html)：`PhysxJointAxisAPI`、`PhysxDrivePerformanceEnvelopeAPI`
8. NVIDIA Omniverse Physics, [Newton Schema](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/schemas/newtonSchema.html)、Isaac Sim 6.1 [omni.usd.schema.newton](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/omni.usd.schema.newton/docs/index.html)：`NewtonMimicAPI`
9. NVIDIA PhysX SDK, [Articulations](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/Articulations.html)：reduced-coordinate の利点（関節誤差ゼロ）、木構造、ループの閉じ方、mimic
10. ROS, [urdf/XML/joint](https://wiki.ros.org/urdf/XML/joint)、[urdfdom_headers joint.h](https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/joint.h)：URDF の関節型と `<mimic>`
11. Isaac Sim 6.1, [isaacsim.asset.importer.utils](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.asset.importer.utils/docs/index.html)（CHANGELOG：URDF/MJCF インポーターは mimic を `NewtonMimicAPI` として出力する）
