# USD のスキーマ入門

USD ファイルに書かれた「属性」が何を意味するのかは、**スキーマ**で決まります。このページでは、スキーマとは何か、どの組織のスキーマがあるか、このプロジェクトの USD でどう使われているかを整理します。関節の定義そのものは [usd_joints.md](usd_joints.md) を参照してください。

## 1. スキーマとは

USD ファイルは、名前の付いた入れ物（**Prim**）と、それに付けた **属性**（名前と値の組）でできています。

```usda
def PhysicsRevoluteJoint "joint1" (
    prepend apiSchemas = ["PhysicsDriveAPI:angular"]
)
{
    float drive:angular:physics:stiffness = 500
}
```

USD のファイル形式そのものは、`stiffness = 500` が何を意味するのか知りません。ただの「名前と数値」です。

そこで、**「この名前の属性は、こういう意味で、こういう型・既定値を持つ」という取り決めの辞書** が別に用意されています。これがスキーマです [1]。たとえば `PhysicsDriveAPI` スキーマには、おおよそ次のようなことが定義されています [5][6]。

| 属性 | 型 | 既定値 | 意味 |
|---|---|---|---|
| `stiffness` | float | 0 | ドライブのばね定数 |
| `damping` | float | 0 | ドライブの減衰係数 |
| `maxForce` | float | inf | ドライブが出せる力の上限 |
| `type` | token | `"force"` | `"force"` か `"acceleration"` のどちらか |

物理エンジンは USD を読み込むとき、この辞書に沿って属性を解釈し、「joint1 は回転関節で、ばね定数 500 のモーターが付いている」と理解します。

## 2. スキーマの種類

| 種類 | 書き方 | 役割 | 1 つの Prim に付けられる数 |
|---|---|---|---|
| **型スキーマ**（IsA / Typed schema） | `def PhysicsRevoluteJoint "joint1"` | Prim が「何であるか」を決める | 1 つだけ [2] |
| **API スキーマ**（applied API schema） | `apiSchemas = ["PhysicsDriveAPI:angular", ...]` | 既存の Prim に機能を後付けする | 複数 [3] |

API スキーマは、さらに次の 2 つに分かれます [3]。

- **single-apply**：1 つの Prim に 1 回だけ付けます。例：`PhysxJointAPI`、`NewtonMimicAPI`
- **multiple-apply**：「インスタンス名」を付けて、同じ Prim に何回も付けられます。インスタンス名は属性名の一部になります。
  - 例：`PhysicsDriveAPI:angular` の `angular` がインスタンス名で、属性名は `drive:angular:physics:stiffness` になります。
  - 関節の自由度ごとに、別々のドライブやリミットを付けるための仕組みです。

## 3. スキーマは作った組織ごとに分かれている

スキーマは 1 冊の辞書ではありません。標準のものと、物理エンジンごとの拡張に分かれています。

| スキーマ群 | 作ったところ | 中身の例 | 誰が読むか |
|---|---|---|---|
| **UsdPhysics** | OpenUSD の標準 [4] | 回転関節・直動関節、`PhysicsDriveAPI`、`PhysicsLimitAPI`、`PhysicsArticulationRootAPI` | 物理を扱うソフトならどれでも読める共通語 |
| **PhysxSchema** | NVIDIA（PhysX 用）[7] | `PhysxJointAPI`（摩擦・armature）、ギア関節、腱、`PhysicsJointStateAPI` | PhysX 向けの追加設定 |
| **Newton スキーマ** | Newton プロジェクト（Linux Foundation 傘下。Disney Research、Google DeepMind、NVIDIA が始めた）[8][9] | `NewtonMimicAPI`、`NewtonArticulationRootAPI` | Newton 向けの追加設定。Isaac Sim でも使われている |
| **MjcPhysics** | MuJoCo 向けの拡張（Isaac Sim には `omni.usd.schema.mujoco` として同梱） | MuJoCo 固有の設定 | MuJoCo |

注意点：
- **名前の接頭辞だけでは、どのスキーマ群のものか判断できません。** `PhysicsJointStateAPI` は名前が `Physics` で始まりますが、定義されているのは UsdPhysics ではなく PhysxSchema です [7]。
- **標準の UsdPhysics には、関節の連動（mimic）の取り決めがありません** [4]。連動を書くときは拡張スキーマを使います（§5）。

## 4. このプロジェクトの USD での例

`usd/reBot_B601_DM/payloads/Physics/physics.usda` と `physx.usda` から抜粋します。1 つの Prim に、複数のスキーマ群から来た機能が混ざって付いています。

```usda
# base_link（physics.usda:28）
prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsArticulationRootAPI",   ← UsdPhysics（標準）
                      "NewtonArticulationRootAPI",                            ← Newton スキーマ
                      "PhysicsMassAPI"]                                       ← UsdPhysics（標準）

# joint1（physx.usda:31）
prepend apiSchemas = ["PhysicsDriveAPI:angular",       ← UsdPhysics（標準）
                      "PhysicsJointStateAPI:angular",  ← PhysxSchema（名前は紛らわしいが PhysX 側）
                      "PhysxJointAPI"]                 ← PhysxSchema
```

## 5. 例：mimic（関節の連動）の置き場所が変わった

「関節を連動させる設定を、どのスキーマの書き方で書くか」が、Isaac Sim の版によって変わっています。

- **以前**：PhysX 用の `PhysxMimicJointAPI` で書いていました。
- **いま（Isaac Sim 6.1）**：Newton スキーマの `NewtonMimicAPI` で書くのが正式です。同梱の PhysxSchema 定義には、`PhysxMimicJointAPI` について「DEPRECATED: use NewtonMimicAPI (single-apply) from the Newton schema instead. ... kept for backward compatibility」と書かれています [7]。

Newton スキーマを使っても、**シミュレーションエンジンが Newton に切り替わるわけではありません。** Isaac Sim の URDF/MJCF インポーターは mimic を `NewtonMimicAPI` として出力し、実行時もその記述を直接読む、と CHANGELOG に書かれています [10]。変わったのは「どのスキーマの書き方で書くか」だけです。

### 書き方のイメージ（グリッパーを連動させる場合）

`NewtonMimicAPI` は、`follower = mimicCoef0 + mimicCoef1 × leader` の関係を保つ single-apply の API スキーマです [8][11]。follower は自分自身の関節、leader は `newton:mimicJoint` で指す関節です。

```usda
over "gripper_joint2" (
    prepend apiSchemas = ["NewtonMimicAPI"]
)
{
    rel newton:mimicJoint = </.../gripper_joint1>  # 追従する相手（leader）
    float newton:mimicCoef0 = 0                     # オフセット
    float newton:mimicCoef1 = 1                     # 倍率：joint2 = 0 + 1 × joint1
}
```

こう書くと、`gripper_joint1` に指令するだけで `gripper_joint2` も同じだけ動くようになります。このプロジェクトの USD には、まだこの設定はありません。

## 6. スキーマ定義の実物を見る

スキーマの定義は、それ自体が USD ファイル（`generatedSchema.usda`）として配布されています。手元の Isaac Sim では次の場所にあります（バージョン番号の部分は環境によって異なります）。

| スキーマ群 | 定義ファイル |
|---|---|
| UsdPhysics | `~/isaacsim/extscache/omni.usd.libs-*/bin/usd/usdPhysics/resources/generatedSchema.usda` |
| PhysxSchema | `~/isaacsim/extscache/omni.usd.schema.physx-*/plugins/PhysxSchema/resources/generatedSchema.usda`（説明付きの `schema.usda` も同じ場所） |
| PhysxSchema の追加分 | `~/isaacsim/extscache/omni.usd.schema.physx-*/plugins/PhysxSchemaAddition/resources/generatedSchema.usda` |
| Newton スキーマ | `~/isaacsim/exts/omni.usd.schema.newton/usd/schema/newton/newton_usd_schemas/generatedSchema.usda` |

たとえば、ある API にどんな属性があるかは次のように調べられます。

```bash
f=$(ls ~/isaacsim/extscache/omni.usd.schema.physx-*/plugins/PhysxSchema/resources/schema.usda)
awk '/class "PhysxJointAPI"/{f=1} f{print} f&&/^}/{exit}' "$f"
```

自作のスキーマも、`schema.usda` を書いて `usdGenSchema` で生成できます [12]。

## まとめ

- スキーマは「USD に書いた属性の意味を決める辞書」です。
- 型スキーマは Prim が何であるかを決め、1 つの Prim に 1 つだけ付きます。API スキーマは機能の後付けで、複数付けられます。
- 共通語は OpenUSD 標準の UsdPhysics です。PhysX や Newton など、エンジン固有の機能は各拡張スキーマで補います。
- mimic は標準の UsdPhysics にはなく、Isaac Sim 6.1 では Newton スキーマの `NewtonMimicAPI` で書くのが正式です。

## 参照

1. OpenUSD Glossary, [Schema](https://openusd.org/release/glossary.html#usdglossary-schema)
2. OpenUSD Glossary, [IsA Schema](https://openusd.org/release/glossary.html#usdglossary-isaschema)：「a prim can subscribe to at most one IsA schema - unlike API schemas」
3. OpenUSD Glossary, [API Schema](https://openusd.org/release/glossary.html#usdglossary-apischema)：non-applied / single-apply / multiple-apply、インスタンス名、`apiSchemas` メタデータ
4. OpenUSD, [Overview of UsdPhysics](https://openusd.org/release/api/usd_physics_page_front.html)
5. OpenUSD, [UsdPhysicsDriveAPI](https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html)
6. Isaac Sim 6.1 同梱の UsdPhysics 定義（`omni.usd.libs` の `usdPhysics/resources/generatedSchema.usda`）：`PhysicsDriveAPI` の既定値
7. Isaac Sim 6.1 同梱の PhysxSchema 定義（`omni.usd.schema.physx` 110.3 の `PhysxSchema/resources/schema.usda`）：`PhysicsJointStateAPI`、`PhysxMimicJointAPI` の DEPRECATED 記載。Web 版：[Omniverse Physics Schema API](https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/latest/index.html)
8. NVIDIA Omniverse Physics, [Newton Schema](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/schemas/newtonSchema.html)、Isaac Sim 6.1 [omni.usd.schema.newton](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/omni.usd.schema.newton/docs/index.html)
9. [newton-physics/newton](https://github.com/newton-physics/newton)（README：Linux Foundation project、Disney Research・Google DeepMind・NVIDIA が開始）
10. Isaac Sim 6.1, [isaacsim.asset.importer.utils](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.asset.importer.utils/docs/index.html)（CHANGELOG：インポーターは mimic を `NewtonMimicAPI` として出力し、実行時はそれを直接読む）
11. Isaac Sim 6.1 同梱の Newton スキーマ定義（`omni.usd.schema.newton` の `newton_usd_schemas/generatedSchema.usda`）：`newton:mimicJoint`、`mimicCoef0`、`mimicCoef1`、`mimicEnabled`
12. OpenUSD, [Generating New Schema Classes](https://openusd.org/release/tut_generating_new_schema.html)
