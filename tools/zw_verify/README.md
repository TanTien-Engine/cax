# zw_verify — ZW3D 转换基线比较 + 按特征二分定位

把 `cadcvt` 重放出的几何体与 ZW3D 自己导出的真值做自动比对，回答两个问题：

1. **这张图纸转换得对不对？**（全量比对：实体数量 / 包围盒 / 体积 / 面积 / 逐面匹配）
2. **不对的话，是哪个特征开始错的？**（`--bisect`：对照插件记录的逐特征状态真值二分）

## 数据来源（zw_export 插件每次导出写出）

| 文件 / 字段 | 含义 |
|---|---|
| `<part>.cax.json` | 参数化历史快照（cadcvt 重放的输入） |
| `<part>.cax.step` | ZW3D 导出的最终几何（**最终真值**） |
| `<part>.cax.feat<K>.step` | 第 K 个特征*新建*的形体（opaque 特征的 authored 输入，**不是**累积状态） |
| 每特征 `_state` 块 | 第 K 个特征执行后**整个零件**的累积状态：shape/face/edge 计数 + 包围盒（`CAX_FEAT_STATE=2` 时再加 面积/体积，mm 单位）。`--bisect` 的逐步真值 |
| `<part>.cax.state<K>.step` | 第 K 步的**累积**零件几何（`CAX_FEAT_STATE_STEP` 选中时导出），用于对二分出的特征做逐面级确认 |

`_state` 由新版 CaxExport.dll 写出（默认开启计数+包围盒；`CAX_FEAT_STATE=0` 关闭，`=2` 加质量属性）。
旧快照没有 `_state`，需要用新插件重导一次才能 `--bisect`。

## 用法

```powershell
# 运行时需要 OCCT DLL（项目已把配套 DLL 复制到 out\<Config>）：
$env:PATH = "D:\projects\tantien-cad\out\RelWithDebInfo;$env:PATH"
$zv = "D:\projects\tantien-cad\out\thirdparty\cax\tools\zw_verify\RelWithDebInfo\zw_verify.exe"

# 1) 全量比对（真值默认取同名 .cax.step）
& $zv part.cax.json

# 2) 只重放前 K 个特征；可对照指定真值 STEP，可把重放结果导出成 STEP 看
& $zv part.cax.json --max-feat 12 --step part.cax.state12.step
& $zv part.cax.json --max-feat 12 --dump replay12.step      # mm 单位，可丢回 CAD 里看

# 3) 二分定位第一个发散特征（需要 _state）
& $zv part.cax.json --bisect

# 4) 指定/全部前缀逐个探测（穷举版，绕过二分的单调性假设）
& $zv part.cax.json --states 4,16,21,47
& $zv part.cax.json --states all

# 5) 深挖：导出全部不匹配面（带质心+面积+法向），做位移聚类 / 结构取证
& $zv part.cax.json --detail-cap 400 > full.log
```

## 输出协议（行式、可机读）

```
INFO  features=56 opaque=56 states=56 ...
INFO  truth_filtered kept=1 solids ...  # 真值 STEP 里的自由线框已被剔除（ZW3D
                                        # "导出所有对象"会带上参考曲线，只污染
                                        # bbox/边数，不影响体积面积）
INFO  replay_msg <...>                  # 重放的软诊断：被静默跳过/无效果的特征
                                        # （如 dressup applied no blends）——
                                        # 有这行必须追查，它就是"哪步丢了"
CHECK <name> <ok|bad> <详情>           # 全量比对的各项检查
STATE feat=K solids=.. faces=.. vol=..  # 重放出的前缀状态（米制）
TRUTHSTATE feat=K n_shape=.. vol=..     # 插件记录的真值状态（已折算成米制）
PROBE feat=K verdict=good|bad topo=a/b  # 单点判定；topo 列是体数差（仅参考，
                                        # 独立体阵列在被吸收前 topo 合法不同）
BISECT first_bad=K last_good=J name=.. zw_type=..
DETAIL unmatched_*_face idx=.. centre=(..) area=.. n=(..)   # 不匹配面明细
VERDICT PASS|FAIL <json>
```

stderr 另有低层诊断（不进协议，但排障必看）：
`[resolve_edge] MISS`（修饰边解析失败：锚点漂移超容差）、
`[FILLET]/[DPRISM] WARNING`（OCCT 失败回退）、`CAX_GEO_LOG=1` 开全量
`[FILLET]` 逐边日志（含 etol/vtol 公差）。

退出码：0 = PASS / bisect 完成；1 = FAIL；2 = 用法或 IO 错误。

## 判定标准

- `solids` 数量必须一致；`faces`/`edges` 仅报告（两个内核对周期面的剖分本来就不同）。
- 包围盒最差角点偏差、体积、面积：相对误差 ≤ `--rel-tol`（默认 1e-4）。
- 全量比对再加双向逐面匹配（质心近邻 + 法向 + 面积），并区分
  `diff_class=geometry`（真几何差异）和 `fragmentation`（只是面剖分不同）。

## 二分工作流

1. `zw_verify part.cax.json` → FAIL？
2. 没有 `_state` 就用新插件重导（ZW3D 里 `~CaxExportRun`，或批处理队列）。
   要逐面确认时设 `CAX_FEAT_STATE_STEP=K1,K2`（或 `all`，注意体积大）。
3. `zw_verify part.cax.json --bisect` → `BISECT first_bad=K`，~log2(N) 次前缀重放。
4. 需要细看时：`--max-feat K --step part.cax.state<K>.step` 做逐面比对，
   或 `--max-feat K --dump out.step` 把重放结果丢回 CAD 目检。

注意：二分假设"一旦发散、后续保持发散"。若怀疑后面的特征掩盖了前面的偏差
（如一刀切掉了出错区域），用 `--states all` 全扫。

## 单位

重放体在米制（`length_unit:mm` → scale 0.001）；真值 STEP 由 OCCT 按 mm 读入、
`_state` 由插件按 mm 记录，比对前都按 reader 的 UnitScale 折算到同一空间。
`CAX_ZW_SCALE1` 强制两侧都用 1.0（mm），与编辑器导入一致。
