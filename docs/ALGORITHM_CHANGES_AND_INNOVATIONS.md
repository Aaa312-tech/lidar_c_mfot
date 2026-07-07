# Algorithm Changes And Innovations

本文总结 C++ LiDAR router 相对原 Python LiDAR router 的算法改变、工程创新点和验证方法。这里的“创新”指工程和算法实现上的改造，不宣称为新的学术理论。

当前 `lidar_c_inovation_2` 版本是 MFOT-only 对照版本：公共布线行为已同步回 `lidar_c`，只保留 MFOT 所需的最小新增模块和 A* 钩子。早期实验中额外的 crossing/postprocess repair、real-port repair 和 fanout-specific fallback 不属于当前版本的算法差异。

## 1. 总体目标变化

原 Python LiDAR router 的重点是功能完整和研究验证；C++ 版本的目标是：

```text
保留 Python 原版布线语义
显著加速核心 route 阶段
输出可直接打开和验证的 GDS
用 DRC、route trace、GDS XOR 证明结果质量
避免按 case 或标准 GDS 做硬编码
```

因此本次改造不是简单重写 A*，而是把 Python router 的中间语义拆开，再逐层移植和验证。

## 2. 继承的核心语义

C++ 版本保留了 Python LiDAR 的关键布线模型：

```text
benchmark YAML 输入模型
component bbox / port / net / group 数据模型
bitmap blockage 和 port-access 网格
crossing-aware A* 搜索
propagation / bend / crossing / congestion 综合代价
rip-up and reroute
history congestion
grid path to micron path 后处理
crossing split 和 crossing cell 插入
gdsfactory/kfactory GDS 渲染
```

这些语义是结果接近标准 GDS 的基础。没有这层兼容，单纯实现一个通用 maze router 很容易 DRC 通过但 GDS 形态完全不同。

## 3. 主要算法改变

### 3.0 MFOT-only Control Scope

当前版本保留的 MFOT 差异包括：

```text
lidar_mfot.h / lidar_mfot.cpp
LidarRouteConfig 中的 MFOT 参数
routeAllNetsGrid() 中的 MFOT plan 构建
routeSingleNetGrid() 中基于 MFOT priority 的 A* heuristic weight
flow/YAML 中的 MFOT 过程指标输出
```

实际生效的主链路是：

```text
全局 MFOT plan -> 每条 net 的 priority/free-energy -> mfotSearchWeightForNet -> A* costF
```

也就是说，当前 MFOT 的主要作用是调整 A* 启发式权重，减少大规模 case 中的无效搜索展开。默认参数下以下分量没有实际施加代价或排序影响：

```text
mfotHardCorridor = false
mfotOutsidePenalty = 0.0
mfotPotentialScale = 0.0
mfotHistoryScale = 0.0
mfotPriorityScale = 0.0
```

因此当前结果适合用来做“只保留 MFOT plan/heuristic weighting”的对照实验，不应与包含额外 repair/fallback 的历史版本混淆。

### 3.1 Native Runtime View

Python 原版直接在 Python object graph 上读取实例、端口、net 和 blockage。C++ 版本增加了 `PicdbLidarView`，把 PIC-DB design 转换为 LiDAR 专用 runtime view。

改变点：

```text
把 PIC-DB component/port/net 映射成 LiDAR routing objects
保留原 net name，避免输出 trace 和标准 case 对不上
统一 micron 坐标、grid 坐标、DBU 坐标之间的转换
把 route result、post path、crossing 信息以 YAML 形式导出
```

价值：

```text
router core 不直接依赖 Python object
route trace 可复现、可比较
后续可以替换输入来源，例如 PICBench bridge
```

### 3.2 Python-Compatible Bitmap Construction

早期 C++ 初版虽然也有 blockage bitmap，但和 Python 原版在边界取整、inclusive range、`abs()`、port 区域标记上存在差异，导致 access grid 偏移。

修正后的策略：

```text
严格对齐 Python 的 bbox 膨胀和整数截断
保留 Python 的 blockage/port/compound/waveguide 分类语义
把 component blockage、port access、已布 waveguide 写入同一查询模型
```

价值：

```text
A* 搜索看到的可走区域与 Python 原版一致
小的整数边界差异不会被放大成整条路径差异
```

### 3.3 Port Access And Spread Alignment

光子端口不能简单从 port center 出发。C++ 版本对齐并重构了 port access 逻辑：

```text
单端口直出 access
多端口 spread access
close-port S-bend spread
group reverse 端口方向
blocked meanLoc 推进
port access grid 标记
```

关键改变是去掉 C++ 初版中过度“自创”的 widening 和 other-blockage 特殊行为，让 access 规则回到 Python LiDAR 的语义，再在此基础上处理 C++ 数据结构。

价值：

```text
减少 source/target 附近的不可解释偏移
减少短 S-bend 和 invalid access waveguide
提高标准 GDS 形态一致性
```

### 3.4 Crossing-Aware A* State

C++ A* 状态不是只有 `(x, y)`，而是包含：

```text
x
y
orientation
crossing budget
crossing net
```

neighbor 类型包含：

```text
straight
90-degree bend
45-degree bend
crossing
S-bend in x/y direction
```

代价函数综合：

```text
传播长度
弯曲损耗
crossing 损耗
history congestion
spacing penalty
illegal crossing penalty
```

改变点：

```text
把 Python 动态状态显式化成 C++ struct/hash/key
把 crossing budget 纳入 visited/open-set 状态
把 crossing net 记录进 route result 供 post-process 使用
```

价值：

```text
可控 crossing 不会被误判成普通冲突
同一坐标但不同 crossing 状态不会被错误剪枝
MMI 这类需要 crossing 的 case 可以得到接近人工布线的形态
```

### 3.5 Deterministic Python-Like Ordering

Python router 中一些 set/dict 遍历顺序会影响 net order、candidate order 和 crossing 选择。C++ 初版如果直接使用 unordered 容器，结果会出现不可复现或与 Python 不一致的问题。

C++ 版本增加了 Python-like deterministic set 行为：

```text
稳定的元素插入/遍历顺序
可重复的 net/crossing candidate 顺序
避免不同编译器或 hash seed 导致结果漂移
```

价值：

```text
同一输入多次运行结果稳定
trace diff 更可信
更容易逼近 Python 原版和标准 GDS
```

### 3.6 Rip-Up / Reroute Policy Alignment

复杂 case 一次 A* 很难完成。C++ 版本保留并显式化了以下策略：

```text
先按 order 路由
失败时尝试 relaxed/fallback 策略
crossing route 与 no-crossing route 比较
必要时 rip-up crossing 相关 net
用 history map 惩罚反复拥塞区域
重新写入 bitmap occupancy
```

改变点：

```text
route result 中记录 success/strict/short_sbend/crossings
把局部失败原因和 DRC 结果输出到可检查文件
把 route core、writeback、render 的耗时拆开记录
```

价值：

```text
失败不再只是没有 GDS，而是能定位到 route、writeback、DRC 或 render 层
多 case regression 可以稳定统计质量和耗时
```

### 3.7 Post-Processing And Crossing Split

GDS 差异最常出现在 post-process，而不是 A* 本身。C++ 版本把 grid path 到 GDS route 的过程拆成可检查步骤：

```text
collinear point cleanup
grid point to micron point conversion
port direction alignment
crossing pair detection
path split around crossing
crossing cell insertion metadata
short connector handling
post path export
```

改造重点：

```text
保留 pre-split origin path 供 trace 对照
保留 post path 供 GDS render 对照
对 crossing hotspot 做 layer XOR 定位
```

价值：

```text
能解释“路径看起来对但 GDS XOR 不为 0”的问题
把 MMI 剩余差异定位到 crossing geometry 周边，而不是误判为 A* 大范围错误
```

### 3.8 Version-Locked GDS Rendering

C++ 版本没有直接手写所有 gdsfactory photonic cells，而是继续通过 gdsfactory/kfactory 渲染最终 GDS。

改变点：

```text
route core 在 C++
component/crossing/path GDS 渲染仍复用 gdsfactory/kfactory
用固定 Python dependency lock 保持 cell geometry 和 metadata 稳定
```

价值：

```text
避免重写复杂 photonic cell 库
更容易与 Python LiDAR 和标准 GDS 的 cell geometry 对齐
减少 C++ route core 与 GDS cell library 的耦合
```

## 4. 工程创新点

### 4.1 Four-Level Validation Ladder

本次不是只看 viewer，而是建立了四层验证：

```text
route metrics: routes / crossings / length / per-net status
DB DRC: component, pin-access, route geometry markers
path trace: origin path / processed path / post path
GDS geometry: bbox / layer area / XOR / hotspot
```

价值：

```text
能判断问题在哪一层出现
避免把 render 版本差异误判成 route 算法错误
避免只凭肉眼相似就认为成功
```

### 4.2 Standard-GDS As Validator, Not Teacher

标准 GDS 只进入 comparison script，不进入 router。

明确禁止：

```text
读取标准 GDS 后复制 polygon
按 case 名输出固定 route
按 net id 写专用绕线规则
根据 XOR hotspot 手工补 polygon
```

允许：

```text
固定 benchmark list 作为 regression set
固定标准文件名作为 comparison input convention
使用通用几何 tolerance
使用版本兼容 shim 处理 gdsfactory API 差异
```

价值：

```text
结果质量来自算法语义对齐，不来自答案泄漏
router 可以迁移到新 case，而不是只会复现三个标准 GDS
```

### 4.3 Portable Regression Harness

新增 regression helper：

```text
run_lidar_benchmark_regression.py
run_all_cases.ps1
compare_gds_geometry.py
compare_lidar_net_metrics.py
```

特点：

```text
支持命令行参数和环境变量指定路径
输出 CSV/JSON summary
保留 stdout/stderr 和 DRC summary
支持 timeout
Windows 下会终止超时进程树
```

价值：

```text
迁移到新机器后可以一键复现
长 case 不会因为子进程残留污染下一轮
结果可以作为 CI 或 release artifact 的基础
```

### 4.4 Reproducible Artifact Layout

输出约定：

```text
<case>_cpp.gds
<case>_cpp.stdout.txt
<case>_cpp.stderr.txt
<case>_cpp_picdb_flow/cpp/lidar_route_result.yml
<case>_cpp_picdb_flow/cpp/db_drc_summary.txt
```

价值：

```text
每个 case 的 GDS、route trace、DRC、日志可以一起归档
方便定位同一 case 的不同版本差异
方便 GitHub release 或外部复现实验
```

## 5. 速度来源

C++ 版本的主要加速来自：

```text
A* open/closed set 和 neighbor expansion 在 C++ 内完成
bitmap/DRC 查询变成连续内存和原生结构查询
route result 只在边界处序列化，不在每步搜索中跨 Python
同一 route result 被复用到 summary、writeback、DRC、GDS render
```

当前端到端瓶颈仍包括：

```text
YAML conversion
gdsfactory route object construction
KLayout/GDS write
large MMI search space
```

后续可继续优化：

```text
把更多 YAML conversion 移到 C++
优化 A* key/hash/open-set 内存布局
给 group routing 做局部并行
减少 render 阶段重复初始化
建立 CI regression 的小/中/大全套分层
```

## 6. 当前质量边界

已达到：

```text
clements_8x8 standard XOR = 0
multiportmmi_8x8 and 16x16 DRC clean
multiportmmi standard overlap ratio > 0.99996
default 9-case regression can run end to end
```

仍需改进：

```text
MMI crossing geometry 仍有极小 XOR
MRR 8x8 / 16x16 仍有 route geometry markers
end-to-end runtime 仍受 Python render 影响
```

这些剩余问题都应继续通过通用算法规则解决，而不是通过 case-specific route patch 或标准 GDS polygon copying 解决。
