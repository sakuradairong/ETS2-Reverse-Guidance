# ETS2 Reverse Guidance v0.2.2

欧卡 2 的 Windows x64 倒车轨迹辅助。基于 [yyysheng 的 ETS2 Reverse Posture Assistant v0.11.1](https://github.com/yyysheng/ETS2mods/tree/reverse-posture-assistant-v0.11.1/mods/ETS2_Reverse_Posture_Assistant)，以 MIT 许可证公开项目源码。

提供牵引车与第一节挂车轨迹、车头外摆线、挂车预测终点轮廓和车挂夹角趋势。驾驶操作由玩家完成。

## 兼容范围

- 新增适配：ETS2 Steam public **1.61.1.9，Windows x64**（rev. `05126873bfae`）。已核对本机 PE 的四个 Hook、四个模型辅助函数、函数边界和牵引车/挂车渲染调用；游戏内显示仍待验证。
- 1.61.1.9 可执行文件 SHA256：`B70C2357CC56EB5834EE47FEA3C60B2F2D4591383FF2AC479CFD3CD254301B8C`。
- 先前已做单机显示验证：1.61.1.1（可执行文件 SHA256 `2014BFCC850A06108F4CA3C233A5BB6D31D7D83346C5D449F94C8E0675E5199A`）。这项结果不能代替 1.61.1.9 的游戏内验收。
- 源码保留 1.61.1.0、1.60.1.7 和一个旧版特征匹配配置；本次没有重新完成这些版本的游戏内验证。支持表见 `src/entity_runtime/build_profiles.hpp`。
- 通过版本/哈希与 Hook 特征检查后才启用原生渲染；不匹配时禁用相关功能。
- **TruckersMP 的加载兼容性与规则许可尚未验证，本次发布面向单机使用。**

## 下载与安装

从 [Releases](https://github.com/sakuradairong/ETS2-Reverse-Guidance/releases) 选择对应游戏版本的 `ETS2-Reverse-Guidance-v<版本>-windows-x64.zip`。已发布的 v0.2.1 没有 1.61.1.9 配置；v0.2.2 的本地验证包位于 `dist/`，发布状态以 Releases 页面为准。`SHA256SUMS.txt` 提供压缩包校验值。解压后：

1. 完全退出欧卡 2。若安装过旧版，备份原来的 DLL、INI 与 `ets2_reverse_posture_tmp` 资源目录；在模组管理器停用原版 Reverse Posture Assistant `.scs`，避免重复轨迹。
2. 把 `plugins/ETS2ReverseEntityRuntime.dll` 和 INI 放到游戏目录的 `bin/win_x64/plugins/`。目录不存在时创建。升级时可以保留自己的 INI。
3. 把 `home/ets2_reverse_posture_tmp/` 放到实际游戏用户资料目录，例如 `Documents/Euro Truck Simulator 2/ets2_reverse_posture_tmp/`。使用 OneDrive 或 `-homedir` 时，以实际目录为准；不额外创建物理 `home` 文件夹。
4. 启动单机存档，挂倒挡检查地面提示。DLL 需要 Microsoft Visual C++ 2015–2022 Redistributable x64。

回退时完全退出游戏并恢复备份。首次安装后的卸载只需移除本插件 DLL/INI 与本插件资源目录；不要移除其他插件或存档。

## 提示含义

| 显示 | 含义 |
| --- | --- |
| 蓝色轨迹 | 牵引车后角路径，以及转向时向外摆动的一个前角路径 |
| 橙色轨迹 | 第一节挂车后角路径 |
| 淡橙色终点轮廓 | 保持当前实际轮角、牵引车继续倒退指定距离时的挂车预测姿态 |
| 车头旁夹角标记 | 当前车挂夹角，以及倒退 1 米后的预测趋势：绿色收直、橙色扩大、淡色变化小 |

**“稳定”只表示夹角变化小，大夹角仍可能很难修正。当前没有独立的大夹角危险警示。**

低速、平坦路面且实际轮角数据可用时，默认显示 5 米预测。超过 5 km/h 后逐渐缩短，到 10 km/h 时缩到 2 米，并隐藏终点轮廓。估计接地坡度超过 4° 时同样收短；达到 10° 时缩到 2 米。实际轮角或车速不可用时保留 2 米轨迹并隐藏终点。挂倒挡但实际向前滚动超过 0.3 m/s 时隐藏提示。以上阈值是保守显示策略，尚未经过游戏内误差校准。

预测使用固定轮角、无侧滑的低速运动学模型。它不读取障碍物，也没有模拟轮胎摩擦、载重惯性和坡道溜车动力学；轮廓尺寸由车轮/挂钩配置估算，不能当作精确碰撞边界。当前只显示第一节挂车，多挂组合的完整扫掠范围不受保证。

## 配置

编辑与 DLL 同目录的 `ETS2ReverseEntityRuntime.ini` 后完全重启游戏：

```ini
[guidance]
enabled=1
endpoint_outline=1
front_swing=1
articulation_indicator=1
prediction_distance_cm=500

[diagnostics]
enabled=0
```

预测距离限制为 200–800 厘米，动态策略只会缩短该距离。诊断默认关闭；加载及错误信息仍可在 `game.log.txt` 与游戏目录的 `ETS2ReverseEntityRuntime.log` 中查看。

## 从源码构建

需要 Windows、Visual Studio 2022 / Build Tools（v143 C++ 工具和 Windows SDK）、Python 3.10+。依赖头文件、MinHook 源码和必要资源已包含在仓库中，不依赖作者的下载目录。

```powershell
git clone https://github.com/sakuradairong/ETS2-Reverse-Guidance.git
cd ETS2-Reverse-Guidance
powershell -ExecutionPolicy Bypass -File .\Build.ps1
```

脚本定位 MSBuild，构建 DLL 与两组测试，验证资源引用链，再在 `dist/` 生成发布 ZIP 和 SHA256SUMS。每次打包使用新的输出目录；再次打包前将旧 `dist/` 重命名或移走。构建过程不会安装插件或修改游戏/存档。

可使用 `./Build.ps1 -GameExecutable '实际游戏目录/bin/win_x64/eurotrucks2.exe'` 对本机文件增加只读检查：根据实际 SHA256 选择配置，验证 Hook 和辅助函数特征，以及牵引车/挂车渲染调用。该检查不会启动游戏，也不能证明运行时显示正常。

如需从原始下载包重新提取资源，可设置 `ETS2_REVERSE_UPSTREAM` 并在不含仓库资源的副本中运行资源准备工具；常规构建直接使用随仓库提供的基础资源并生成新增颜色。

## 验证情况

- Release x64 构建及构建配置匹配测试；覆盖 1.61.1.9 的精确哈希、每个 Hook/辅助函数特征损坏拒绝和未知新版本哈希拒绝。
- 105 组距离、转向与夹角组合，以及数据缺失、接地坡度、抬轴切换、缓存和几何边界检查。
- PMD → MAT → TOBJ → DDS 引用闭包和 8 个畸形格式拒绝用例；ZIP 内 DLL、INI、35 个资源逐项校验。
- 1.61.1.1 的单机截图与日志确认轨迹、外摆与夹角提示已显示。1.61.1.9 的显示、实际倒车预测误差、动态隐藏和帧率影响尚未完成受控游戏内验收。

## 来源与许可证

项目修改保留原作者 MIT 许可与署名。第三方 MinHook/SCS SDK 依赖及素材来源见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。发布包提供完整第三方许可文本。游戏及其商标属于 SCS Software；这些权利不因项目 MIT 许可证而改变。
