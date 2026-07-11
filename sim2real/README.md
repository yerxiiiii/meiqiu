# sim2real 侧配置快照（与 amp_right_hold 相关）

本目录保存 **运控配置快照**，便于决策层仓库自包含。

真机生效路径：`$SIM2REAL_WS/install/share/sim2real/`（推荐 `~/sim2real` 软链）。

## 安装到本机运控

```bash
# 先保证能找到 workspace
ln -sfn /path/to/your/sim2real_ws ~/sim2real
# 或: export SIM2REAL_WS=/path/to/your/sim2real_ws

./scripts/install_amp_right_hold.sh
```

## 文件

| 路径 | 作用 |
|------|------|
| `config/walk/amp_pi_plus_20dof_right_hold.yaml` | **`amp_right_hold`** 策略本体（右臂 hold） |
| `config/walk/amp_pi_plus_20dof.yaml` | 基准 `amp`（right_hold 由它改出） |
| `config/walk/lr.yaml` | `lr` 下半身走（brain 也会切） |
| `config/pi_plus_22dof_config/pi_plus_22dof_rl_config.yaml` | 注册：`amp` → `amp_right_hold` → `lr` → `footstep` |
| `config/pi_plus_22dof_config/pi_plus_22dof_pd_config.yaml` | PD / default 姿态（右臂 hold 角已验证） |
| `config/walk/README_amp_right_hold.md` | 与 `docs/AMP_RIGHT_HOLD.md` 同文 |

决策层对接说明见：[`../docs/AMP_RIGHT_HOLD.md`](../docs/AMP_RIGHT_HOLD.md)。

**未上传**：`.rknn` / `.trt` 模型权重（体积大，机器本地已有，与原版 `amp` 共用）。
