#!/usr/bin/env bash
# 安装 GitHub 版 Moon systemd unit；uwb-follow 安装后默认 disable（跟随走 mode_arbiter）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cp "${ROOT}/uwb-follow.service" /etc/systemd/system/uwb-follow.service
cp "${ROOT}/brain/moon-arbiter.service" /etc/systemd/system/moon-arbiter.service
cp "${ROOT}/vision/zed-obstacle.service" /etc/systemd/system/zed-obstacle.service
cp "${ROOT}/voice/moon-kws.service" /etc/systemd/system/moon-kws.service
systemctl daemon-reload
systemctl disable --now uwb-follow.service || true
echo "已安装 GitHub 版 uwb-follow.service（当前 disabled）。"
echo "跟随请用: python3 ${ROOT}/brain/mode_arbiter.py"
