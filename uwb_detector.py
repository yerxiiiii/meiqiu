#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UWB USB 设备热插拔监测与诊断工具
1. 循环扫描串口，自动识别新接入或拔出的 USB 串口设备。
2. 过滤并识别 Silicon Labs CP210x 芯片（常见 UWB 设备）。
3. 自动诊断读写权限，如遇到 Permission Denied 自动输出解决方案。
"""

import time
import serial
import serial.tools.list_ports

# 常见 CP210x (Silicon Labs) 的 USB Vendor ID
CP210X_VIDS = [0x10C4]

def check_permission(port_device) -> tuple:
    """
    测试是否拥有串口读写权限
    返回: (has_permission, error_msg)
    """
    try:
        ser = serial.Serial(port_device)
        ser.close()
        return True, ""
    except serial.SerialException as e:
        err_msg = str(e)
        if "Permission denied" in err_msg or "PermissionError" in err_msg:
            return False, "权限不足 (Permission Denied)"
        return False, err_msg
    except Exception as e:
        return False, str(e)

def get_usb_ports() -> dict:
    """
    扫描当前 USB 串口设备，返回设备路径到详细信息的字典
    """
    ports = serial.tools.list_ports.comports()
    usb_devices = {}
    for p in ports:
        # 只监测 USB 串口
        if p.vid is not None:
            is_cp210x = p.vid in CP210X_VIDS
            usb_devices[p.device] = {
                'device': p.device,
                'name': p.name,
                'description': p.description,
                'hwid': p.hwid,
                'vid': f"{p.vid:04X}" if p.vid else "None",
                'pid': f"{p.pid:04X}" if p.pid else "None",
                'is_cp210x': is_cp210x
            }
    return usb_devices

def main():
    print("="*60)
    print("  UWB USB 串口设备热插拔诊断监测工具已启动")
    print("  正在实时监听 USB 设备插拔，退出请按 Ctrl+C ...")
    print("="*60)

    # 首次扫描获取当前设备列表
    last_devices = get_usb_ports()
    if last_devices:
        print(f"\n[当前已连接的 USB 串口] (共 {len(last_devices)} 个):")
        for dev, info in last_devices.items():
            is_uwb_hint = " 🌟 [建议UWB设备]" if info['is_cp210x'] else ""
            print(f" 🔌 设备路径: {dev}")
            print(f"    - 芯片描述: {info['description']}")
            print(f"    - 硬件ID: {info['hwid']}{is_uwb_hint}")
            
            # 权限检查
            has_perm, err = check_permission(dev)
            if has_perm:
                print("    - 读写权限: \033[92m正常 (可读写)\033[0m")
            else:
                print(f"    - 读写权限: \033[91m异常 ({err})\033[0m")
                print("      👉 修复建议: 运行 'sudo usermod -aG dialout $USER' 并重启，或使用 'sudo' 执行脚本。")
    else:
        print("\n[当前未检测到任何 USB 串口设备]")

    print("\n" + "-"*40 + " 开始监听插拔 " + "-"*40)

    try:
        while True:
            time.sleep(1.0)
            current_devices = get_usb_ports()

            # 1. 查找新插入的设备
            for dev in current_devices:
                if dev not in last_devices:
                    info = current_devices[dev]
                    is_uwb_hint = " 🌟 [疑似 UWB 设备]" if info['is_cp210x'] else ""
                    print(f"\n\033[92m[➕ 发现新插入设备]\033[0m 设备路径: {dev}")
                    print(f"   - 芯片描述: {info['description']}")
                    print(f"   - USB VID:PID = {info['vid']}:{info['pid']}{is_uwb_hint}")
                    
                    # 进行权限诊断
                    has_perm, err = check_permission(dev)
                    if has_perm:
                        print("   - 权限状态: \033[92m可正常读取\033[0m")
                    else:
                        print(f"   - 权限状态: \033[91m无法读取 ({err})\033[0m")
                        print("     👉 修复命令: sudo usermod -aG dialout $USER 并且重启机器人")

            # 2. 查找被拔出的设备
            for dev in last_devices:
                if dev not in current_devices:
                    info = last_devices[dev]
                    print(f"\n\033[91m[➖ 设备已被拔出]\033[0m 设备路径: {dev} ({info['description']})")

            last_devices = current_devices

    except KeyboardInterrupt:
        print("\n监测工具已安全关闭。")

if __name__ == '__main__':
    main()
