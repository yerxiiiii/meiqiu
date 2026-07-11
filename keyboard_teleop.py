#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
专门针对 Pi Plus 机器人的键盘控制脚本。
特色：
1. 运行前自动杀掉 joy_teleop 以防指令冲突。
2. 保持发布包含 lt=1.0, rt=1.0 的安全心跳的 /joy_msg，并映射对应摇杆数值。
3. 同时发布 /cmd_vel，配合 linear.z = 1.0 作为运动控制使能信号。
4. 退出时自动恢复常规 joy_teleop 节点。
"""

import rospy
from geometry_msgs.msg import Twist
from sim2real_msg.msg import Joy
import sys
import select
import termios
import tty
import subprocess
import time
import threading
from pathlib import Path

# moon repo root for common.sim2real_env
_MOON = str(Path(__file__).resolve().parent)
if _MOON not in sys.path:
    sys.path.insert(0, _MOON)

msg = """
---------------------------------------------
Pi Plus 机器人键盘控制工具 (本地高频双通道版)
---------------------------------------------
控制键位 (与经典 teleop_twist_keyboard 一致):
        u    i    o
        j    k    l
        m    ,    .

i : 向前             , : 向后
j : 原地左转         l : 原地右转
u : 前左偏           o : 前右偏
m : 后左偏           . : 后右偏
k : 停止所有运动 (使能保持)

速度调节键:
q/z : 增加/减少线速度 10%
w/x : 增加/减少角速度 10%

按 Ctrl+C 退出脚本并恢复手柄默认控制。
---------------------------------------------
"""

# 按键移动映射 (x_dir, y_dir, yaw_dir)
moveBindings = {
    'i': (1, 0, 0),     # 前进
    'o': (1, -0.5, -0.5), # 前右偏
    'j': (0, 0, 1),     # 左转
    'l': (0, 0, -1),    # 右转
    'u': (1, 0.5, 1),   # 前左偏
    ',': (-1, 0, 0),    # 后退
    '.': (-1, 0.5, 0.5), # 后左偏
    'm': (-1, -0.5, -0.5),# 后右偏
    'k': (0, 0, 0),     # 停止
}

speedBindings = {
    'q': (1.1, 1.0),
    'z': (0.9, 1.0),
    'w': (1.0, 1.1),
    'x': (1.0, 0.9),
}

# 全局运动参数
speed = 0.20   # 默认线速度
turn = 0.40    # 默认角速度
x_dir = 0
y_dir = 0
yaw_dir = 0
is_running = True

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def publish_loop(joy_pub, cmd_pub):
    global x_dir, y_dir, yaw_dir, speed, turn, is_running
    rate = rospy.Rate(50)  # 50Hz
    
    while not rospy.is_shutdown() and is_running:
        # 1. 构造使能的 /joy_msg 锁扣数据
        joy_msg = Joy()
        joy_msg.lt = 1.0
        joy_msg.rt = 1.0
        
        # 摇杆数值映射 (这里主要针对旋转做物理拟合)
        joy_msg.l_vertical = float(x_dir)
        joy_msg.l_horizontal = float(y_dir)
        # 将转弯角速度折算回 -1.0 到 1.0 范围的手柄 r_horizontal 数值
        # 因为我们有公式：angular.z = r_horizontal * 1.57，因此反向折算为：
        joy_msg.r_horizontal = (yaw_dir * turn) / 1.57
        
        # 2. 构造 /cmd_vel 控制信号
        twist_msg = Twist()
        twist_msg.linear.x = x_dir * speed
        twist_msg.linear.y = y_dir * speed
        twist_msg.linear.z = 1.0  # 关键：策略使能位
        twist_msg.angular.z = yaw_dir * turn
        
        # 3. 双通道发布
        joy_pub.publish(joy_msg)
        cmd_pub.publish(twist_msg)
        rate.sleep()

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    print("正在关闭冲突的 joy_teleop 节点...")
    subprocess.run(['rosnode', 'kill', '/joy_teleop'], capture_output=True)
    time.sleep(1.0)
    
    rospy.init_node('keyboard_teleop_custom')
    joy_pub = rospy.Publisher('/joy_msg', Joy, queue_size=5)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    
    # 启动 50Hz 独立发布线程
    t = threading.Thread(target=publish_loop, args=(joy_pub, cmd_pub))
    t.daemon = True
    t.start()
    
    try:
        print(msg)
        print(f"当前速度配置: 线速度 {speed:.2f} m/s | 角速度 {turn:.2f} rad/s")
        
        while not rospy.is_shutdown():
            key = getKey()
            if key == '':
                continue
                
            if key in moveBindings.keys():
                x_dir, y_dir, yaw_dir = moveBindings[key]
            elif key in speedBindings.keys():
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                print(f"当前速度配置: 线速度 {speed:.2f} m/s | 角速度 {turn:.2f} rad/s")
            elif key == '\x03':  # Ctrl+C
                break
                
    except Exception as e:
        print(e)
        
    finally:
        is_running = False
        time.sleep(0.2)
        
        # 恢复 terminal 配置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        # 恢复机器人手柄节点
        print("\n正在恢复默认的 joy_teleop 节点...")
        try:
            from common.sim2real_env import joy_teleop_restore_cmd
            restore_cmd = joy_teleop_restore_cmd()
        except Exception:
            restore_cmd = (
                "source /home/nvidia/sim2real/install/setup.bash && "
                "roslaunch sim2real_master joy_teleop.launch use_filter:=true &"
            )
        subprocess.Popen(
            ['bash', '-c', restore_cmd],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        print("已退出键盘控制模式。")
