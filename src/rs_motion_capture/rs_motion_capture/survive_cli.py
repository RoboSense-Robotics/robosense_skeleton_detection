#!/usr/bin/python3
import subprocess
import signal
import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Timer


def run_command_with_timeout(command, timeout_sec=5):
    """
    执行命令，等待指定时间后终止进程，并返回输出及状态
    :param command: 要执行的命令（列表形式，如 ["my_tool", "-arg1"]）
    :param timeout_sec: 超时时间（秒）
    :return: (输出内容, 是否成功)
    """
    # 启动子进程，捕获输出
    proc = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,  # 合并标准错误到标准输出
        text=True,                 # 以文本形式返回（Python 3.7+）
        encoding='utf-8',          # 设置编码
        errors='replace'           # 编码错误处理
    )

    # 创建定时器：超时后发送终止信号
    def timeout_handler():
        try:
            if sys.platform == 'win32':
                # Windows 使用强制终止
                proc.kill()
            else:
                # Unix/Linux 发送 SIGINT (模拟 Ctrl+C)
                os.kill(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass  # 进程已结束则忽略

    timer = Timer(timeout_sec, timeout_handler)
    timer.start()

    # 等待进程结束并捕获输出
    try:
        stdout, _ = proc.communicate()  # 等待进程结束
    finally:
        timer.cancel()  # 确保取消定时器

    # 检查输出是否包含成功标识（根据实际需求修改）
    success = check_success(stdout)

    return stdout, success


def check_success(output):
    """
    根据输出判断是否成功（示例逻辑）
    实际使用需替换为您的判断逻辑
    """
    # 示例1: 检查是否输出"success"关键字
    # return "success" in output.lower()

    # 示例2: 检查无错误关键词（根据实际情况调整）
    error_failures = []
    for info in output.split('\n'):
        print("====", info)
        if "error failures" in info:
            terms = info.split(' ')
            error_failures .append(int(terms[-1].strip()))

    if len(error_failures) == 0:
        return False
    if sum(error_failures) != 0:
        return False

    # 示例3: 如果有预期输出则成功（根据工具特点调整）
    return True


if __name__ == "__main__":
    rclpy.init()
    node = Node("infrared_ws_calib")
    hci_pub = node.create_publisher(
        String, "/robot_control/hci_status", 10)
    mqtt_sub = node.create_publisher(
        String, "/mqtt/calib_progress", 10)
    msg = String()
    msg.data = "infrared_ws_calib"
    hci_pub.publish(msg)
    tool_command = ["survive-cli",
                    "--force-calibrate"]  # 替换为实际命令

    os.makedirs(os.path.join(os.path.expanduser("~"),
                ".config", "libsurvive"), exist_ok=True)

    output, success = run_command_with_timeout(tool_command, timeout_sec=10)
    while not success:
        output, success = run_command_with_timeout(tool_command, timeout_sec=10)
        print("Command Output:")
        print("-" * 40)
        print(output)
        print("-" * 40)
        print(f"Result: {'SUCCESS' if success else 'FAILURE'}")
    finish_msg = String()
    finish_msg.data = "calib_finish"
    print("publish finished hci message.")
    hci_pub.publish(finish_msg)
    mqtt = String()
    mqtt.data = "infrared_ws_calib:100"
    rclpy.shutdown()
