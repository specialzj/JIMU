# pip install pyserial
import serial 
import time

import subprocess

def set_device_permissions():
    # 替换为您的 sudo 密码
    sudo_password = "123456"
    try:
        # 使用 echo 将密码传递给 sudo
        command = "sudo -S chmod 666 /dev/ttyACM1"
        process = subprocess.run(
            command,
            input=sudo_password + "\n",  # 将密码作为输入传递
            text=True,
            shell=True,
            check=True
        )
        command = "sudo -S chmod 666 /dev/ttyACM0"
        process = subprocess.run(
            command,
            input=sudo_password + "\n",  # 将密码作为输入传递
            text=True,
            shell=True,
            check=True
        )
        print("Permissions updated successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to update permissions: {e}")


def calculate_bcc(data):  
    """  
    计算BCC校验和  
    """  
    bcc = 0  
    for byte in data:  
        bcc ^= byte  
    return bcc  

def build_command(control_id, control_mode, direction, subdivision, angle, speed):  
    """  
    构建协议数据帧  
    """  
    # 帧头和帧尾  
    frame_head = 0x7B  
    frame_tail = 0x7D

    # 将角度和速度放大10倍  
    angle *= 10  
    speed *= 10  

    # 角度和速度的高八位和低八位  
    angle_high = (angle >> 8) & 0xFF  
    angle_low = angle & 0xFF  
    speed_high = (speed >> 8) & 0xFF  
    speed_low = speed & 0xFF  

    # 构建数据帧（不包括BCC和帧尾）  
    data = [  
        frame_head,  
        control_id,  
        control_mode,  
        direction,  
        subdivision,  
        angle_high,  
        angle_low,  
        speed_high,  
        speed_low  
    ]  

    # 计算BCC校验位  
    bcc = calculate_bcc(data)  

    # 添加BCC和帧尾  
    data.append(bcc)  
    data.append(frame_tail)  

    return bytearray(data)  

def usb_send(port, baudrate, control_id, control_mode, direction, subdivision, angle, speed):  
    """  
    通过串口发送协议数据帧  
    """  
    # if direction == 0x00 :
    #     print("open gripper",port, baudrate, control_id, control_mode, direction, subdivision, angle, speed)  
    # if direction == 0x01 :
    #     print("close gripper",port, baudrate, control_id, control_mode, direction, subdivision, angle, speed)  
    # 打开串口  
    ser = serial.Serial(port, baudrate, timeout=0.5)  
    # time.sleep(0.2)
    # 构建数据帧  
    command = build_command(control_id, control_mode, direction, subdivision, angle, speed)
    # time.sleep(0.2)
    # 发送数据帧
    ser.write(command)



if __name__ == "__main__":
    # from USB_control import set_device_permissions
    # set_device_permissions()  # 夹具权限  
    # 串口配置  
    port = "/dev/ttyLeftGripper"  # 修改为实际的串口号  
    # port = "/dev/ttyRightGripper"  # 修改为实际的串口号  
    # port = "/dev/ttyACM0"
    baudrate = 115200
    
    # 控制参数  ss
    control_id = int("0x01", 16)  
    control_mode = int("0x02", 16)   
    direction = int("0x01", 16)       # 1合 0开
    subdivision = int("0x20", 16)
    angle = 800# 1872°
    speed = 10  # 20 Rad/s332
    usb_send(port, baudrate, control_id, control_mode, direction, subdivision, angle, speed)
    # while True:
    #     direction = int("0x00", 16)       # 1合 0开
    #     angle = 500 # 1872°
    #     usb_send(port, baudrate, control_id, control_mode, direction, subdivision, angle, speed)
    #     time.sleep(1)
    #     direction = int("0x00", 16)       # 1合 0开
    #     angle = 500 # 1872°
    #     usb_send(port, baudrate, control_id, control_mode, direction, subdivision, angle, speed)
    #     time.sleep(1)
    
