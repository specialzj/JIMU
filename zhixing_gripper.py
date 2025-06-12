import minimalmodbus
import serial
import threading
import time
# 寄存器地址
POSITION_HIGH_8 = 0x0102  # 位置寄存器高八位
POSITION_LOW_8 = 0x0103  # 位置寄存器低八位
SPEED = 0x0104
FORCE = 0x0105
MOTION_TRIGGER = 0x0108

PORT = '/dev/ttyUSB0'  # 修改为您的COM口号
BAUD = 115200

left_instrument = minimalmodbus.Instrument(PORT, 1)
left_instrument.serial.baudrate = BAUD
left_instrument.serial.timeout = 1

right_instrument = minimalmodbus.Instrument(PORT, 2)
right_instrument.serial.baudrate = BAUD
right_instrument.serial.timeout = 1

lock = threading.Lock()

# # 写入高八位位置
# def write_position_high8(instrument, value):
#     with lock:
#         instrument.write_register(POSITION_HIGH_8, value, functioncode=6)

# # 写入低八位位置
# def write_position_low8(instrument, value):
#     with lock:
#         instrument.write_register(POSITION_LOW_8, value, functioncode=6)

# 写入位置
def write_position(instrument, value):
    with lock:
        instrument.write_long(POSITION_HIGH_8, value)

# 写入速度
def write_speed(instrument, speed):
    with lock:
        instrument.write_register(SPEED, speed, functioncode=6)

# 写输入
def write_force(instrument, force):
    with lock:
        instrument.write_register(FORCE, force, functioncode=6)

# 触发运动
def trigger_motion(instrument):
    with lock:
        instrument.write_register(MOTION_TRIGGER, 1, functioncode=6)

if __name__ == '__main__':
    # 写入高八位位置
    # write_position_high8(left_instrument, 0x00)
    # write_position_high8(right_instrument, 0x00)

    # # 写入低八位位置
    # write_position_low8(left_instrument, 0x640)
    # write_position_low8(right_instrument, 0x640)
    i=1
    while True:
        # for i in range(10):  # 循环10次，可根据需要修改
        write_position(left_instrument, 8000)
        trigger_motion(left_instrument)
        time.sleep(0.5)
        write_position(right_instrument, 8000)
        trigger_motion(right_instrument)
        time.sleep(1)

        write_position(left_instrument, 5000)
        trigger_motion(left_instrument)
        time.sleep(0.5)
        write_position(right_instrument, 5000)
        trigger_motion(right_instrument)

        time.sleep(1)
        i+=1
        print(i)

    # # 写入速度
    # write_speed(instrument, 80)

    # # 写输入
    # write_force(instrument, 50)
    # from IPython import embed; embed()

