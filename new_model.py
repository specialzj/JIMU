import time
import numpy as np
from APP import App  
import logging  
from Initialize import Initialize  
from threading import Thread, Event  
import threading
from copy import deepcopy
import numpy as np
from math import cos, sin, radians
from Function0603 import Function 
from rich.logging import RichHandler
import asyncio  
import nest_asyncio
# 添加文件操作相关模块
import os
import shutil
from datetime import datetime

def save_latest_files(source_dir, target_dir):
    """
    保存指定源目录下最新的子文件夹到目标目录

    参数:
        source_dir: 源文件目录
        target_dir: 目标保存目录
    """
    try:
        # 确保目标目录存在
        if not os.path.exists(target_dir):
            os.makedirs(target_dir)

        # 获取源目录下所有子目录
        subdirs = []
        for item in os.listdir(source_dir):
            item_path = os.path.join(source_dir, item)
            if os.path.isdir(item_path):
                subdirs.append((item_path, os.path.getmtime(item_path)))

        # 如果没有找到子目录，记录日志并返回
        if not subdirs:
            logging.warning(f"未在 {source_dir} 找到任何子目录")
            print(f"未在 {source_dir} 找到任何子目录")
            return

        # 按修改时间排序，获取最新的子目录
        subdirs.sort(key=lambda x: x[1], reverse=True)
        latest_dir = subdirs[0][0]
        latest_dir_name = os.path.basename(latest_dir)

        # 创建带时间戳的目标文件夹
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        error_save_dir = os.path.join(target_dir, f"error_backup_{timestamp}")
        os.makedirs(error_save_dir, exist_ok=True)

        # 复制最新的子目录到目标位置
        target_subdir = os.path.join(error_save_dir, latest_dir_name)
        shutil.copytree(latest_dir, target_subdir)

        logging.info(f"已保存最新子目录 {latest_dir} 到 {target_subdir}")
        print(f"已保存最新子目录到: {target_subdir}")

    except Exception as e:
        logging.error(f"保存文件时出错: {str(e)}")
        print(f"保存文件时出错: {str(e)}")
class TASK():  
    
    def __init__(self, init):  
        self.init  = init
    def perform_sequence(self):
        global ttt

        def execute_task(command, work_name=None, repeat=1):
            """
            执行任务的通用逻辑
            参数:
                command: 发送给 app 的指令
                work_name: 任务名称（可选）
                repeat: 重复执行的次数（默认为 1）
            """
            global ttt
            for i in range(repeat):
                if ttt == 0 or ttt==3:
                    pose_tuple, pose_param_list = app.send_recv(command)
                else:
                    pose_tuple, pose_param_list = app.orecv(command)
                # from IPython import embed; embed()             
                #  检查姿态角是否在±15度范围内，错误则记录日志并退出
                for arm_name in pose_tuple:
                    for pose in pose_tuple[arm_name]:
                        # 检查姿态角
                        if not -15 <= pose[3] <= 15 or not -15 <= pose[4] <= 15:
                            error_msg = f"姿态角超出范围！机械臂: {arm_name}, 当前值: {pose[3]}, {pose[4]}，允许范围: ±15度"
                            print(error_msg)
                            logging.error(error_msg)  # 添加错误日志记录
                            # 保存最新文件到指定目录
                            source_dir = "/home/dexforce/project/ww/data/PickLight"
                            target_dir = "/home/dexforce/文档/问题数据0521"
                            save_latest_files(source_dir, target_dir)
                            exit()

                        # 检查高程（Z轴坐标）
                        max_height = 500  # 假设最大高程为500mm，请根据实际情况调整
                        if pose[2] > max_height:
                            error_msg = f"高程超出安全范围！机械臂: {arm_name}, 当前高程: {pose[2]}mm，最大允许高程: {max_height}mm"
                            print(error_msg)
                            logging.error(error_msg)  # 添加错误日志记录
                            # 保存最新文件到指定目录
                            source_dir = "/home/dexforce/project/ww/data/PickLight"
                            target_dir = "/home/dexforce/文档/问题数据0521"
                            save_latest_files(source_dir, target_dir)
                            exit()
                use_arm = function.Use_arm(pose_tuple, self.init["ROBOT1"]["move_limit"], 
                                           self.init["ROBOT2"]["move_limit"], 0)  # 选择使用哪个手臂
                if work_name in ["2-右臂组装三球积木","3-右臂组装单球积木","1-右臂拆卸双球积木"]:
                    use_arm = "right_arm"
                elif work_name in ["3-左臂拆卸单球积木","2-左臂拆卸三球积木"]:
                    use_arm = "left_arm"

                asyncio.run(function.Move_posetuning_list(pose_tuple, work_name, index=ttt, arm=use_arm))
                ttt+=1


        while True:
            if aaa == 0:
                ttt=0
              
                task = [
                    ("d,0,0,0,0,0,0,2", "1-组装双球积木"),
                    ("d,0,0,0,0,0,0,2", "2-右臂组装三球积木"),
                    ("d,0,0,0,0,0,0,2", "3-右臂组装单球积木"),
                    ("d,0,0,0,0,0,0,3", "3-左臂拆卸单球积木"),
                    ("d,0,0,0,0,0,0,3", "2-左臂拆卸三球积木"),
                    ("d,0,0,0,0,0,0,3", "1-右臂拆卸双球积木"),
                ]
                for command, work_name in task:
                    execute_task(command, work_name)

            elif aaa == 1:
                ttt=3
                task = [
                    ("d,0,0,0,0,0,0,3", "3-左臂拆卸单球积木"),
                    ("d,0,0,0,0,0,0,3", "2-左臂拆卸三球积木"),
                    ("d,0,0,0,0,0,0,3", "1-右臂拆卸双球积木"),
                ]
                for command, work_name in task:
                    execute_task(command, work_name)
                ttt=0
            
                tasks2 = [
                    ("d,0,0,0,0,0,0,2", "1-组装双球积木"),
                    ("d,0,0,0,0,0,0,2", "2-右臂组装三球积木"),
                    ("d,0,0,0,0,0,0,2", "3-右臂组装单球积木"),]
                for command, work_name in tasks2:
                    execute_task(command, work_name)


if __name__ == '__main__':
    #———————————————————————————配置加载初始化———————————————————————————————#
    nest_asyncio.apply()  # 允许嵌套事件循环
    np.set_printoptions(7, suppress=True)
    
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(filename)s - Line: %(lineno)d - %(message)s')
    config_file_path = 'agile_config.json'
    init, log_msg = Initialize.load_config(config_file_path)
    global running # 标定部分
    running = True
    use_robot = "AGILE" # 选择使用的机器人 W1 或 AGILE
    task = TASK(init)
    aaa = 0 # 0为装，1为拆
    
    #———————————————————————————视觉初始化——————————————————————————————#
    app = App(init, log_msg)
    app.connection()
   
    #———————————————————————————AGILE机器人初始化————————————————————————————#
    from Function import AGILE_Instruction
    from USB_control import set_device_permissions
    set_device_permissions()  # 夹具权限


    agile = AGILE_Instruction(init,log_msg)
    agile.initialize()
    
    # agile.get_current_pose("ROBOT1")
    # agile.get_current_pose("ROBOT2")
    asyncio.run(agile.GoHome())
    # 获取当前位置
    # from IPython import embed; embed()
    #———————————————————————————功能函数初始化————————————————————————————#
    function = Function(init, log_msg,use_robot,app)
    #———————————————————————————进入线程作业——————————————————————————————#
    thread2 = Thread(target=task.perform_sequence)
    thread2.start()
    thread2.join()



