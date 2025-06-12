import numpy as np  
from scipy.spatial.transform import Rotation  
import logging  
import time
from math import cos, sin, radians
from Initialize import Initialize  
import asyncio  
from rich.logging import RichHandler
from unittest.mock import MagicMock
import math
# import nest_asyncio
# nest_asyncio.apply()  # 允许嵌套事件循环
# from embodychain.deploy.w1.ros2_controller.w1_controller import W1Controller, EndEffectorType

from copy import deepcopy
class W1_Instruction:

    def __init__(self,log_msg):  
        from embodychain.deploy.w1.ros2_controller.w1_controller import W1Controller, EndEffectorType
        self.W1Controller = W1Controller
        self.EndEffectorType = EndEffectorType
        self.w1_controller = self.W1Controller(end_effector_type=self.EndEffectorType.PGC_GRIPPER)
        self.log_msg = log_msg
        self.loop = None 

    def initialize(self):
        # 设置初始位置, 后面计算出来的关节, 会插入这个初始位置里面
        self.w1_controller.real_w1_init_qpos = np.array([0.85001,-1.7,1.19888,-0.00001,-0.00005,-0.79998,-0.56319,-1.01778,0.00007,-1.38675,1.08819,-0.14756,-1.6996,0.56319,1.01777,-0.00001,1.38665,-1.08819,0.1475,1.69957,])
        self.w1_controller.real_current_qpos = deepcopy(self.w1_controller.real_w1_init_qpos)
        tcp_xpos = np.eye(4)
        tcp_xpos[2, 3] = 0.145
        self.w1_controller.set_tcp_xpos(name="left_arm", tcp_xpos=tcp_xpos)
        self.w1_controller.set_tcp_xpos(name="right_arm", tcp_xpos=tcp_xpos)

    async def async_MoveJ(self, pose, name):
        if self.loop is None:
            self.loop = asyncio.get_running_loop()
        success = await self.loop.run_in_executor(
            None, 
            lambda: self.w1_controller.set_current_qpos(
                name=name,
                qpos=pose
            )
        )
        return success

    async def async_MoveL(self, pose_list, name):
        """
        说明:
            pose_tuple: {"left_arm":robot1pose,"right_arm":robot2pose} 机器人坐标系坐标
        """
        left_pose = (pose_tuple["left_arm"]) if "left_arm" in pose_tuple else self.get_current_pose('l')[0]
        right_pose = (pose_tuple["right_arm"]) if "right_arm" in pose_tuple else self.get_current_pose('r')[0]


        if self.loop is None:
            self.loop = asyncio.get_running_loop()
        success = await self.loop.run_in_executor(
            None, 
            lambda: self.w1_controller.move_linear(
                position=left_pose[:3],
                orientation=left_pose[3:],
                name=name,
                rotation_sequence="ZYX"
            )
        )
        return success

    async def async_MoveJ1(self, pose_tuple):
        """
        说明:
            pose_tuple: {"left_arm":robot1pose,"right_arm":robot2pose} 机器人坐标系坐标
        """  
        if len(pose_tuple)!=2:
            exit()

        if self.loop is None:
            self.loop = asyncio.get_running_loop()
        success = await self.loop.run_in_executor(
            None, 
            lambda: self.w1_controller.move_joint_dual(
            left_joint_positions=pose_tuple["left_arm"],
            right_joint_positions= pose_tuple["right_arm"],
            ratio=0.5,
            )
        )
        return success

    async def async_MoveL1(self, pose_tuple):
        """
        说明:
            输入pose_tuple若只含一个运动坐标,自动生成另一个臂当前坐标
            pose_tuple: {"left_arm":robot1pose,"right_arm":robot2pose} 机器人坐标系坐标
        """
        left_pose = (pose_tuple["left_arm"]) if "left_arm" in pose_tuple else self.get_current_pose('l')[0]
        right_pose = (pose_tuple["right_arm"]) if "right_arm" in pose_tuple else self.get_current_pose('r')[0]
        # from IPython import embed; embed()
        if self.loop is None:
            self.loop = asyncio.get_running_loop()
        success = await self.loop.run_in_executor(
            None, 
            lambda: self.w1_controller.move_linear_dual(
            left_pose_list=left_pose,
            right_pose_list=right_pose,
            threshold=0.005,  # 直线插值间距为2mm
            ratio=5       # 5倍速度
            )
        )
        return success

    def gripper(self,name,status):
        """
        夹具开合状态
        
        参数:
            name: r / l
            status: float(0-1) 开合状态
        """
        if name == "r": ee_name = 'right_gripper' 
        elif name == "l": ee_name = 'left_gripper'
        self.w1_controller.set_gripper_state(status, ee_name)
        self.w1_controller.set_gripper_state(status, ee_name)

    def get_move_pose(self,pose_tuple):
        """
        说明:
            pose_tuple: {"left_arm":robot1pose,"right_arm":robot2pose} 机器人坐标系坐标
        """
        left_pose = (pose_tuple["left_arm"]) if "left_arm" in pose_tuple else self.get_current_pose('l')[0]
        right_pose = (pose_tuple["right_arm"]) if "right_arm" in pose_tuple else self.get_current_pose('r')[0]

        left_fk_pose_list, right_fk_pose_list = self.w1_controller.get_linear_trajectory(
            left_pose_list=left_pose,
            right_pose_list=right_pose,
            qpos_seed=self.w1_controller.real_current_qpos,
            rotation_sequence="ZYX",
            threshold=0.01,
            ratio=0.5,
            sample_num=10,
        )
        return left_fk_pose_list, right_fk_pose_list 

    def GoHome(self):
        _ = self.w1_controller.go_home()
        self.w1_controller.set_gripper_state(1.0, ee_name="right_gripper")
        self.w1_controller.set_gripper_state(1.0, ee_name="left_gripper")
        time.sleep(1.0)
    
    def calibration(self,robot_arm,sign,use_movepse):  
        self.w1_controller.real_w1_init_qpos = self.w1_controller.zero_err_node.get_qpos()
        self.w1_controller.real_current_qpos = deepcopy(self.w1_controller.real_w1_init_qpos)
        global running
        if log_msg["wait_calib_sign"][1]==1: logging.info(log_msg["wait_calib_sign"][0].format(sign)) 
        app_msg=App.orecv() # 接受视觉返回信息
        if robot_arm == "right_arm":
            calib_pose=[ # 10个坐标俯视 末端旋转90度
            [0.0874009522347654, 0.5565474046047592, 0.028642297523802096, 1.6167917819816555, -0.41109486692844666, 0.8449717454019758, 0.16631707323653844],
            [0.0874009522347654, 0.5565593888296645, 0.028642297523802096, 1.6647286816030817, -0.5189409068517503, 0.894143020188654, 0.010510165241997882],
            [0.13534983608109696, 0.688385862788587, -0.18706176654771056, 1.6647286816030817, -0.521349736057727, 0.8941190517388433, 0.010534133691808556],
            [0.43494347449010595, 1.011923982558498, -0.30690401560127656, 1.688685147188889, -0.7370538001292397, 0.6484184727292228, 0.2381984542188671],
            [0.786069279992148, 0.810624956823224, -0.7143796466083052, 1.664716697378176, -0.5453181858684402, 0.5645408826166318, 0.06086787829430618],
            [0.8579985978740985, 0.5949568454264269, -0.9899928509816958, 1.3411665933833596, -0.6040528721295928, 0.6244739913683204, -0.44246956773067003],
            [0.7621008301814349, 0.6069051176570674, -0.8581783612476785, 1.1614032198030104, -0.6519777875261137, 0.8042373649486687, -0.44248155195557537],
            [0.7621128144063403, 0.6068691649823514, -1.0367433123374918, 1.2811975319569546, -0.8077607270708436, 0.5765370917468942, -0.6102726848554725],
            [0.7621008301814349, 0.6069051176570674, -1.0367552965623972, 1.1913398136165911, -0.5788620313785331, 0.5765490759717995, -0.6102726848554725],
            [0.7621128144063403, 0.6069171018819728, -0.8809723570176669, 1.191351797841497, -0.7106885053374552, 0.8521742645700949, -0.5863162192696647],
            [0.6422705653527747, 0.810624956823224, -0.8809603727927615, 1.251260938143374, -0.998345855740729, 0.6364462320487716, -0.5863042350447594],
            [0.6422705653527747, 0.8106009883734133, -0.8809483885678562, 1.251260938143374, -0.998309903066013, 0.6364582162736769, -0.586292250819854]
            ]  
        elif robot_arm == "left_arm":
            calib_pose=[ # 10个坐标俯视 末端旋转90度
            [-0.7872796867075893, -1.0178202212119336, 0.740685020275563, -1.0751767216089703, -0.13421133471508817, 0.8830815806010097, -2.0591415074632717],
            [-0.5955560566716946, -0.9218984850694598, 0.740685020275563, -1.0751527531591596, -0.1342233189399935, 0.8830815806010097, -2.0591534916881766],
            [-0.823232361423659, -0.9615183326065684, 0.8365468352935101, -0.9912871472714744, -0.11024288490437506, 0.8830935648259146, -2.0591415074632717],
            [-0.8232203771987536, -0.9615183326065684, 0.8724995100095798, -1.135097846135753, 0.00961134837409583, 1.038876504370645, -2.346774889416735],
            [-1.0509206504005286, -0.9734905732870196, 0.9923417590631454, -1.18305871420699, 0.03357979818480894, 0.9914189737454331, -2.346774889416735],
            [-1.0509566030752446, -0.9734306521624929, 0.9923417590631454, -1.182986808857558, 0.0335678139599036, 0.9789553798438622, -2.346774889416735],
            [-1.038972378169888, -0.7097777042446483, 0.9923537432880511, -1.2908568172306725, 0.03357979818480894, 0.9789433956189573, -2.3467509209669237],
            [-0.9910234943235565, -0.7936792628070495, 0.884507703364747, -1.2908688014555778, -0.07430219441321073, 0.9190462395419852, -2.4426486886595873],
            [-0.8591850361397286, -0.7937152154817659, 1.0043379681934077, -1.2549281109644135, -0.2420693588632976, 1.0987976288974277, -2.2748575557596897],
            [-1.074901084436147, -0.7217379607001941, 0.9689485520478893, -1.2549161267395081, -0.24205737463839183, 1.0988096131223335, -2.274869539984595]
            ]    
        if app_msg==sign:
            if use_movepse==1: # 使用自动走点功能
                def check_exit():
                    global running
                    input("按 Enter 键停止...\n")  # 阻塞，直到用户按回车
                    running = False  # 修改标志，退出循环
                threading.Thread(target=check_exit, daemon=True).start() # 启动线程监听回车键

                for i in range(len(calib_pose)):
                    success = w1_controller.set_current_qpos(
                        name=robot_arm,
                        qpos=calib_pose[i]
                    )
                    time.sleep(0.5)
                    while running:
                        print("回车键跳出: ", running)
                        W1_deg_pose = w1_controller.get_current_pose(name=robot_arm)# 获取臂当前位姿矩阵
                        W1_joint_pose = w1_controller.get_current_qpos(name=robot_arm)
                        if log_msg["robot_joint_pose"][1]==1: logging.info(log_msg["robot_joint_pose"][0].format(robot_arm,W1_joint_pose.tolist())) 
                        send_msg = "p," + ",".join([f"{num:.3f}" for num in W1_deg_pose.tolist()])
                        if log_msg["send_msg"][1]==1: logging.info(log_msg["send_msg"][0].format(robot_arm,send_msg)) 
                        App.send(send_msg) 
                        time.sleep(0.5)
                    if not running:
                        print("回车键跳出: ", running)
                        running = True
                        threading.Thread(target=check_exit, daemon=True).start() # 启动线程监听回车键
                        continue
                        
            elif use_movepse==0: # 使用自动走点功能
                while True:
                    W1_deg_pose = w1_controller.get_current_pose(name=robot_arm)# 获取臂当前位姿矩阵
                    W1_joint_pose = w1_controller.get_current_qpos(name=robot_arm)
                    if log_msg["robot_cart_pose"][1]==1: logging.info(log_msg["robot_cart_pose"][0].format(robot_arm,W1_deg_pose.tolist())) 
                    if log_msg["robot_joint_pose"][1]==1: logging.info(log_msg["robot_joint_pose"][0].format(robot_arm,W1_joint_pose.tolist())) 
                    send_msg = "p," + ",".join([f"{num:.3f}" for num in W1_deg_pose.tolist()])
                    if log_msg["send_msg"][1]==1: logging.info(log_msg["send_msg"][0].format(robot_arm,send_msg)) 
                    # from IPython import embed; embed()

                    App.send(send_msg) 
            else:
                print("use_movepse要等于0或1")
        else:
            if log_msg["calibration_error"][1]==1: logging.info(log_msg["calibration_error"][0].format(app_msg,sign)) 

    def get_current_pose(self,robot):
        """
        获取机械臂当前坐标
        
        参数:
            robot: r / l
            
        返回:
            W1_deg_pose,W1_joint_pose
        """
        if robot == "r": name = 'right_arm' 
        elif robot == "l": name = 'left_arm'
        W1_deg_pose = self.w1_controller.get_current_pose(name)# 获取臂当前位姿矩阵
        W1_joint_pose = self.w1_controller.get_current_qpos(name)
        W1_deg_pose = [round(num, 3) for num in W1_deg_pose]
        W1_joint_pose = [round(num, 3) for num in W1_joint_pose]
        if self.log_msg["robot_cart_pose"][1]==1: logging.info(self.log_msg["robot_cart_pose"][0].format(name,W1_deg_pose)) 
        if self.log_msg["robot_joint_pose"][1]==1: logging.info(self.log_msg["robot_joint_pose"][0].format(name,W1_joint_pose)) 
        return W1_deg_pose,W1_joint_pose
  
class AGILE_Instruction:
    def __init__(self,init, log_msg):   
        import sys  
        sys.path.append("/home/dexforce/文档/AgileRobot/bin_2.16/AGILE_API")  
        sys.path.append("/home/dexforce/文档/AgileRobot/bin_2.16/AGILE_API1")  
        from USB_control import usb_send
        from AGILE_API import DianaApi  
        from AGILE_API1 import DianaApi as DianaApi1  
        self.usb_send = usb_send
        self.log_msg = log_msg
        self.DianaApi1 = DianaApi1
        self.DianaApi = DianaApi    
        self.init = init
        self.loop = None 
        self.lock = None

    # def gripper_init(self):
    #     """初始化夹爪通信"""
    #     import minimalmodbus
    #     import threading
    #     # 寄存器地址定义为类变量
    #     self.POSITION_HIGH_8 = 0x0102  # 位置寄存器高八位
    #     self.POSITION_LOW_8 = 0x0103  # 位置寄存器低八位
    #     self.SPEED = 0x0104
    #     self.FORCE = 0x0105 
    #     self.MOTION_TRIGGER = 0x0108
    #     PORT = '/dev/ttyUSB0'
    #     BAUD = 115200
    #     # 初始化两个夹爪实例
    #     self.left_instrument = minimalmodbus.Instrument(PORT, 1)
    #     self.left_instrument.serial.baudrate = BAUD
    #     self.left_instrument.serial.timeout = 1

    #     self.right_instrument = minimalmodbus.Instrument(PORT, 2)
    #     self.right_instrument.serial.baudrate = BAUD
    #     self.right_instrument.serial.timeout = 1

    #     self.lock = threading.Lock()

    # def write_position(self, instrument, value):
    #     """写入夹爪位置"""
    #     with self.lock:
    #         instrument.write_long(self.POSITION_HIGH_8, value)

    # def trigger_motion(self, instrument):
    #     """触发夹爪运动"""
    #     with self.lock:
    #         instrument.write_register(self.MOTION_TRIGGER, 1, functioncode=6)

    # def gripper(self, robot_name, status):
    #     """
    #     控制夹爪开合
        
    #     参数:
    #         robot_name: "ROBOT1"(左臂) 或 "ROBOT2"(右臂)
    #         status: 夹爪位置值
    #     """
    #     try:
    #         if robot_name == "ROBOT1":
    #             self.write_position(self.left_instrument, status)
    #             self.trigger_motion(self.left_instrument)
    #         elif robot_name == "ROBOT2":
    #             self.write_position(self.right_instrument, status)
    #             self.trigger_motion(self.right_instrument)
    #     except Exception as e:
    #         print(f"夹爪控制错误: {str(e)}")


    def usb_grap(self,status,angle,robot_name):
        u = self.init[robot_name]["USB"]
        self.usb_send(u["port"], u["baudrate"], int(u["control_id"], 16) ,int(u["control_mode"], 16), int("0x0"+str(status),16),int(u["subdivision"], 16), angle,u["speed"])

    def initialize(self):
        self.DianaApi1.initSrv((self.init["ROBOT1"]["IP"], 0, 0, 0, 0, 0))  
        self.DianaApi1.releaseBrake(self.init["ROBOT1"]["IP"])  
        self.DianaApi.initSrv((self.init["ROBOT2"]["IP"], 0, 0, 0, 0, 0))  
        self.DianaApi.releaseBrake(self.init["ROBOT2"]["IP"])

    async def GoHome(self):
        """
        说明:
            机械臂回原点
        参数:
            robot_name: "ROBOT1" or "ROBOT2"
        """
        async def left_gohome():
            self.usb_grap(1, 1700, "ROBOT1")  
            await self.async_MoveJ(self.init["Pose_List"]["basic_pose"]["l_base"], 'ROBOT1')
            time.sleep(1.5)   
            self.usb_grap(0, 750, "ROBOT1") 
            
        async def right_gohome():
            self.usb_grap(1, 1700, "ROBOT2")
            await self.async_MoveJ(self.init["Pose_List"]["basic_pose"]["r_base"], 'ROBOT2')    
            time.sleep(1.5)
            self.usb_grap(0, 750, "ROBOT2")
            
        await asyncio.gather(right_gohome(), left_gohome())


    def get_current_pose(self,robot_name):
        api = self.DianaApi1 if robot_name == "ROBOT1" else self.DianaApi  
        srcMatrixPose=[0] * 6 # 工件坐标系的值
        dstMatrixPose=[0] * 6
        dstPose = [0] * 6
        srcPose = [0] * 6
        api.getTcpPoseByWorkPieceName(self.init[robot_name]["WorkPieceName"], srcMatrixPose,self.init[robot_name]["IP"])
        poses = [0]*6
        api.getTcpPos(poses, self.init[robot_name]["IP"])
        # api.poseTransform(poses,srcMatrixPose,dstPose,dstMatrixPose) 
        self.print_robot_pose(poses,dstMatrixPose,srcMatrixPose,robot_name)

    def print_robot_pose(self,dstPose,dstMatrixPose,srcMatrixPose,robot_name):
        api = self.DianaApi1 if robot_name == "ROBOT1" else self.DianaApi    
        #查看发送坐标
        srcPose1 = [0]*6
        api.poseTransform(dstPose,dstMatrixPose,srcMatrixPose,srcPose1)
        c=srcPose1[3:]   
        api.axis2RPY(c)
        rx1 = math.degrees(c[0])
        ry1 = math.degrees(c[1])
        rz1 = math.degrees(c[2])
        pose1=[srcPose1[0]*1000,srcPose1[1]*1000,srcPose1[2]*1000,rx1,ry1,rz1]
        pose1 = [round(num, 2) for num in pose1]
        if self.log_msg["24"][1]==1: logging.debug(self.log_msg["24"][0].format(robot_name,pose1))



    def get_robot_pose(self,dstPose,dstMatrixPose,srcMatrixPose,robot_name):
        if robot_name == "left_arm": robot_name="ROBOT1"
        elif robot_name == "right_arm": robot_name="ROBOT2"
        api = self.DianaApi1 if robot_name == "ROBOT1" else self.DianaApi    
        #查看发送坐标
        srcPose1 = [0]*6
        api.poseTransform(dstPose,dstMatrixPose,srcMatrixPose,srcPose1)
        c=srcPose1[3:]   
        api.axis2RPY(c)
        rx1 = math.degrees(c[0])
        ry1 = math.degrees(c[1])
        rz1 = math.degrees(c[2])
        pose1=[srcPose1[0]*1000,srcPose1[1]*1000,srcPose1[2]*1000,rx1,ry1,rz1]
        pose1 = [round(num, 2) for num in pose1]
        if self.log_msg["robot_cart_pose"][1]==1: logging.debug(self.log_msg["robot_cart_pose"][0].format(robot_name,pose1))


    async def async_MoveJ(self, joint_pose, robot_name):
        """
        说明:
            机械臂关节运动
        参数:
            joint_pose: 机械臂关节坐标
            robot_name: 机械臂名称
        """
        if robot_name == "left_arm": robot_name="ROBOT1"
        elif robot_name == "right_arm": robot_name="ROBOT2"
        api = self.DianaApi1 if robot_name == "ROBOT1" else self.DianaApi
        vel = self.init[robot_name]["Global_agile"]["vel"]
        acc =  self.init[robot_name]["Global_agile"]["acc"]
        zv_shaper_order = self.init[robot_name]["Global_agile"]["zv_shaper_order"]
        zv_shaper_frequency = self.init[robot_name]["Global_agile"]["zv_shaper_frequency"]
        zv_shaper_damping_ratio = self.init[robot_name]["Global_agile"]["zv_shaper_damping_ratio"]
        ipAddress = self.init[robot_name]["IP"]
        if self.loop is None:
            self.loop = asyncio.get_running_loop()
        await self.loop.run_in_executor(
            None, 
            lambda: api.moveJToTarget(joint_pose, vel+0.2,acc+0.2, zv_shaper_order, zv_shaper_frequency,zv_shaper_damping_ratio, ipAddress)
        )
        api.wait_move()

    async def async_MoveL(self, card_pose, robot_name):
        """
        说明:
            机械臂笛卡尔直线运动
        参数:
            card_pose: 机械臂笛卡尔运动坐标
            robot_name: 机械臂名称
        """
        if robot_name == "left_arm": robot_name="ROBOT1"
        elif robot_name == "right_arm": robot_name="ROBOT2"
        api = self.DianaApi1 if robot_name == "ROBOT1" else self.DianaApi
        vel = self.init[robot_name]["Global_agile"]["vel"]
        acc =  self.init[robot_name]["Global_agile"]["acc"]
        zv_shaper_order = self.init[robot_name]["Global_agile"]["zv_shaper_order"]
        zv_shaper_frequency = self.init[robot_name]["Global_agile"]["zv_shaper_frequency"]
        zv_shaper_damping_ratio = self.init[robot_name]["Global_agile"]["zv_shaper_damping_ratio"]
        ipAddress = self.init[robot_name]["IP"]
        srcMatrixPose=[0] * 6 # 工件坐标系的值
        dstMatrixPose=[0] * 6
        dstPose = [0] * 6
        api.getTcpPoseByWorkPieceName(self.init[robot_name]["WorkPieceName"], srcMatrixPose,self.init[robot_name]["IP"])
        srcPose = self.convert_units((card_pose[0], card_pose[1], card_pose[2], card_pose[3], card_pose[4], card_pose[5]),robot_name)
        api.poseTransform(srcPose,srcMatrixPose,dstMatrixPose,dstPose)
        # self.get_robot_pose(dstPose,dstMatrixPose,srcMatrixPose,robot_name)
        # input("按回车键继续...")
        if self.loop is None:
            self.loop = asyncio.get_running_loop()
        await self.loop.run_in_executor(
            None, 
            lambda: api.moveLToPose(dstPose, vel,acc, zv_shaper_order,
                                    zv_shaper_frequency,zv_shaper_damping_ratio, 
                                    ipAddress)
        )
        api.wait_move()
    
    def convert_units(self,pose,robot_name):  
        if robot_name == "left_arm": robot_name="ROBOT1"
        elif robot_name == "right_arm": robot_name="ROBOT2"
        api = self.DianaApi1 if robot_name == "ROBOT1" else self.DianaApi   
        x, y, z, rx, ry, rz = pose
        x_meters = x / 1000.0  
        y_meters = y / 1000.0  
        z_meters = z / 1000.0  
        rx_radians = math.radians(rx)  
        ry_radians = math.radians(ry)  
        rz_radians = math.radians(rz)  
        srcPose = [x_meters, y_meters, z_meters, rx_radians, ry_radians, rz_radians]  
        a = srcPose[3:]
        api.rpy2Axis(a)
        srcPose[3:] = a
        return srcPose

class Function:

    def __init__(self, init, log_msg,robot,app):  
        self.init = init  
        self.log_msg = log_msg
        self.loop = None 
        # self.w1_instruction = W1_Instruction(log_msg)
        self.agile_instruction = AGILE_Instruction(init, log_msg)
        self.robot=robot
        self.app = app

    def grabpose_xyz_offset(self, pose, offset):
        """
        沿着抓取点自身坐标系的方向进行偏移
        
        参数:
            coord: 抓取点坐标和方向 [x, y, z, a, b, c] (单位: mm, 度)
            offset: 偏移量字典，如 {"x": 10, "y": 5, "z": 15} (单位: mm)
            euler_order: 欧拉角顺序，支持 "ZYX"（默认）或 "xyz"
            
        返回:
            偏移后的新坐标 [x', y', z', a, b, c]
        """
        x, y, z, a, b, c = pose
        
        # 如果RX和RY均为0，则直接处理Z偏移（无需旋转）
        if a == 0 and b == 0:
            new_pose = [
                x + offset.get("x", 0),
                y + offset.get("y", 0),
                z + offset.get("z", 0),
                a,
                b,
                c
            ]
            return new_pose
        
        # 否则计算旋转矩阵（xyz顺序：R = Rx(a) @ Ry(b) @ Rz(c)）
        rx = radians(a)
        ry = radians(b)
        rz = radians(c)
        
        Rx = np.array([
            [1, 0, 0],
            [0, cos(rx), -sin(rx)],
            [0, sin(rx), cos(rx)]
        ])
        
        Ry = np.array([
            [cos(ry), 0, sin(ry)],
            [0, 1, 0],
            [-sin(ry), 0, cos(ry)]
        ])
        
        Rz = np.array([
            [cos(rz), -sin(rz), 0],
            [sin(rz), cos(rz), 0],
            [0, 0, 1]
        ])
        
        R = np.dot(Rx, np.dot(Ry, Rz))
        
        # 局部偏移向量
        offset_vec = np.array([
            offset.get("x", 0),
            offset.get("y", 0),
            offset.get("z", 0)
        ])
        
        # 将局部偏移转换到世界坐标系
        world_offset = np.dot(R, offset_vec)
        
        # 返回新位姿（角度不变）
        new_pose = [
            x + world_offset[0],
            y + world_offset[1],
            z + world_offset[2],
            a,
            b,
            c
        ]
        
        return new_pose
        


    def move_fix_step(self,robot,step):
        """
        沿着基座标系的方向进行偏移
        
        参数:
            robot: r / l
            step: [dx,dy,dz] 单位m
        """
        if robot == "r": name = 'right_arm' 
        elif robot == "l": name = 'left_arm'
        self.w1_controller.translate_end_effector(name,deltaX=step[0], deltaY=step[1], deltaZ=step[2])
        self.w1_controller.translate_end_effector(name,deltaX=step[0], deltaY=step[1], deltaZ=step[2])

    def Use_arm(self,pose_tuple,left_arm_range,right_arm_range,axis):
        """
        说明:
            使用那个手臂抓取,x为左右移动

        参数:
            pose_tuple: {"left_arm":robot1pose,"right_arm":robot2pose} 机器人坐标系坐标
            max_left_arm : 左手axis轴范围
            max_right_arm : 右手axis轴范围
            axis: 轴名称,0,1,2分别对应x,y,z轴
        """
        if left_arm_range[0] < pose_tuple["left_arm"][0][axis] < left_arm_range[1]:
            use_arm = "left_arm"
        if right_arm_range[0] < pose_tuple["right_arm"][0][axis] < right_arm_range[1]:  
            use_arm = "right_arm"
        logging.info(self.log_msg["use_arm"][0].format(use_arm))

        return use_arm

    def pose_deal(self, pose_tuple, work_name, index):
        """
        优化后的抓取坐标处理函数：
        1. 统一所有坐标的第四第五位元素为0
        2. 根据不同任务和索引设置最优抓放rz角度
        """
        def adjust_angle(a, b, toy_type):
            """
            通用角度调整函数
            参数:
                a: 初始角度[-180,180]
                b: 初始b值
                toy_type: 积木类型决定调整模式
            返回:
                调整后的(a, b)
            """
            # 定义各模式参数
            mode_params = {
                "12": {
                    "a_adjustments": [0, 90, -90, 180, -180],
                    "b_values": [0, 90, -90, 180, -180],
                    "sync_rotation": False
                },
                "3": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [90, -90],
                    "sync_rotation": False # 不需要保持抓放旋转一致
                },
                "5": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [-180, 180, 0],
                    "sync_rotation": False
                },
                "4": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [90, -90],
                    "sync_rotation": True # 需要保持抓放旋转一致
                },
                "6": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [-120, 60],
                    "sync_rotation": True
                },
                "7": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [120,-60],
                    "sync_rotation": True
                },
                "8": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [-180, 180,90,-90, 0],
                    "sync_rotation": False
                }
            }


            params = mode_params.get(toy_type)
            print(toy_type)
            if not params:
                raise ValueError(f"无效的toy_type: {toy_type}")

            def normalize_angle(angle):
                """角度归一化到[-180,180]"""
                return (angle + 180) % 360 - 180

            if params["sync_rotation"]:
                # 同步旋转模式
                best_a, best_b = a, b
                min_distance = float('inf')

                for delta in params["a_adjustments"]:
                    new_a = normalize_angle(a + delta)
                    new_b = normalize_angle((b if b is not None else 0) + delta)
                    
                    if new_b not in params["b_values"]:
                        continue
                    
                    distance = abs(new_a)
                    if distance < min_distance or \
                    (distance == min_distance and abs(new_b - new_a) < abs(best_b - best_a)):
                        min_distance = distance
                        best_a, best_b = new_a, new_b
                
                return best_a, best_b
            else:
                # 非同步旋转模式
                adjusted_a = min(
                    [normalize_angle(a + delta) for delta in params["a_adjustments"]],
                    key=lambda x: abs(x)
                )
                adjusted_b = min(
                    params["b_values"],
                    key=lambda x: abs(x - adjusted_a)
                )
                return adjusted_a, adjusted_b


        def adjust_angle1(a, b):
            if abs(a - b) > 180:
                if a < b:
                    a = a - 180
                    b = b - 180
                else:
                    a = a + 180
                    b = b + 180
                # 将角度调整到 [-180, 180]
                a = round((a + 180) % 360 - 180,2)
                b = round((b + 180) % 360 - 180,2)
            return a, b
        
        # 1. 统一设置第四第五位元素为0
        for arm in pose_tuple:
            for pose in pose_tuple[arm]:
                pose[3] = pose[4] = 0

        pose_tuple['left_arm'][1][5] = round(pose_tuple['left_arm'][1][5])
        pose_tuple['right_arm'][1][5] = round(pose_tuple['right_arm'][1][5])


        # 2. 根据任务和索引设置最优角度
        # 定义任务索引与调整类型的映射
        index_mapping = {
            0: "12", 
            1: "12",
            2: "3",
            3: "4",
            4: "5",
            5: "6",
            6: "7",
            9: "8",
            11: "8"
        }
        
        if index in index_mapping:
            toy_type = index_mapping[index]
            for arm in pose_tuple:
                a, b = adjust_angle(
                    pose_tuple[arm][0][5],
                    pose_tuple[arm][1][5],
                    toy_type
                )
                a= round(a,2)
                b= round(b,2)
                if self.log_msg["pose_deal"][1]==1: logging.info(self.log_msg["pose_deal"][0].format(arm,a,b))#传入日志
                if abs(a-b) > 180 and a<b:
                    a,b = adjust_angle1(a,b)
                    if self.log_msg["pose_deal2"][1]==1: logging.info(self.log_msg["pose_deal2"][0].format(arm,a,b))
                pose_tuple[arm][0][5] = a
                pose_tuple[arm][1][5] = b
        # if a or b ==0:
        return pose_tuple



    async def Move_posetuning_list(self, pose_tuple_in, work_name, index, arm="all"):
        """
        说明:
            movel可以每次移动一个坐标
            根据输入pose,示教法移动多组坐标。
            机械臂选择,抓取点顺序,抓取点示教及调整均在config的Pose_List中完成。
            robot1pose = [[grabpose],[placepose]],注意可能无placepose,config中设置。
        参数:
            pose_tuple: {"left_arm":robot1pose,"right_arm":robot2pose} 机器人坐标系坐标
            work_name: 任务名
            arm: 臂选择，'all'代表使用config里的arm,left_arm或right_arm则是指定"work_name"固定使用该臂
        """
        # 深拷贝输入的 pose_tuple_in
        # from IPython import embed; embed()
        # 1. 统一设置第四第五位元素为0
        for arm_l in pose_tuple_in:
            for pose in pose_tuple_in[arm_l]:
                if not -25<pose[3]<25 and not -25<pose[4]<25 and index<7:
                    print("识别错误，退出程序")
                    exit()
                pose[3] = pose[4] = 0
        
        if not -25<pose_tuple_in['left_arm'][0][3]<25 and not -25<pose_tuple_in['left_arm'][0][4]<25 and index>6:
            print("识别错误，退出程序")
            exit()

        pose_tuple_new = deepcopy(pose_tuple_in)
        pose_tuple = self.pose_deal(pose_tuple_new, work_name, index)
        config_list = self.init["Pose_List"][work_name]
        euler = self.init["ROBOT1"]["Euler"]
        def calculate_new_pose(pose_cfg, robot_cfg, target_arm):
            """
            计算新的姿态坐标
            robot_cfg[0]选择第一个or第二个参数,第一个是抓取点坐标,第二个是放置点坐标
            pose_tuple_in[target_arm][robot_cfg[0]]记录下来的抓取点/放置点坐标，通常是辅助臂坐标系下的坐标
            pose_tuple_in软件给出的左右臂抓取点/放置点坐标
            config_list[pose_cfg][0]给出的示教点
            """
            if len(config_list[pose_cfg][0]) == 6:
                return self.posetuning_calculate(
                    pose_tuple_in[target_arm][robot_cfg[0]],
                    config_list["apose"][robot_cfg[0]],
                    config_list[pose_cfg][0],
                    euler
                )
            return pose_tuple[robot_cfg[4]][robot_cfg[0]] if target_arm == 'all' else pose_tuple[target_arm][robot_cfg[0]]

        def apply_offset(pose, offset, offset_type="base"):
            """应用偏移量"""
            if offset != [0]:
                pose = pose.copy()
                if offset_type == "base": # 基坐标系偏移：只在Z轴方向上加偏移
                    pose[2] += offset[2]
                elif offset_type == "grab": # 抓取坐标系偏移：在工具坐标系下偏移
                    pose = self.grabpose_xyz_offset(
                        pose, {"x": offset[0], "y": offset[1], "z": offset[2]}
                    )
            # 对结果进行精度处理
            return [round(num, 3 - len(str(int(num)))) if num != 0 else 0 for num in pose]

        async def execute_move(task_type, pose, arm, robot_cfg):
            """执行移动任务"""
            if self.log_msg[f"{task_type} success"][1] == 1:
                logging.debug(self.log_msg[f"{task_type} success"][0].format(robot_cfg[4] if arm == 'all' else arm, pose))#MoveL success or MoveJ success

            if task_type == "MoveL":
                if self.robot == "W1":
                    self.w1_instruction.get_move_pose({arm: pose})
                    return await self.w1_instruction.async_MoveL({arm: pose}, arm)
                elif self.robot == "AGILE":
                    return await self.agile_instruction.async_MoveL(pose, arm)
            elif task_type == "MoveJ":
                if self.robot == "W1":
                    return await self.w1_instruction.async_MoveJ(pose, arm)
                elif self.robot == "AGILE":
                    return await self.agile_instruction.async_MoveJ(pose, arm)

        async def process_pose(pose_cfg):
            """处理单个位姿的异步函数"""
            if "tpose" not in pose_cfg and "jpose" not in pose_cfg:
                return None
            #robot_cfg是机器人控制参数
            robot_cfg = config_list[pose_cfg][3]
            task_type = "MoveL" if "tpose" in pose_cfg else "MoveJ"

            # 确定目标手臂
            if robot_cfg[4] == "same":
                target_arm = arm
            elif robot_cfg[4] == "dif":
                target_arm = "right_arm" if arm == "left_arm" else "left_arm"
            else:
                target_arm = arm

            # 计算新位姿
            if task_type == "MoveL":
                new_pose = calculate_new_pose(pose_cfg, robot_cfg, target_arm)
                new_pose = apply_offset(new_pose, config_list[pose_cfg][1], offset_type="base")
                new_pose = apply_offset(new_pose, config_list[pose_cfg][2], offset_type="grab")
                if isinstance(config_list[pose_cfg][0][0], str):
                    pose_name = config_list[pose_cfg][0][0]
                    new_pose = self.init["Pose_List"]["basic_pose"].get(pose_name, config_list[pose_cfg][0])
                    new_pose = apply_offset(new_pose, config_list[pose_cfg][1], offset_type="base")
                    new_pose = apply_offset(new_pose, config_list[pose_cfg][2], offset_type="grab")
            else:  # MoveJ
                pose_name = config_list[pose_cfg][0][0]
                if robot_cfg[4] == "same" and not pose_name.startswith(("l_", "r_")):
                    pose_name = f"{'l_' if arm == 'left_arm' else 'r_'}{pose_name}"
                elif robot_cfg[4] == "dif" and not pose_name.startswith(("l_", "r_")):
                    pose_name = f"{'r_' if arm == 'left_arm' else 'l_'}{pose_name}"
                elif robot_cfg[4] == "all" and not pose_name.startswith(("l_", "r_")):
                    pose_name = f"{'l_' if arm == 'left_arm' else 'r_'}{pose_name}"
                new_pose = self.init["Pose_List"]["basic_pose"].get(pose_name, config_list[pose_cfg][0])

            return {
                'pose_cfg': pose_cfg,
                'task_type': task_type,
                'new_pose': new_pose,
                'target_arm': target_arm,
                'robot_cfg': robot_cfg
            }

        # 并行处理所有位姿计算
        tasks = [process_pose(pose_cfg) for pose_cfg in config_list]
        processed_poses = await asyncio.gather(*tasks)
        
        # 顺序执行运动和夹爪控制
        for result in processed_poses:
            if result is None:
                continue
                
            pose_cfg = result['pose_cfg']
            task_type = result['task_type']
            new_pose = result['new_pose']
            target_arm = result['target_arm']
            robot_cfg = result['robot_cfg']

            # 记录移动状态
            if self.log_msg["move_status"][1] == 1:
                logging.info(self.log_msg["move_status"][0].format(work_name, pose_cfg))

            # 执行移动
            success = await execute_move(task_type, new_pose, target_arm, robot_cfg)

            # 处理夹爪状态
            if self.robot == "W1":
                self.w1_instruction.gripper("l", robot_cfg[2])
            elif self.robot == "AGILE" and robot_cfg[1] != 0 and robot_cfg[3] != 0:#前后延时都不为0才进入
                robot_name = "ROBOT1" if target_arm == "left_arm" else "ROBOT2"
                #如果夹具控制是[1,700]形式，则700是夹具开合角度否则使用默认开合角度
                angle = robot_cfg[2][1] if isinstance(robot_cfg[2], list) else self.init[robot_name]["USB"]["angle"] - 360
                gripper_status = robot_cfg[2][0] if isinstance(robot_cfg[2], list) else robot_cfg[2]
                #前延时
                time.sleep(robot_cfg[1])
                self.agile_instruction.usb_grap(gripper_status, angle, robot_name)
                #后延时
                time.sleep(robot_cfg[3])

            # 发送命令到 app
            if "app" in pose_cfg:
                command = config_list[pose_cfg][4][0]
                self.app.osend(command)

    def posetuning_calculate(self, robot_pose, shotpose, grabpose, euler):  
        """
        计算示教坐标转换后的实际抓取位姿
        参数:
            robot_pose: 软件给出的抓取/放置点坐标
            shotpose: 记录的抓取/放置点坐标
            grabpose: 示教的抓取坐标
            euler: 欧拉角顺序
        """
        def poseConvert(pose:list, rotation:str):
            # 将位姿参数转换为4x4齐次变换矩阵
            rot_matrix = Rotation.from_euler(rotation, [pose[3],pose[4],pose[5]], degrees=True).as_matrix()
            pose_matrix = np.identity(4)
            pose_matrix[:3,:3] = rot_matrix
            pose_matrix[:3,3] = [pose[0],pose[1],pose[2]]
            return np.mat(pose_matrix)
        if self.log_msg["teach"][1]==1: logging.info(self.log_msg["teach"][0].format(robot_pose,shotpose,grabpose))
        if shotpose[5]=="n":
            return grabpose# 如果记录的抓取/放置点位姿的第6个参数为"n"，直接返回示教的抓取位姿
        else:
            app_matrix = poseConvert(shotpose, euler)  # 记录位置的变换矩阵
            teach_matrix = poseConvert(grabpose, euler)  # 示教抓取位置的变换矩阵
            matrix = np.linalg.inv(app_matrix) @ teach_matrix  # 计算相对变换矩阵
            CAM_BD = poseConvert(robot_pose, euler)  # 软件给出位置的变换矩阵
            PUT3 = CAM_BD @ matrix   # 计算目标位置的变换矩阵
            rxyz = Rotation.from_matrix(PUT3[:3, :3]).as_euler(euler, degrees=True)  
            rxyz = [round(num, 2) for num in rxyz]  
            x = round(PUT3[0, 3], 2)  
            y = round(PUT3[1, 3], 2)  
            z = round(PUT3[2, 3], 2)  
            return [x, y, z] + rxyz  
    