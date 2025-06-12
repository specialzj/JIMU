import logging  
import traceback
from socket import *
from scipy.spatial.transform import Rotation
import numpy as np  

class App:
    def __init__(self, init, log_msg):  
        self.init = init  
        self.log_msg = log_msg

    def connection(self):
        try:
            if self.log_msg["connect app"][1]==1: logging.info(self.log_msg["connect app"][0])
            self.app = socket(AF_INET, SOCK_STREAM)
            app_addr = (self.init["APP"]["IP"], self.init["APP"]["PORT"])
            self.app.connect(app_addr)
            if self.log_msg["connection success"][1]==1: logging.info(self.log_msg["connection success"][0])
        except Exception as e:
            if self.log_msg["connection failed"][1]==1: logging.error(self.log_msg["connection failed"][0])
            logging.error(traceback.format_exc())

    def osend(self, app_recv: str):
        if self.log_msg["app recv msg"][1] == 1: logging.info(self.log_msg["app recv msg"][0].format(app_recv))
        self.app.send(app_recv.encode())

    def orecv(self, app_recv,a=100,calib=2):
        if calib==2:
            app_camera_pose='0,0,0,0,0,0'
            # app_camera_pose = ['1,6.4,-165.7,767.8,-165.0,-8.0,-114.9,-37.3,-168.3,871.2,166.9,-6.3,1.7',
            #                    '1,-79.0,-139.8,752.9,-178.4,14.5,117.0,29.2,29.1,134.0,0.0,0.0,-30.0',
            #                     '1,-35.1,-161.3,788.7,-169.8,6.4,179.1,29.7,-3.7,109.0,0.0,0.0,90.0' ,
            #                     '1,-36.7,-152.4,813.7,176.0,13.3,89.6,29.2,-1.0,83.0,0.0,0.0,180.0',
            #                     '1,-39.8,-114.8,830.1,175.5,13.8,90.9,-3.6,0.0,57.2,0.0,0.0,180.0'
            #                    ]
            # app_camera_pose = '1,10.0,203.1,790.3,167.8,9.5,53.5,23.4,-150.6,890.6,175.8,-14.2,-59.8'
            # app_camera_pose = app_camera_pose[a]

            app_camera_pose = self.app.recv(1024).decode("utf-8")
            if self.log_msg["app return msg"][1]==1: logging.info(self.log_msg["app return msg"][0].format(app_camera_pose))  
            # input("请按回车键继续...")
            while '0,0,0,0,0,0' in app_camera_pose:
                input("无返回")
                if self.log_msg["app recv msg"][1] == 1: logging.info(self.log_msg["app recv msg"][0].format(app_recv))
                self.app.send(app_recv.encode())
                app_camera_pose = self.app.recv(1024).decode("utf-8")
                if self.log_msg["app return msg"][1]==1: logging.info(self.log_msg["app return msg"][0].format(app_camera_pose))  
                continue   
            camera_pose_list, pose_param_list, pose_num = self.analysis_app_msg(app_camera_pose)
            robot1_pose_list, robot2_pose_list = self.get_calib_robot_pose_list(camera_pose_list)
            pose_tuple= {"left_arm":robot1_pose_list,"right_arm":robot2_pose_list}     
            return pose_tuple, pose_param_list
        elif calib==1:
            app_send = self.app.recv(1024).decode("utf-8")
            if self.log_msg["app return msg"][1]==1: logging.info(self.log_msg["app return msg"][0].format(app_send))  
            return app_send  

    def send_recv(self, app_recv: str,calib=2):  
        """
        说明:
            标定选择
        参数:
            app_recv: 发送给app的字符串
            calib: 1表示视觉返回机器人坐标系坐标,使用app的标定,2表示视觉返回两个相机坐标系坐标,脚本双臂标定
        """
        if calib==1:
            while True:
                app_send=[]
                if self.log_msg["app recv msg"][1]==1: logging.info(self.log_msg["app recv msg"][0].format(app_recv))  
                self.app.send(app_recv.encode())
                app_send = self.app.recv(1024).decode("utf-8")
                if self.log_msg["app return msg"][1]==1: logging.info(self.log_msg["app return msg"][0].format(app_send))  
                return app_send    
        elif calib==2:
            self.osend(app_recv)  
            pose_tuple, pose_param_list = self.orecv(app_recv)    

            return pose_tuple, pose_param_list  


    def calib_send(self, app_recv: str):  
        if self.log_msg["app recv msg"][1]==1: logging.info(self.log_msg["app recv msg"][0].format(app_recv))  
        self.app.send(app_recv.encode())   
    
    #消息解析函数
    def analysis_app_msg(self,msg):
            pose_elements = msg.split(',')  
            pose_elements = [item for item in pose_elements if item != '']   #去掉所有空的字符串，末尾有逗号就会有空字符串
            pose_num = int(pose_elements[0])+1
            pose_list = []  
            pose_param_list = []
            elements_per_pose = self.init["APP"]["Pose_Type_Num"]

            for i in range(pose_num):  
                start_index = 1 + i * elements_per_pose  
                end_index = start_index + elements_per_pose  
                pose_list.append([float(item) for item in pose_elements[start_index:end_index]])  
            if len(pose_elements)>7 and self.init["APP"]["Pose_Param_Num"] !=0:
                pose_param_list = [float(pose_elements[7]),float(pose_elements[8]),float(pose_elements[9]),float(pose_elements[10])]
                # for i in range(elements_per_param):  
                #     start_index = 1 + pose_num * elements_per_pose + i * elements_per_param  
                #     end_index = start_index + elements_per_param  
                #     pose_param_list.append([float(item) for item in pose_elements[start_index:end_index]]) 
                if self.log_msg["25"][1]==1: logging.info(self.log_msg["25"][0].format(pose_param_list))   

            logging.info("pose_param_list:{}".format(pose_param_list))
            return pose_list , pose_param_list , pose_num

    #机器人位姿计算函数
    def get_calib_robot_pose_list(self,camera_pose_list:list):
        def poseConvert(pose: list, rotation: str):#生成4x4齐次变换矩阵
            pose_m = [x / 1000 for x in pose[:3]] + pose[3:]
            if rotation == 'xyz':
                rot_matrix = Rotation.from_euler(rotation, [pose_m[3], pose_m[4], pose_m[5]], degrees=True).as_matrix()
            elif rotation == 'ZYX':
                rot_matrix = Rotation.from_euler(rotation, [pose_m[5], pose_m[4], pose_m[3]], degrees=True).as_matrix()                
            pose_matrix = np.identity(4)
            pose_matrix[:3, :3] = rot_matrix
            pose_matrix[:3, 3] = [pose_m[0], pose_m[1], pose_m[2]]
            return np.mat(pose_matrix)

        #标定位姿转换，并转换为欧拉角
        def get_calib_pose_shift(pose_matrix: np.mat, calib_matrix: np.mat, Euler: str) -> np.mat:   # type: ignore
            new_matrix = calib_matrix * pose_matrix #手眼转换 
            # print(calib_matrix)
            # print(pose_matrix)            
            new_pose = [  
                round(new_matrix[0, 3], 3), round(new_matrix[1, 3], 3), round(new_matrix[2, 3], 3),  
                round(Rotation.from_matrix(new_matrix[:3, :3]).as_euler(Euler, degrees=True)[0], 3),  
                round(Rotation.from_matrix(new_matrix[:3, :3]).as_euler(Euler, degrees=True)[1], 3),  
                round(Rotation.from_matrix(new_matrix[:3, :3]).as_euler(Euler, degrees=True)[2], 3)  
            ]
            new_pose_mm = [x * 1000 for x in new_pose[:3]] + new_pose[3:]#mm转换成m
            return new_pose_mm
        
        #增加RZ旋转补偿
        def increase_rz(pose_list, offset, euler_order): 
            # 更新pose_list中每个坐标的RZ值  
            for pose in pose_list:  
                if euler_order.upper() == 'ZYX':  
                    pose[3] += offset  
                elif euler_order.upper() == 'XYZ':  
                    pose[5] += offset  
                elif euler_order.lower() == 'xyz':  
                    # 假设内旋'xyz'的RZ是列表中的第6个元素  
                    pose[5] += offset  
                else:  
                    raise ValueError("Invalid Euler order: {}".format(euler_order))  
            return pose_list  
        
        robot1_pose_list = [None] * len(camera_pose_list)  
        robot2_pose_list = [None] * len(camera_pose_list)  
        for i in range(len(camera_pose_list)):
            list_values =camera_pose_list[i]
            temp_camera_pose_matrix = poseConvert(list_values, self.init["ROBOT1"]["Euler"])
            pose_values = get_calib_pose_shift(temp_camera_pose_matrix, self.init["ROBOT1"]["Calib_martix"],self.init["ROBOT1"]["Euler"])
            robot1_pose_list[i] = pose_values
            temp_camera_pose_matrix = poseConvert(list_values, self.init["ROBOT2"]["Euler"])
            pose_values = get_calib_pose_shift(temp_camera_pose_matrix,self.init["ROBOT2"]["Calib_martix"],self.init["ROBOT2"]["Euler"])
            robot2_pose_list[i] = pose_values
        robot1_pose_list = increase_rz(robot1_pose_list,self.init["ROBOT1"]["Gripper_ratate"],self.init["ROBOT1"]["Euler"])
        robot2_pose_list = increase_rz(robot2_pose_list,self.init["ROBOT2"]["Gripper_ratate"],self.init["ROBOT2"]["Euler"])
        for i in range(len(camera_pose_list)):
            # robot1_pose_list[1][5] = 90
            # robot2_pose_list[1][5] = 90
            if self.log_msg["poseturning"][1]==1: logging.info(self.log_msg["poseturning"][0].format(self.init["ROBOT1"]["NAME"], i+1, robot1_pose_list[i]))  
            if self.log_msg["poseturning"][1]==1: logging.info(self.log_msg["poseturning"][0].format(self.init["ROBOT2"]["NAME"], i+1, robot2_pose_list[i]))   
        return robot1_pose_list ,robot2_pose_list

