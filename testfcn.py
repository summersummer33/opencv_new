import cv2
import numpy as np
import math
import time
import serial 
# import testdef
import testdef_pro as testdef
import os
import logging


class FunctionHandler:
    def __init__(self):
        # 硬件资源初始化
        self.cap = None
        self.code_cap = None
        self.ser = testdef.serialInit()
        self.frame_width = 1280
        self.frame_height = 720
        # 全局状态变量
        # 二维码信息
        self.get_order = [3,1,2]
        self.put_order = [2,1,3]
        # 标志位
        # self.line_flag = 0
        # self.move_flag = 0
        self.move_flag_color=0
        self.move_flag_color_1=0
        self.move_flag_color_2=0
        # 轮次计数（后续取消，放到32里计数
        self.circle_time = 1
        self.plate_time = 1
        self.recv = ''
        # 不记得是啥
        self.get_order_blank=[]

    # # 默认初始值，后续改掉
    # get_order=[2,3,1]
    # put_order=[1,3,2]

    def init_camera_up(self):
        """初始化上部摄像头"""
        self.cap = cv2.VideoCapture("/dev/up_video1", cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(3, self.frame_width)
        self.cap.set(4, self.frame_height)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 10)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def init_camera_code(self):
        """初始化二维码摄像头"""
        self.code_cap = cv2.VideoCapture("/dev/code_video1", cv2.CAP_V4L2)
        self.code_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.code_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.code_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.code_cap.set(cv2.CAP_PROP_BRIGHTNESS, 10)
        self.code_cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
    
    def check_camera(self, cap_check, cam_name="摄像头"):
        """检查摄像头状态"""
        if not hasattr(self, 'cap') or cap_check is None:
            print(f"{cam_name}未初始化")
            return False
            
        check_ok = cap_check.isOpened()
        print(f"{cam_name}状态: {'正常' if check_ok else '异常'}")
        return check_ok

    def get_code(self):
        """处理二维码识别"""
        if not self.check_camera(self.code_cap,"二维码摄像头"):
            self.init_camera_code()
        time_c = time.time()
        time_code = 15
        code_end = 0
        data = ""
        # ret = self.code_cap.grab()
        # ret = self.code_cap.grab()
        # print("ret:",ret)
        while (time.time() - time_c < time_code) and (not code_end):
            data, code_flag = testdef.code(self.code_cap)
            if len(data) == 7 and code_flag == 1:
                code_end = 1
                break
        self.get_order = testdef.sort(data[0:3])
        self.put_order = testdef.sort(data[4:7])
        order = self.get_order + self.put_order
        testdef.sendMessage3(self.ser,order)
        time.sleep(0.002)
        testdef.sendMessage3(self.ser,order)
        time.sleep(0.002)
        testdef.sendMessage3(self.ser,order)
        self.code_cap.release()
        self.code_cap = None
        self.recv = b'st'

    def get_from_plate(self, plate_order):
        """处理转盘夹取物料"""
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        stop_flag=0   #初始化转盘是否停止标志位
        i = 0   #运行轮数
        while i < 3:
        ####依据颜色顺序循环处理3个物料
            # print("iii:",i)
            # flagno = testdef.detectPlate(cap, 1)
            ret=self.cap.grab()
            while not stop_flag:
                print("i:",i)
                flag2 = testdef.detectPlate(self.cap,plate_order[i])
                x_,y_,img_,flag1,detx,dety = testdef.findBlockCenter(self.cap,plate_order[i])
                ####当所看对应颜色物料静止时
                if  (flag2 == 1 and flag1 == 1):
                    stop_flag = plate_order[i]  #令标志位为颜色序号
                    print("stop_flag",stop_flag)
                    testdef.sendMessage2(self.ser,detx,dety)  #给机械臂发送大致调整参数，机械臂只动一下
                    time.sleep(0.05)
                    if stop_flag == 1:  #发送到位信息，不同颜色发送不同值
                        testdef.sendMessage(self.ser,7)
                        time.sleep(0.1)
                        testdef.sendMessage(self.ser,7)
                    elif stop_flag == 2:
                        testdef.sendMessage(self.ser,8)
                        time.sleep(0.1)
                        testdef.sendMessage(self.ser,8)
                    elif stop_flag == 3:
                        testdef.sendMessage(self.ser,9)
                        time.sleep(0.1)
                        testdef.sendMessage(self.ser,9)
                    # testdef.sendMessage(ser,stop_flag)
            # Time = time.time()
            stop_flag=0    #重置停止标志位，给下一轮使用
            flag_check=0    #初始化、重置检查物料是否夹到标志位
            ####在第一次夹取时进行二次检查 确保夹到了第一个物料
            if i == 0:
                # Time1=time.time()-Time
                # print("Time1:",Time1)
                cv2.destroyAllWindows()
                ret = self.cap.grab()
                ret = self.cap.grab()
                time.sleep(1)
                print("start checkkkkkkkkkkkkkkkkkkk")
                flag_check =testdef.detectPlate_check(self.cap,plate_order[i])
                # Time2=time.time()-Time
                # print("Time2:",Time2)
                print("flag_chexk:",flag_check)
                if flag_check :  #如果夹到了继续下一个颜色
                    print("next colorrrrrrrrrrrrrrrrrrrr")
                    i=i+1
                else:  #没夹到仍等待第一个,机械臂不回头放物料，直接下降
                    testdef.sendMessage(self.ser,3)
            else :
                time.sleep(3)
                i=i+1
            # time.sleep(3)
            # i=i+1
            ret=self.cap.grab()
            ret=self.cap.grab()
        cv2.destroyAllWindows()
        self.recv=b'st'


    # 粗定位函数：直线和圆环一起调整
    def cu_positioning(self, limit_circle, limit_line, timeout_cu=5):
        # if limit_line is None:
        #     limit_line = 0.3
        """粗定位车身位置（直线和圆环一起调整）"""

        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        # 在开始粗定位前，重置 together_line_circle1 的内部状态
        testdef.reset_together_state()
        
        ret=self.cap.grab()
        print("cccccccccccc")
        line_flag=0   #粗调时的直线圆环标志位置0
        move_flag=0
        # while not self.cap.isOpened():
        #     print("Not open colorcap")
        # for i in range(2):
        #     q=cap.grab()
        ####粗调 圆环粗定位和直线一起调整
        ####粗调开始计时
        Time1=time.time()   
        # time_together=5   #粗调超时
        ####发送偏差值信息，调整车身位置直到超时或者直线圆环均到位
        while ((time.time()-Time1)<timeout_cu) and ((not line_flag) or (not move_flag)) :
            # theta,line_flag,detx,dety,move_flag=testdef.together_line_circle1(self.cap,limit_position_circle=limit_circle, 
            #                                                                   limit_position_line=limit_line)
            theta,line_flag,detx,dety,move_flag=testdef.together_line_circle_det(self.cap,limit_position_circle=limit_circle, 
                                                                              limit_position_line=limit_line)
            if line_flag==0 or move_flag==0:
                if line_flag ==1:   #直线到位则后续角度一直为0（这个逻辑真的能实现吗？--by cy）
                    theta=0
                if move_flag ==1:   #圆环到位则后续xy一直为0
                    detx=0
                    dety=0
                testdef.sendMessage5(self.ser,theta,detx,dety)
        print("together okokokokokokokokok line_flag:",line_flag,"move_flag:",move_flag)
        testdef.sendMessage(self.ser,68)  #发送到位信息
        time.sleep(0.001)
        move_flag=0  #标志位清零以便下次使用
        line_flag=0
        ret=self.cap.grab()
        cv2.destroyAllWindows()

    # 细调函数：颜色定位和灰度定位
    def xi_positioning(self, circle_order, timeout_xi=1.5, run_time=3):
        """细调圆环位置（颜色定位和灰度定位）"""
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        for i in range(run_time):
            ret=self.cap.grab()
            testdef.g_prev_smoothed_circle=None
            print("iiiiiiiiiiiiii:",i,"color:",circle_order[i])
            ####接收到爪子下降消息再开始进入细调
            recv_first=None
            while True:
                recv_first=testdef.receiveMessage(self.ser)
                # if recv_first != None:
                #     print("recv_first",recv_first)
                if recv_first==b'nearground':
                    print("recv_first",recv_first)
                    break
            ####细调开始计时
            # for j in range(3):
            #     q=cap.grab()
            Time3=time.time()
            ####细调第一步 颜色定五环
            move_flag_color_1=0 
            while (not move_flag_color_1 and (time.time()-Time3)<timeout_xi):
            # while (not move_flag_color_1):
            # while((time.time()-Time3)<timeout_xi):
                timee=time.time()
                print("cccccccccccc")
                # q=cap.grab()
                x_,y_,img_,move_flag_color_1,detx_p,dety_p = testdef.circlePut_color(self.cap,circle_order[i])
                if move_flag_color_1==0:    #flag=1后那次数据不需发送
                    testdef.sendMessage2(self.ser,detx_p,dety_p)
                    print("cutiao time:",time.time()-timee)
            print("xitiao11 okokokokokokokokok")
            move_flag_color_1=0   

            # move_flag_color_2 = 0
            # stable_count = 0
            # required_stable_frames = 3  # 需要连续稳定的帧数
            # flag_mid = 0
            # while (stable_count < required_stable_frames) and (time.time()-Time3)<timeout_xi:
            #     timeee=time.time()
            #     print("xxxxxxxx")
            #     timeee = time.time()
            #     detx, dety, flag_mid  = testdef.circlePut1(self.cap)
                
            #     if flag_mid:
            #         stable_count += 1
            #     else:
            #         stable_count = 0  # 重置计数器
            #         testdef.sendMessage2(self.ser, detx, dety)
            #     if stable_count == required_stable_frames:
            #         move_flag_color_2 = 1
                
            #     print("xitiao time:", time.time()-timeee)            

            #细调第二步 灰度定中心（第一版-无到位后二次检测
            testdef.reset_circle_put_state()
            move_flag_color_2=0
            cnt = 0
            # while (not move_flag_color_2 and (time.time()-Time3)<timeout_xi):
            # while (not move_flag_color_2 ):
            while( time.time()-Time3<timeout_xi):
                print("xxxxxxxx")
                timeee=time.time()
                # detx,dety,move_flag_color_2=testdef.circlePut1(self.cap)
                # detx,dety,move_flag_color_2=testdef.circlePut_hzw(self.cap)
                detx,dety,move_flag_color_2=testdef.circlePut_det(self.cap)
                cnt += 1
                # if move_flag_color_2==0:
                #     testdef.sendMessage2(self.ser,detx,dety)
                #     print("xitiao time:",time.time()-timeee)
                if move_flag_color_2 == 1 and cnt>1:
                    break
                else:
                    testdef.sendMessage2(self.ser,detx,dety)
                    print("xitiao time:",time.time()-timeee)
            # ###细调第二步 灰度定中心（第二版-到位后做二次检测-防止物料贴环边立即就放
            # move_flag_color_2_2=0
            # while (not move_flag_color_2_2 and (time.time()-Time3)<timeout_xi):
            # # while (not move_flag_color_2_2 ):
            #     print("xxxxxxxx")
            #     timeee=time.time()
            #     detx,dety,move_flag_color_2=testdef.circlePut1(self.cap)
            #     # timeuart=time.time()
            #     # print("xitiao time:",time.time()-timeee)
            #     # testdef.sendMessage2(ser,detx,dety)
            #     # print("xitiao time:",time.time()-timeuart)
            #     if move_flag_color_2==0:
            #         # a=1
            #         testdef.sendMessage2(self.ser,detx,dety)
            #         print("xitiao time:",time.time()-timeee)
            #     else:
            #         # detxx=0
            #         # detyy=0
            #         flagg=0
            #         time_check=2
            #         ####初次到位后再看一次是否还在中心内 若在则到位 若不在则继续调整
            #         for k in range(time_check):
            #             detx2,dety2,move_flag_color_22=testdef.circlePut1(self.cap)
            #             testdef.sendMessage2(self.ser,detx2,dety2)
            #             # print("double check")
            #             # detxx+=detx
            #             # detyy+=dety
            #             flagg+=move_flag_color_22
            #             print("double check    flagg:",flagg)
            #         if flagg==time_check:
            #             move_flag_color_2_2=1
            #             break
            #         move_flag_color_2_2=1
            #         break
            print("xitiao22 okokokokokokokokok  move_flag_color_2:",move_flag_color_2)
            ####发送到位信息 根据颜色发送不同到位信息
            if circle_order[i] == 1:
                testdef.sendMessage(self.ser,57)
            elif circle_order[i] == 2:
                testdef.sendMessage(self.ser,64)
            elif circle_order[i] == 3:
                testdef.sendMessage(self.ser,65)
            time.sleep(0.01)
            move_flag_color_2=0
            # i = i+1  #继续下一个颜色
            cv2.destroyAllWindows()
        ret=self.cap.grab()
        self.recv=b'st'  #完成功能后进入空循环

    def adjust_line_gray_yellow(self,timeout_line=1.2):
        """调整直线——灰黄交界"""
        # while not self.cap.isOpened():
        #     print("Not open colorcap")
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        ####开始计时
        line_flag=0
        ret=self.cap.grab()



        Time_l=time.time()
        ####调整车身姿态直到直线到位或超时
        while (not line_flag and (time.time()-Time_l)<timeout_line):
        # while (not line_flag):
            ####清理视频流缓存区
            # for i in range(4):
            #     # theta1,line_flag1=testdef.detectLine(cap)
            #     ret=cap.grab()
            theta,line_flag=testdef.detectLine_gray(self.cap)
            # theta,line_flag=testdef.detectLine(cap)
            if line_flag ==0:   #到位后当次偏差值不发送
                testdef.sendMessage5(self.ser,theta,0,0)
                print("main li de theta:",theta)
            # elif line_flag==1:
        print("line_flag:",line_flag)
        testdef.sendMessage(self.ser,39)
        time.sleep(0.003)
        testdef.sendMessage(self.ser,40)
        time.sleep(0.003)
        testdef.sendMessage(self.ser,68)
        line_flag=0
        ret=self.cap.grab()
        ret=self.cap.grab()

    def cu_positioning_test(self,timeout_cu=5):
        '''测试代码：粗调先调直线再调xy方向'''
        ret=self.cap.grab()
        print("cccccccccccc")
        line_flag=0   #粗调时的直线圆环标志位置0
        move_flag=0
        while not self.cap.isOpened():
            print("Not open colorcap")
        # for i in range(2):
        #     q=cap.grab()
        ####粗调 圆环粗定位和直线一起调整
        ####粗调开始计时
        Time1=time.time()   
        ####发送偏差值信息，调整车身位置直到超时或者直线圆环均到位
        while ((time.time()-Time1)<timeout_cu) and ((not line_flag) or (not move_flag)) :
            theta,line_flag,detx,dety,move_flag=testdef.together_line_circle1(self.cap)
            if line_flag==0:
                detx=0
                dety=0
            elif line_flag ==1:   #直线到位则后续角度一直为0
                theta=0
            testdef.sendMessage5(self.ser,theta,detx,dety)
        print("together okokokokokokokokok line_flag:",line_flag,"move_flag:",move_flag)
        testdef.sendMessage(self.ser,68)  #发送到位信息
        time.sleep(0.001)
        move_flag=0  #标志位清零以便下次使用
        line_flag=0
        ret=self.cap.grab()
        cv2.destroyAllWindows()


    def plate_adjust_then_put(self, plate_order, adjust_finely=0):
        i=0
        # while not self.cap.isOpened():
        #     print("Not open colorcap")
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        print("plate_order:",plate_order)
        stop_flag=0
        # i=0
        stop_flag_1=0
        stop_first=0
        x_last=0
        y_last=0
        detx_last = 0
        dety_last = 0
        # while not stop_first:
        #     flag2, direction = testdef.detectPlate_gray(self.cap)
        #     x_,y_,img_,flag1,detx,dety,color = testdef.findBlockCenter_gray(self.cap)
        #     if  (flag2 == 1 and flag1 == 1):
        #         x_last=x_
        #         y_last=y_
        #         while not stop_first:
        #             x_,y_,img_,flag_,detx_,dety_,color_number= testdef.findBlockCenter_gray(self.cap)
        #             if abs(x_last-x_)<0.05 and abs(y_last-y_)<0.05:
        #                 print("1")
        #                 x_last=x_
        #                 y_last=y_
        #                 detx_last = detx_
        #                 dety_last = dety_
        #             else:
        #                 stop_first=1
        # 1. 引入新变量来存储静止时的精确偏差
        is_locked = False       # 标志位，表示我们是否已经锁定了静止的圆环
        locked_detx = 0         # 存储静止时测得的精确 detx
        locked_dety = 0         # 存储静止时测得的精确 dety
        is_stopped = False
        
        # 2. 修改你的主循环，使其成为一个状态切换的逻辑
        # 我们不再需要 stop_first 标志，is_locked 更清晰
        while not is_locked:
            # 持续检测转盘是否停止 和 圆环是否存在
            is_stopped, direction = testdef.detectPlate_gray(self.cap)
            x_, y_, img_, is_found, detx, dety, color = testdef.findBlockCenter_gray(self.cap)
            if is_found and is_stopped:
                print("检测到静止圆环！开始进行精确采样...")
                # 进行多次采样以获得精确偏差
                detx_samples = []
                dety_samples = []
                for _ in range(5):
                    _, _, _, f, dx, dy, _ = testdef.findBlockCenter_gray(self.cap)
                    if f:
                        detx_samples.append(dx)
                        dety_samples.append(dy)
                    # time.sleep(0.05)
                if detx_samples:
                    locked_detx = round(np.mean(detx_samples))
                    locked_dety = round(np.mean(dety_samples))
                    is_locked = True # 成功锁定！退出这个循环
                    print(f"成功锁定！静止偏差为: detx={locked_detx:.2f},d dety={locked_dety:.2f}")
                else:
                    print("采样失败，将重试...")
        # 3. 锁定成功后，进入下一个阶段：等待圆环离开
        print("已锁定位置，现在等待圆环离开...")
        has_departed = False
        while not has_departed:
            is_stopped, direction = testdef.detectPlate_gray(self.cap) # 持续检测停止状态
            # x_, y_, img_, is_found, detx, dety, color = testdef.findBlockCenter_gray(self.cap)
            if not is_stopped:
                has_departed = True
                print("检测到转盘开始移动，圆环已离开！")
        print("kaishidongjixiebi kaishidongjixiebi")
        if adjust_finely == 1:
            while not stop_flag_1:
                flag2, direction = testdef.detectPlate_gray(self.cap)
                x_,y_,img_,flag1,detx,dety,color = testdef.findBlockCenter_gray(self.cap)
                if  (flag2 == 1 and flag1 == 1):
                # if flag2==1: 
                    time_start=time.time()
                    while (not stop_flag_1 and (time.time()-time_start)<3.2):
                        x_,y_,img_,flag9,detx9,dety9,color = testdef.findBlockCenter_gray(self.cap)
                        print("qqqqqqqq:",abs(detx9),abs(dety9))
                        # if abs(detx9)<12 and abs(dety9)<12 and detx9!=0 and dety9!=0:
                        if abs(detx9)<5 and abs(dety9)<5 and flag9==1:
                            stop_flag_1=1
                            print("stop_flag_1:",stop_flag_1)
                        else:
                            testdef.sendMessage2(self.ser,detx9,dety9)
                            time.sleep(0.01)
                    if stop_flag_1==1:
                        testdef.sendMessage(self.ser,57)
                        time.sleep(3.5)#避免在该圆环下定位后立即判断该色到位，加一个延时，放掉这个圆环，对
                    else:
                        print("chaoshilechaoshilechaoshilechaoshile")
                        time.sleep(2)
        else:
            testdef.sendMessage2(self.ser,locked_detx,locked_dety)
            time.sleep(2)
        # testdef.sendMessage(self.ser,57)
        while i<3:
            while not stop_flag:
                print("i:",i)
            #     # if stop_flag_1==0:
                flag2 = testdef.detectPlate(self.cap,plate_order[i])
                x_,y_,img_,flag1,detx,dety = testdef.findBlockCenter(self.cap,plate_order[i])
                if  (flag2 == 1 and flag1 == 1):
                    # stop_flag=plate_order[i]
                    # if color==plate_order[i]:
                        stop_flag=1
                        # if plate_order[i] == 1:
                        #     testdef.sendMessage(self.ser,7)
                        # elif plate_order[i] == 2:
                        #     testdef.sendMessage(self.ser,8)
                        # elif plate_order[i]== 3:
                        #     testdef.sendMessage(self.ser,9)
                        testdef.sendMessage(self.ser,119)
                    # testdef.sendMessage(ser,stop_flag)
            # Time = time.time()
            stop_flag=0
            stop_flag_1=0
            time.sleep(4)
            i=i+1
        # plate_time += 1
        cv2.destroyAllWindows()


    def plate_adjust_then_put_pre_color(self, plate_order, adjust_finely=0):
        """
        中间的精调是动机械臂(adjust_finely=1)
        """
        i=0
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        print("plate_order:",plate_order)
        stop_flag=0
        # i=0
        stop_flag_1=0
        stop_first=0
        x_last=0
        y_last=0
        # while not stop_first:
        #     flag2, direction = testdef.detectPlate_gray(self.cap)
        #     x_,y_,img_,flag1,detx,dety,color = testdef.findBlockCenter_gray(self.cap)
        #     if  (flag2 == 1 and flag1 == 1):
        #         x_last=x_
        #         y_last=y_
        #         while not stop_first:
        #             x_,y_,img_,flag_,detx_,dety_,color_number= testdef.findBlockCenter_gray(self.cap)
        #             if abs(x_last-x_)<0.05 and abs(y_last-y_)<0.05:
        #                 print("1")
        #                 x_last=x_
        #                 y_last=y_
        #             else:
        #                 stop_first=1
        # print("kaishidongjixiebi kaishidongjixiebi")
        # 1. 引入新变量来存储静止时的精确偏差
        is_locked = False       # 标志位，表示我们是否已经锁定了静止的圆环
        locked_detx = 0         # 存储静止时测得的精确 detx
        locked_dety = 0         # 存储静止时测得的精确 dety
        is_stopped = False
        
        # 2. 修改你的主循环，使其成为一个状态切换的逻辑
        # 我们不再需要 stop_first 标志，is_locked 更清晰
        while not is_locked:
            # 持续检测转盘是否停止 和 圆环是否存在
            is_stopped, direction = testdef.detectPlate_gray(self.cap)
            x_, y_, img_, is_found, detx, dety, color = testdef.findBlockCenter_gray(self.cap)
            if is_found and is_stopped:
                print("检测到静止圆环！开始进行精确采样...")
                # 进行多次采样以获得精确偏差
                detx_samples = []
                dety_samples = []
                for _ in range(5):
                    _, _, _, f, dx, dy, _ = testdef.findBlockCenter_gray(self.cap)
                    if f:
                        detx_samples.append(dx)
                        dety_samples.append(dy)
                    # time.sleep(0.05)
                if detx_samples:
                    locked_detx = round(np.mean(detx_samples))
                    locked_dety = round(np.mean(dety_samples))
                    is_locked = True # 成功锁定！退出这个循环
                    print(f"成功锁定！静止偏差为: detx={locked_detx:.2f},d dety={locked_dety:.2f}")
                else:
                    print("采样失败，将重试...")
        # 3. 锁定成功后，进入下一个阶段：等待圆环离开
        print("已锁定位置，现在等待圆环离开...")
        has_departed = False
        while not has_departed:
            is_stopped, direction = testdef.detectPlate_gray(self.cap) # 持续检测停止状态
            # x_, y_, img_, is_found, detx, dety, color = testdef.findBlockCenter_gray(self.cap)
            if not is_stopped:
                has_departed = True
                print("检测到转盘开始移动，圆环已离开！")
        print("kaishidongjixiebi kaishidongjixiebi")
        if adjust_finely == 1:
            while not stop_flag_1:
                flag2, direction = testdef.detectPlate_gray(self.cap)
                x_,y_,img_,flag1,detx,dety,color = testdef.findBlockCenter_gray(self.cap)
                if  (flag2 == 1 and flag1 == 1):
                # if flag2==1: 
                    time_start=time.time()
                    while (not stop_flag_1 and (time.time()-time_start)<3.2):
                        x_,y_,img_,flag9,detx9,dety9,color = testdef.findBlockCenter_gray(self.cap)
                        print("qqqqqqqq:",abs(detx9),abs(dety9))
                        # if abs(detx9)<12 and abs(dety9)<12 and detx9!=0 and dety9!=0:
                        if abs(detx9)<12 and abs(dety9)<12 and flag9==1:
                            stop_flag_1=1
                            print("stop_flag_1:",stop_flag_1)
                        else:
                            testdef.sendMessage2(self.ser,detx9,dety9)
                            time.sleep(0.01)
                    if stop_flag_1==1:
                        testdef.sendMessage(self.ser,57)
                        time.sleep(3.5)#避免在该圆环下定位后立即判断该色到位，加一个延时，放掉这个圆环，对
                    else:
                        print("chaoshilechaoshilechaoshilechaoshile")
                        time.sleep(2)
        else:
            testdef.sendMessage2(self.ser,locked_detx,locked_dety)
            time.sleep(2)
        i=0
        while i<3:
            # while not stop_flag:
            #     print("i:",i)
            # #     # if stop_flag_1==0:
            #     flag2 = testdef.detectPlate(cap,plate_order[i])
            #     x_,y_,img_,flag1,detx,dety = testdef.findBlockCenter(cap,plate_order[i])
            #     if  (flag2 == 1 and flag1 == 1):
            #         # stop_flag=plate_order[i]
            #         # if color==plate_order[i]:
            #             stop_flag=1
            #             if plate_order[i] == 1:
            #                 testdef.sendMessage(ser,7)
            #             elif plate_order[i] == 2:
            #                 testdef.sendMessage(ser,8)
            #             elif plate_order[i]== 3:
            #                 testdef.sendMessage(ser,9)
            color_=0
            if (plate_order[i]==1):
                color_=2
            elif(plate_order[i]==2):
                color_=3
            elif(plate_order[i]==3):
                color_=1
            # if (plate_order[i]==1):
            #     color_=3
            # elif(plate_order[i]==2):
            #     color_=1
            # elif(plate_order[i]==3):
            #     color_=2
            while not stop_flag:
                flag2 = testdef.detectPlate(self.cap,color_)
                x_,y_,img_,flag1,detx,dety = testdef.findBlockCenter(self.cap,color_)
                if  (flag2 == 1 and flag1 == 1):
                    x_last=x_
                    y_last=y_
                    while not stop_flag:
                        x_,y_,img_,flag_,detx_,dety_,color_number= testdef.findBlockCenter_gray(self.cap)
                        if abs(x_last-x_)<0.05 and abs(y_last-y_)<0.05:
                            print("1")
                            x_last=x_
                            y_last=y_
                        else:
                            stop_flag=1
            testdef.sendMessage(self.ser,119)
            stop_flag=0
            stop_flag_1=0
            time.sleep(4)
            i=i+1
        cv2.destroyAllWindows()


    def plate_adjust_then_put_pre_color_pro(self, plate_order, adjust_finely=0):
        """
        在位置锁定阶段"同时"检测灰度圆环和颜色圆环
        放第一个颜色时直接调整机械臂位置 
        """
        # while not self.cap.isOpened():
        #     print("Not open colorcap")
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        print("plate_order:",plate_order)
        stop_flag=0
        stop_flag_1=0
        x_last=0
        y_last=0
        # 1. 引入新变量来存储静止时的精确偏差
        is_locked = False       # 标志位，表示我们是否已经锁定了静止的圆环
        locked_detx = 0         # 存储静止时测得的精确 detx
        locked_dety = 0         # 存储静止时测得的精确 dety
        is_stopped = False
        flag_color = 0
        i=0
        for i in range(3):
        # while i<3:
            color_=0
            if (plate_order[i]==1):
                color_=2
            elif(plate_order[i]==2):
                color_=3
            elif(plate_order[i]==3):
                color_=1
            # if (plate_order[i]==1):
            #     color_=3
            # elif(plate_order[i]==2):
            #     color_=1
            # elif(plate_order[i]==3):
            #     color_=2

            if i == 0:
                while not is_locked:
                    # 持续检测转盘是否停止 和 圆环是否存在
                    is_stopped, direction = testdef.detectPlate_gray(self.cap)
                    x_, y_, img_, is_found, detx, dety, color = testdef.findBlockCenter_gray(self.cap)
                    x_,y_,img_,flag_color,detx,dety = testdef.findBlockCenter(self.cap,color_)
                    if is_found and is_stopped:
                        print("检测到静止圆环！开始进行精确采样...")
                        # 进行多次采样以获得精确偏差
                        detx_samples = []
                        dety_samples = []
                        for _ in range(5):
                            _, _, _, f, dx, dy, _ = testdef.findBlockCenter_gray(self.cap)
                            if f:
                                detx_samples.append(dx)
                                dety_samples.append(dy)
                            # time.sleep(0.05)
                        if detx_samples:
                            locked_detx = round(np.mean(detx_samples))
                            locked_dety = round(np.mean(dety_samples))
                            is_locked = True # 成功锁定！退出这个循环
                            print(f"成功锁定！静止偏差为: detx={locked_detx:.2f},d dety={locked_dety:.2f}")
                        else:
                            print("采样失败，将重试...")
                # 3. 锁定成功后，进入下一个阶段：等待圆环离开
                print("已锁定位置，现在等待圆环离开...")
                has_departed = False
                while not has_departed:
                    is_stopped, direction = testdef.detectPlate_gray(self.cap) # 持续检测停止状态
                    # x_, y_, img_, is_found, detx, dety, color = testdef.findBlockCenter_gray(self.cap)
                    if not is_stopped:
                        has_departed = True
                        print("检测到转盘开始移动，圆环已离开！")
                print("kaishidongjixiebi kaishidongjixiebi")
                if adjust_finely == 1:
                    while not stop_flag_1:
                        flag2, direction = testdef.detectPlate_gray(self.cap)
                        x_,y_,img_,flag1,detx,dety,color = testdef.findBlockCenter_gray(self.cap)
                        if  (flag2 == 1 and flag1 == 1):
                        # if flag2==1: 
                            time_start=time.time()
                            while (not stop_flag_1 and (time.time()-time_start)<3.2):
                                x_,y_,img_,flag9,detx9,dety9,color = testdef.findBlockCenter_gray(self.cap)
                                print("qqqqqqqq:",abs(detx9),abs(dety9))
                                # if abs(detx9)<12 and abs(dety9)<12 and detx9!=0 and dety9!=0:
                                if abs(detx9)<12 and abs(dety9)<12 and flag9==1:
                                    stop_flag_1=1
                                    print("stop_flag_1:",stop_flag_1)
                                else:
                                    testdef.sendMessage2(self.ser,detx9,dety9)
                                    time.sleep(0.01)
                            if stop_flag_1==1:
                                testdef.sendMessage(self.ser,57)
                                time.sleep(3.5)#!避免在该圆环下定位后立即判断该色到位，加一个延时，放掉这个圆环，对
                            else:
                                print("chaoshilechaoshilechaoshilechaoshile")
                                time.sleep(2)
                else:
                    testdef.sendMessage2(self.ser,locked_detx,locked_dety)
                    time.sleep(0.05)
                    if flag_color:
                        testdef.sendMessage(self.ser,119)
                        stop_flag=0
                        stop_flag_1=0
                        time.sleep(4)
                        continue





            while not stop_flag:
                flag2 = testdef.detectPlate(self.cap,color_)
                x_,y_,img_,flag1,detx,dety = testdef.findBlockCenter(self.cap,color_)
                if  (flag2 == 1 and flag1 == 1):
                    x_last=x_
                    y_last=y_
                    while not stop_flag:
                        # x_,y_,img_,flag_,detx_,dety_,color_number= testdef.findBlockCenter_gray(self.cap)
                        # if abs(x_last-x_)<0.05 and abs(y_last-y_)<0.05:
                        #     print("1")
                        #     x_last=x_
                        #     y_last=y_
                        # else:
                        #     stop_flag=1
                        # stop_flag_pre, direction = testdef.detectPlate_gray(self.cap) # 持续检测停止状态
                        stop_flag_pre = testdef.detectPlate(self.cap,color_)
                        if not stop_flag_pre:
                            stop_flag = 1
                        elif stop_flag_pre:
                            print("1")
            testdef.sendMessage(self.ser,119)
            stop_flag=0
            stop_flag_1=0
            time.sleep(4)  #我看到上个颜色圆环离开-通知机械臂回身取物料-此延时用于防止在这个过程中看到颜色动与不动造成误判断
        cv2.destroyAllWindows()


    def plate_adjust_then_put_pre_color_pro_move_car(self, plate_order, adjust_finely=0):
        """
        在位置锁定阶段"同时"检测灰度圆环和颜色圆环
        放第一个颜色时直接调整机械臂位置 
        """
        # while not self.cap.isOpened():
        #     print("Not open colorcap")
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        print("plate_order:",plate_order)
        stop_flag=0
        stop_flag_1=0
        x_last=0
        y_last=0
        # 1. 引入新变量来存储静止时的精确偏差
        is_locked = False       # 标志位，表示我们是否已经锁定了静止的圆环
        locked_detx = 0         # 存储静止时测得的精确 detx
        locked_dety = 0         # 存储静止时测得的精确 dety
        is_stopped = False
        flag_color = 0
        i=0
        for i in range(3):
        # while i<3:
            color_=0
            # if (plate_order[i]==1):
            #     color_=2
            # elif(plate_order[i]==2):
            #     color_=3
            # elif(plate_order[i]==3):
            #     color_=1
            if (plate_order[i]==1):
                color_=3
            elif(plate_order[i]==2):
                color_=1
            elif(plate_order[i]==3):
                color_=2

            if i == 0:
                while not is_locked:
                    # 持续检测转盘是否停止 和 圆环是否存在
                    is_stopped, direction = testdef.detectPlate_gray(self.cap)
                    x_, y_, img_, is_found, detx, dety, color = testdef.findBlockCenter_gray(self.cap)
                    x_,y_,img_,flag_color,detx,dety = testdef.findBlockCenter(self.cap,color_)
                    if is_found and is_stopped:
                        print("检测到静止圆环！开始进行精确采样...")
                        # 进行多次采样以获得精确偏差
                        detx_samples = []
                        dety_samples = []
                        for _ in range(5):
                            _, _, _, f, dx, dy, _ = testdef.findBlockCenter_gray(self.cap)
                            if f:
                                detx_samples.append(dx)
                                dety_samples.append(dy)
                            # time.sleep(0.05)
                        if detx_samples:
                            locked_detx = round(np.mean(detx_samples))
                            locked_dety = round(np.mean(dety_samples))
                            is_locked = True # 成功锁定！退出这个循环
                            # print(f"成功锁定！静止偏差为: detx={locked_detx:.2f},d dety={locked_dety:.2f}")
                        else:
                            print("采样失败，将重试...")
                # 3. 锁定成功后，进入下一个阶段：等待圆环离开
                # print("已锁定位置，现在等待圆环离开...")
                has_departed = False
                while not has_departed:
                    is_stopped, direction = testdef.detectPlate_gray(self.cap) # 持续检测停止状态
                    # x_, y_, img_, is_found, detx, dety, color = testdef.findBlockCenter_gray(self.cap)
                    if not is_stopped:
                        has_departed = True
                        print("检测到转盘开始移动，圆环已离开！")
                print("开始动底盘")
                if adjust_finely == 1:
                    while not stop_flag_1:
                        flag2, direction = testdef.detectPlate_gray(self.cap)
                        x_,y_,img_,flag1,detx,dety,color = testdef.findBlockCenter_gray(self.cap)
                        if  (flag2 == 1 and flag1 == 1):
                        # if flag2==1: 
                            time_start=time.time()
                            while (not stop_flag_1 and (time.time()-time_start)<2.7):
                                x_,y_,img_,flag9,detx9,dety9,color = testdef.findBlockCenter_gray(self.cap)
                                # print("qqqqqqqq:",abs(detx9),abs(dety9))
                                # if abs(detx9)<12 and abs(dety9)<12 and detx9!=0 and dety9!=0:
                                if abs(detx9)<10 and abs(dety9)<10 and flag9==1:
                                    stop_flag_1=1
                                    print("stop_flag_1:",stop_flag_1)
                                else:
                                    testdef.sendMessage5(self.ser,0,detx9,dety9)
                                    # time.sleep(0.1)
                            if stop_flag_1==1:
                                testdef.sendMessage(self.ser,68)
                                # time.sleep(3.5)#!避免在该圆环下定位后立即判断该色到位，加一个延时，放掉这个圆环，对
                            else:
                                print("chaoshilechaoshilechaoshilechaoshile")
                                time.sleep(2)
                else:
                    testdef.sendMessage2(self.ser,locked_detx,locked_dety)
                    time.sleep(0.05)
                    if flag_color:
                        testdef.sendMessage(self.ser,119)
                        stop_flag=0
                        stop_flag_1=0
                        time.sleep(4)
                        continue

            while not stop_flag:
                flag2 = testdef.detectPlate(self.cap,color_)
                x_,y_,img_,flag1,detx,dety = testdef.findBlockCenter(self.cap,color_)
                if  (flag2 == 1 and flag1 == 1):
                    x_last=x_
                    y_last=y_
                    while not stop_flag:
                        stop_flag_pre = testdef.detectPlate(self.cap,color_)
                        if not stop_flag_pre:
                            stop_flag = 1
                        elif stop_flag_pre:
                            print("1")
            testdef.sendMessage(self.ser,119)
            stop_flag=0
            stop_flag_1=0
            time.sleep(4)  #我看到上个颜色圆环离开-通知机械臂回身取物料-此延时用于防止在这个过程中看到颜色动与不动造成误判断
        cv2.destroyAllWindows()


    
    def plate_adjust_then_put_pre_color_faster(self, plate_order):
        """
        静止时放走第一个 转动时直接等待并精调
        无机械臂动一下方案
        """
        # while not self.cap.isOpened():
        #     print("Not open colorcap")
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        print("plate_order:",plate_order)
        stop_flag=0
        stop_flag_1=0
        x_last=0
        y_last=0
        # 1. 引入新变量来存储静止时的精确偏差
        is_locked = False       # 标志位，表示我们是否已经锁定了静止的圆环
        locked_detx = 0         # 存储静止时测得的精确 detx
        locked_dety = 0         # 存储静止时测得的精确 dety
        is_stopped = False
        flag_color = 0
        i=0
        for i in range(3):
        # while i<3:
            color_=0
            if (plate_order[i]==1):
                color_=2
            elif(plate_order[i]==2):
                color_=3
            elif(plate_order[i]==3):
                color_=1
            # if (plate_order[i]==1):
            #     color_=3
            # elif(plate_order[i]==2):
            #     color_=1
            # elif(plate_order[i]==3):
            #     color_=2

            if i == 0:
                initial_is_stopped, _ = testdef.detectPlate_gray(self.cap)
                _, _, _, initial_is_found, _, _, _ = testdef.findBlockCenter_gray(self.cap)
                x_,y_,img_,flag_color,detx,dety = testdef.findBlockCenter(self.cap,color_)
                if initial_is_stopped and initial_is_found:
                    print("### 情况一：初始静止，执行'先锁定，后精调'策略 ###")
                    # 3. 锁定成功后，进入下一个阶段：等待圆环离开
                    print("已锁定位置，现在等待圆环离开...")
                    has_departed = False
                    while not has_departed:
                        is_stopped, direction = testdef.detectPlate_gray(self.cap) # 持续检测停止状态
                        # x_, y_, img_, is_found, detx, dety, color = testdef.findBlockCenter_gray(self.cap)
                        if not is_stopped:
                            has_departed = True
                            print("检测到转盘开始移动，圆环已离开！")
                else:
                    print("### 情况二：初始转动，执行'直接等待并精调'策略 ###")
                    pass # 明确表示我们在这里是故意跳过的
                print("kaishidongjixiebi kaishidongjixiebi")
                
                while not stop_flag_1:
                    flag2, direction = testdef.detectPlate_gray(self.cap)
                    x_,y_,img_,flag1,detx,dety,color = testdef.findBlockCenter_gray(self.cap)
                    if  (flag2 == 1 and flag1 == 1):
                    # if flag2==1: 
                        time_start=time.time()
                        while (not stop_flag_1 and (time.time()-time_start)<2.5):
                            x_,y_,img_,flag9,detx9,dety9,color = testdef.findBlockCenter_gray(self.cap)
                            print("qqqqqqqq:",abs(detx9),abs(dety9))
                            # if abs(detx9)<12 and abs(dety9)<12 and detx9!=0 and dety9!=0:
                            if abs(detx9)<12 and abs(dety9)<12 and flag9==1:
                                stop_flag_1=1
                                print("stop_flag_1:",stop_flag_1)
                            else:
                                testdef.sendMessage2(self.ser,detx9,dety9)
                                time.sleep(0.01)
                        if stop_flag_1==1:
                            testdef.sendMessage(self.ser,57)
                            # time.sleep(3.5)#!避免在该圆环下定位后立即判断该色到位，加一个延时，放掉这个圆环，对
                        else:
                            print("chaoshilechaoshilechaoshilechaoshile")
                            time.sleep(2)


            while not stop_flag:
                flag2 = testdef.detectPlate(self.cap,color_)
                x_,y_,img_,flag1,detx,dety = testdef.findBlockCenter(self.cap,color_)
                if  (flag2 == 1 and flag1 == 1):
                    x_last=x_
                    y_last=y_
                    while not stop_flag:
                        stop_flag_pre = testdef.detectPlate(self.cap,color_)
                        if not stop_flag_pre:
                            stop_flag = 1
                        elif stop_flag_pre:
                            print("1")
            testdef.sendMessage(self.ser,119)
            stop_flag=0
            stop_flag_1=0
            time.sleep(4)  #我看到上个颜色圆环离开-通知机械臂回身取物料-此延时用于防止在这个过程中看到颜色动与不动造成误判断
        cv2.destroyAllWindows()



    def plate_adjust_then_put_nocolor_ring(self, adjust_finely=0):
        """
        在位置锁定阶段检测灰度圆环，中间的精调部分是动底盘(adjust_finely=1)
        """
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        stop_flag=0
        stop_flag_1=0
        x_last=0
        y_last=0
        # 1. 引入新变量来存储静止时的精确偏差
        is_locked = False       # 标志位，表示我们是否已经锁定了静止的圆环
        locked_detx = 0         # 存储静止时测得的精确 detx
        locked_dety = 0         # 存储静止时测得的精确 dety
        is_stopped = False
        flag_color = 0
        i=0
        for i in range(3):

            if i == 0:
                while not is_locked:
                    # 持续检测转盘是否停止 和 圆环是否存在
                    is_stopped = testdef.detectPlate_nocolor_ring(self.cap)
                    x_,y_,img_,is_found,detx,dety=testdef.enhance_and_find_ring_new(self.cap)
                    if is_found and is_stopped:
                        print("检测到静止圆环！开始进行精确采样...")
                        # 进行多次采样以获得精确偏差
                        detx_samples = []
                        dety_samples = []
                        for _ in range(5):
                            x_,y_,img_,is_found,detx,dety=testdef.enhance_and_find_ring_new(self.cap)
                            if is_found:
                                detx_samples.append(detx)
                                dety_samples.append(dety)
                            # time.sleep(0.05)
                        if detx_samples:
                            locked_detx = round(np.mean(detx_samples))
                            locked_dety = round(np.mean(dety_samples))
                            is_locked = True # 成功锁定！退出这个循环
                            print(f"成功锁定！静止偏差为: detx={locked_detx:.2f},d dety={locked_dety:.2f}")
                        else:
                            print("采样失败，将重试...")
                # 3. 锁定成功后，进入下一个阶段：等待圆环离开
                print("等待圆环离开...")
                has_departed = False
                while not has_departed:
                    is_stopped = testdef.detectPlate_nocolor_ring(self.cap) # 持续检测停止状态
                    # x_, y_, img_, is_found, detx, dety, color = testdef.findBlockCenter_gray(self.cap)
                    if not is_stopped:
                        has_departed = True
                        print("圆环离开！")
                print("等待开始调整底盘")
                if adjust_finely == 1:
                    while not stop_flag_1:
                        flag2= testdef.detectPlate_nocolor_ring(self.cap)
                        x_,y_,img_,is_found,detx,dety=testdef.enhance_and_find_ring_new(self.cap)
                        if  (flag2 == 1 and is_found == 1):
                            time_start=time.time()
                            while (not stop_flag_1 and (time.time()-time_start)<2.7):
                                x_,y_,img_,is_found,detx,dety=testdef.enhance_and_find_ring_new(self.cap)
                                # print("qqqqqqqq:",abs(detx),abs(dety))
                                if abs(detx)<10 and abs(dety)<10 and is_found==1:
                                    stop_flag_1=1
                                    print("stop_flag_1:",stop_flag_1)
                                else:
                                    testdef.sendMessage5(self.ser,0,detx,dety)    
                                    time.sleep(0.05)
                                    # time.sleep(0.01)
                            if stop_flag_1==1:
                                testdef.sendMessage(self.ser,68)
                                # time.sleep(3.5)#!避免在该圆环下定位后立即判断该色到位，加一个延时，放掉这个圆环，对
                            else:
                                print("超时！")
                                time.sleep(2)
                # else:
                #     testdef.sendMessage5(self.ser,0,locked_detx,locked_dety)
                #     time.sleep(0.05)
                #     if flag_color:
                #         testdef.sendMessage(self.ser,119) # 发送到位信息，不同颜色发送不同值
                #         stop_flag=0
                #         stop_flag_1=0
                #         time.sleep(4)
                #         continue
            if i == 0:
                while not stop_flag:
                    flag2 = testdef.detectPlate_nocolor_ring(self.cap)
                    x_,y_,img_,is_found,detx,dety=testdef.enhance_and_find_ring_new(self.cap)
                    if  (flag2 == 1 and is_found == 1):
                        x_last=detx
                        y_last=dety
                        while not stop_flag:
                            stop_flag_pre = testdef.detectPlate_nocolor_ring(self.cap)
                            # stop_flag_pre, direction = testdef.detectPlate_gray(self.cap)
                            if not stop_flag_pre:
                                stop_flag = 1
                            elif stop_flag_pre:
                                print("1")
            else:
                while not stop_flag:
                    # flag2 = testdef.detectPlate_nocolor_ring(self.cap)
                    # x_,y_,img_,is_found,detx,dety=testdef.enhance_and_find_ring_new(self.cap)
                    flag2, direction = testdef.detectPlate_gray(self.cap)
                    if  (flag2 == 1):
                    # if  (flag2 == 1 and is_found == 1):
                        x_last=detx
                        y_last=dety
                        while not stop_flag:
                            # stop_flag_pre = testdef.detectPlate_nocolor_ring(self.cap)
                            stop_flag_pre, direction = testdef.detectPlate_gray(self.cap)
                            if not stop_flag_pre:
                                stop_flag = 1
                            elif stop_flag_pre:
                                print("1")
            testdef.sendMessage(self.ser,119)
            stop_flag=0
            stop_flag_1=0
            time.sleep(4)  #我看到上个颜色圆环离开-通知机械臂回身取物料-此延时用于防止在这个过程中看到颜色动与不动造成误判断
        cv2.destroyAllWindows()

    def plate_adjust_then_put_nocolor_ring_for_adjust(self, adjust_finely=0):
        """
        在位置锁定阶段检测灰度圆环 
        """
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        stop_flag=0
        stop_flag_1=0
        x_last=0
        y_last=0
        # 1. 引入新变量来存储静止时的精确偏差
        is_locked = False       # 标志位，表示我们是否已经锁定了静止的圆环
        locked_detx = 0         # 存储静止时测得的精确 detx
        locked_dety = 0         # 存储静止时测得的精确 dety
        is_stopped = False
        flag_color = 0
        i=0
        for i in range(1):

            if i == 0:
                while not is_locked:
                    # 持续检测转盘是否停止 和 圆环是否存在
                    is_stopped = testdef.detectPlate_nocolor_ring(self.cap)
                    x_,y_,img_,is_found,detx,dety=testdef.enhance_and_find_ring_new(self.cap)
                    if is_found and is_stopped:
                        print("检测到静止圆环！开始进行精确采样...")
                        # 进行多次采样以获得精确偏差
                        detx_samples = []
                        dety_samples = []
                        for _ in range(5):
                            x_,y_,img_,is_found,detx,dety=testdef.enhance_and_find_ring_new(self.cap)
                            if is_found:
                                detx_samples.append(detx)
                                dety_samples.append(dety)
                            # time.sleep(0.05)
                        if detx_samples:
                            locked_detx = round(np.mean(detx_samples))
                            locked_dety = round(np.mean(dety_samples))
                            is_locked = True # 成功锁定！退出这个循环
                            print(f"成功锁定！静止偏差为: detx={locked_detx:.2f},d dety={locked_dety:.2f}")
                        else:
                            print("采样失败，将重试...")
                # 3. 锁定成功后，进入下一个阶段：等待圆环离开
                print("等待圆环离开...")
                has_departed = False
                while not has_departed:
                    is_stopped = testdef.detectPlate_nocolor_ring(self.cap) # 持续检测停止状态
                    # x_, y_, img_, is_found, detx, dety, color = testdef.findBlockCenter_gray(self.cap)
                    if not is_stopped:
                        has_departed = True
                        print("圆环离开！")
                print("等待开始调整底盘")
                if adjust_finely == 1:
                    while not stop_flag_1:
                        flag2= testdef.detectPlate_nocolor_ring(self.cap)
                        x_,y_,img_,is_found,detx,dety=testdef.enhance_and_find_ring_new(self.cap)
                        if  (flag2 == 1 and is_found == 1):
                            time_start=time.time()
                            while (not stop_flag_1 and (time.time()-time_start)<2.7):
                                x_,y_,img_,is_found,detx,dety=testdef.enhance_and_find_ring_new(self.cap)
                                # print("qqqqqqqq:",abs(detx),abs(dety))
                                if abs(detx)<10 and abs(dety)<10 and is_found==1:
                                    stop_flag_1=1
                                    print("stop_flag_1:",stop_flag_1)
                                else:
                                    testdef.sendMessage5(self.ser,0,detx,dety)    
                                    time.sleep(0.01)
                            if stop_flag_1==1:
                                testdef.sendMessage(self.ser,68)
                                # time.sleep(3.5)#!避免在该圆环下定位后立即判断该色到位，加一个延时，放掉这个圆环，对
                            else:
                                print("超时！")
                                time.sleep(2)
                # else:
                #     testdef.sendMessage5(self.ser,0,locked_detx,locked_dety)
                #     time.sleep(0.05)
                #     if flag_color:
                #         testdef.sendMessage(self.ser,119) # 发送到位信息，不同颜色发送不同值
                #         stop_flag=0
                #         stop_flag_1=0
                #         time.sleep(4)
                #         continue

            
            stop_flag=0
            stop_flag_1=0
            time.sleep(4)  #我看到上个颜色圆环离开-通知机械臂回身取物料-此延时用于防止在这个过程中看到颜色动与不动造成误判断
        cv2.destroyAllWindows()


    def get_from_plate_check_eachtime_old(self, plate_order, run_time=3):
        """夹完在物料盘处二次检查 次次检查"""
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        stop_flag=0   #初始化转盘是否停止标志位
        i = 0   #运行轮数
        while i < run_time:
        ####依据颜色顺序循环处理3个物料
            strat_time_get = time.time()
            # print("iii:",i)
            # flagno = testdef.detectPlate(cap, 1)
            ret=self.cap.grab()
            while not stop_flag:
            # while not stop_flag and (time.time()-strat_time_get)<60: 
                print("i:",i)
                flag2 = testdef.detectPlate(self.cap,plate_order[i])
                x_,y_,img_,flag1,detx,dety = testdef.findBlockCenter(self.cap,plate_order[i])
                ####当所看对应颜色物料静止时
                if  (flag2 == 1 and flag1 == 1):
                    stop_flag = plate_order[i]  #令标志位为颜色序号
                    print("stop_flag",stop_flag)
                    testdef.sendMessage2(self.ser,detx,dety)  #给机械臂发送大致调整参数，机械臂只动一下
                    time.sleep(0.01)
                    if stop_flag == 1:  #发送到位信息，不同颜色发送不同值
                        testdef.sendMessage(self.ser,7)
                    elif stop_flag == 2:
                        testdef.sendMessage(self.ser,8)
                    elif stop_flag == 3:
                        testdef.sendMessage(self.ser,9)
                    # testdef.sendMessage(ser,stop_flag)
            # if not stop_flag:
            #     i += 1
            #     continue

            Time = time.time()
            stop_flag=0    #重置停止标志位，给下一轮使用
            flag_check=0    #初始化、重置检查物料是否夹到标志位
            time_plate_check=6.2
            #每次转到物料盘后检查
            while (not flag_check) and ((time.time()-Time)<time_plate_check):
                recv_check=testdef.receiveMessage(self.ser)
                print("recv_check",recv_check)
                if recv_check==b'check':
                    print("start checkkkkkkkkkkkkkkkkkkk")
                    flag_check=testdef.detectPlate_check(self.cap,plate_order[i])
                    print("flag_chexk:",flag_check)
                    if flag_check :  #如果夹到了继续下一个颜色
                        print("next colorrrrrrrrrrrrrrrrrrrr")
                        i=i+1
                    else:  #没夹到仍等待第一个
                        testdef.sendMessage(self.ser,3)
                        # flag_check=1
                    break
            # if not flag_check:
            #     i+=1
            ret=self.cap.grab()
            ret=self.cap.grab()
        cv2.destroyAllWindows()


    def get_from_plate_check_eachtime(self, plate_order, run_time=3, max_try=0):
        """夹完在物料盘处二次检查 次次检查 (使用嵌套循环结构优化)"""
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        i = 0
        for i in range(run_time):
            try_count = 0  # 初始化当前物料的尝试次数
            strat_time_get = time.time()
            print(f"\n--- [第 {i+1}/{run_time} 轮] 开始处理物料 {plate_order[i]}，计时开始 ---")
            while True:
                if (time.time() - strat_time_get) > 30:
                    print(f"警告：处理物料 {plate_order[i]} 总时间超过60秒，放弃并跳到下一个。")
                    break  # 跳出内层 while True 循环，外层 for 循环会继续下一个 i


                ret=self.cap.grab()
                stop_flag = 0
                while not stop_flag: 
                    print(f"i:{i}, 正在寻找...")
                    flag2 = testdef.detectPlate(self.cap,plate_order[i])
                    x_,y_,img_,flag1,detx,dety = testdef.findBlockCenter(self.cap,plate_order[i])
                    if  (flag2 == 1 and flag1 == 1):
                        stop_flag = plate_order[i]
                        print("stop_flag",stop_flag)
                        testdef.sendMessage2(self.ser,detx,dety)
                        time.sleep(0.05)
                        if stop_flag == 1: testdef.sendMessage(self.ser,7)
                        elif stop_flag == 2: testdef.sendMessage(self.ser,8)
                        elif stop_flag == 3: testdef.sendMessage(self.ser,9)
                        
                # break


                Time = time.time()
                flag_check = 0  #! 这里是否空抓的标志位初始值应该为0吗？
                time_plate_check = 6.2
                cv2.destroyAllWindows()
                ret = self.cap.grab()
                ret = self.cap.grab()
                
                while ((time.time()-Time)<time_plate_check):
                    recv_check=testdef.receiveMessage(self.ser)
                    print("recv_check",recv_check)
                    if recv_check==b'check':
                        time_start_check = time.time()
                        print("start checkkkkkkkkkkkkkkkkkkk")
                        flag_check=testdef.detectPlate_check(self.cap,plate_order[i])
                        print("flag_chexk:",flag_check)
                        break # 收到check信号就跳出检查等待

                try_count += 1 # 无论成功失败，都算一次尝试

                if flag_check:  
                    print("next colorrrrrrrrrrrrrrrrrrrr")
                    break # 成功了，跳出内层 while True，外层 for 循环会自动进入下一个 i
                else:  #没夹到仍等待第一个 (flag_check为False)
                    print("重试")
                    if max_try == 0 or try_count < max_try: # 如果max_try为0或当前尝试次数小于max_try，则重试
                        testdef.sendMessage(self.ser,3)
                    else:
                        print("重试次数超过最大值，放弃")
                        break

                
        ret=self.cap.grab()
        ret=self.cap.grab()
        cv2.destroyAllWindows()


    def get_from_ground_in_line(self):
        '''在一条直线三个圆环处夹取物料 用于判定位置和颜色对应关系'''
        # while not self.cap.isOpened():
        #     print("Not open colorcap")
        # if not self.cap.isOpened():
        #     print("Not open colorcap")
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        ret=self.cap.grab()
        get_order_blank=[]
        #开始计时
        Time_l=time.time()
        time_l=2
        line_flag = 0
        #调整车身姿态直到直线到位或超时
        while (not line_flag and (time.time()-Time_l)<time_l):
        # while (not line_flag ):
            theta,line_flag=testdef.detectLine(self.cap)
            if line_flag ==0:
                testdef.sendMessage5(self.ser,theta,0,0)
                print("main li de theta:",theta)
        print("line_flag:",line_flag)
        testdef.sendMessage(self.ser,39)
        time.sleep(0.01)
        testdef.sendMessage(self.ser,40)
        time.sleep(0.01)
        testdef.sendMessage(self.ser,68)

        line_flag=0        
        while True:
            recv_first=testdef.receiveMessage(self.ser)
            print("recv_first",recv_first)
            if recv_first==b'nearground':
                break
        color_2=0
        flag1=0
        Time_xy=time.time()
        time_xy=3
        while (not flag1 and (time.time()-Time_xy)<time_xy):
            x_,y_,img_,flag1,detx,dety,color_2= testdef.findBlockCenter_acquaint_color(self.cap)
            if  (flag1 == 0):
                testdef.sendMessage5(self.ser,0,detx,dety)
        testdef.sendMessage(self.ser,68)
        ret = self.cap.grab()
        flag1=0
        while True:
            recv_first=testdef.receiveMessage(self.ser)
            print("recv_first",recv_first)
            if recv_first==b'nearground':
                break

        color_1=0
        for i in range(2):
            x_,y_,img_,flag2,detx,dety,color= testdef.findBlockCenter_acquaint_color(self.cap)
        ret = self.cap.grab()
        x_,y_,img_,flag2,detx,dety,color_1= testdef.findBlockCenter_acquaint_color(self.cap)
        get_order_blank.append(color_1)

        get_order_blank.append(color_2)
        color_3=6-color_1-color_2
        get_order_blank.append(color_3)

        testdef.sendMessage6(self.ser,get_order_blank)

        get_order_blank=[]


    def get_from_ground_in_line_for_test(self):
        '''在一条直线三个圆环处夹取物料 用于判定位置和颜色对应关系'''
        # while not self.cap.isOpened():
        #     print("Not open colorcap")
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        ret=self.cap.grab()
        get_order_blank=[]
        #开始计时
        Time_l=time.time()
        time_l=2
        line_flag = 0
        #调整车身姿态直到直线到位或超时
        while (not line_flag and (time.time()-Time_l)<time_l):
        # while (not line_flag ):
            theta,line_flag=testdef.detectLine(self.cap)
            if line_flag ==0:
                testdef.sendMessage5(self.ser,theta,0,0)
                print("main li de theta:",theta)
        print("line_flag:",line_flag)
        testdef.sendMessage(self.ser,39)
        time.sleep(0.01)
        testdef.sendMessage(self.ser,40)
        time.sleep(0.01)
        testdef.sendMessage(self.ser,68)

        line_flag=0        
        while True:
            recv_first=testdef.receiveMessage(self.ser)
            print("recv_first",recv_first)
            if recv_first==b'nearground':
                break
        color_2=0
        flag1=0
        Time_xy=time.time()
        time_xy=3
        while (not flag1 and (time.time()-Time_xy)<time_xy):
            x_,y_,img_,flag1,detx,dety,color_2= testdef.findBlockCenter_acquaint_color(self.cap)
            if  (flag1 == 0):
                testdef.sendMessage5(self.ser,0,detx,dety)
        testdef.sendMessage(self.ser,68)
        ret = self.cap.grab()
        # flag1=0
        # while True:
        #     recv_first=testdef.receiveMessage(self.ser)
        #     print("recv_first",recv_first)
        #     if recv_first==b'nearground':
        #         break

        # color_1=0
        # for i in range(2):
        #     x_,y_,img_,flag2,detx,dety,color= testdef.findBlockCenter_acquaint_color(self.cap)
        # ret = self.cap.grab()
        # x_,y_,img_,flag2,detx,dety,color_1= testdef.findBlockCenter_acquaint_color(self.cap)
        # get_order_blank.append(color_1)

        # get_order_blank.append(color_2)
        # color_3=6-color_1-color_2
        # get_order_blank.append(color_3)

        # testdef.sendMessage6(self.ser,get_order_blank)

        # get_order_blank=[]
    

    def xi_positioning_update(self, circle_order, timeout_xi=2, run_time=3):
        """
        细调圆环位置（颜色定位和灰度定位）
        # 更新偏差值
        """
        if not self.check_camera(self.cap,"上部摄像头"):
            self.init_camera_up()
        for i in range(run_time):
            ret=self.cap.grab()
            testdef.g_prev_smoothed_circle=None
            print("iiiiiiiiiiiiii:",i,"color:",circle_order[i])
            ####接收到爪子下降消息再开始进入细调
            recv_first=None
            recv_update=None
            while True:
                recv_update=testdef.receiveMessage(self.ser)
                print("recv_update",recv_update)
                if recv_update==b'update':#??????????????????
                    # time_update=time.time()
                    testdef.updateCorrectxy(self.cap,circle_order[i])
                    # print("update center time:",time.time()-time_update)
                    break
            while True:
                recv_first=testdef.receiveMessage(self.ser)
                # if recv_first != None:
                #     print("recv_first",recv_first)
                if recv_first==b'nearground':
                    print("recv_first",recv_first)
                    break
            ####细调开始计时
            # for j in range(3):
            #     q=cap.grab()
            Time3=time.time()
            ####细调第一步 颜色定五环
            move_flag_color_1=0 
            while (not move_flag_color_1 and (time.time()-Time3)<timeout_xi):
            # while (not move_flag_color_1):
            # while((time.time()-Time3)<timeout_xi):
                timee=time.time()
                print("cccccccccccc")
                # q=cap.grab()
                x_,y_,img_,move_flag_color_1,detx_p,dety_p = testdef.circlePut_color(self.cap,circle_order[i])
                if move_flag_color_1==0:    #flag=1后那次数据不需发送
                    testdef.sendMessage2(self.ser,detx_p,dety_p)
                    print("cutiao time:",time.time()-timee)
            print("xitiao11 okokokokokokokokok")
            move_flag_color_1=0   

            # move_flag_color_2 = 0
            # stable_count = 0
            # required_stable_frames = 3  # 需要连续稳定的帧数
            # flag_mid = 0
            # while (stable_count < required_stable_frames) and (time.time()-Time3)<timeout_xi:
            #     timeee=time.time()
            #     print("xxxxxxxx")
            #     timeee = time.time()
            #     detx, dety, flag_mid  = testdef.circlePut1(self.cap)
                
            #     if flag_mid:
            #         stable_count += 1
            #     else:
            #         stable_count = 0  # 重置计数器
            #         testdef.sendMessage2(self.ser, detx, dety)
            #     if stable_count == required_stable_frames:
            #         move_flag_color_2 = 1
                
            #     print("xitiao time:", time.time()-timeee)            

            #细调第二步 灰度定中心（第一版-无到位后二次检测
            testdef.reset_circle_put_state()
            move_flag_color_2=0
            while (not move_flag_color_2 and (time.time()-Time3)<timeout_xi):
            # while (not move_flag_color_2 ):
                print("xxxxxxxx")
                timeee=time.time()
                # detx,dety,move_flag_color_2=testdef.circlePut1(self.cap)
                # detx,dety,move_flag_color_2=testdef.circlePut_hzw(self.cap)
                detx,dety,move_flag_color_2=testdef.circlePut_det(self.cap)
                if move_flag_color_2==0:
                    testdef.sendMessage2(self.ser,detx,dety)
                    print("xitiao time:",time.time()-timeee)
    
            # ###细调第二步 灰度定中心（第二版-到位后做二次检测-防止物料贴环边立即就放
            # move_flag_color_2_2=0
            # while (not move_flag_color_2_2 and (time.time()-Time3)<timeout_xi):
            # # while (not move_flag_color_2_2 ):
            #     print("xxxxxxxx")
            #     timeee=time.time()
            #     detx,dety,move_flag_color_2=testdef.circlePut1(self.cap)
            #     # timeuart=time.time()
            #     # print("xitiao time:",time.time()-timeee)
            #     # testdef.sendMessage2(ser,detx,dety)
            #     # print("xitiao time:",time.time()-timeuart)
            #     if move_flag_color_2==0:
            #         # a=1
            #         testdef.sendMessage2(self.ser,detx,dety)
            #         print("xitiao time:",time.time()-timeee)
            #     else:
            #         # detxx=0
            #         # detyy=0
            #         flagg=0
            #         time_check=2
            #         ####初次到位后再看一次是否还在中心内 若在则到位 若不在则继续调整
            #         for k in range(time_check):
            #             detx2,dety2,move_flag_color_22=testdef.circlePut1(self.cap)
            #             testdef.sendMessage2(self.ser,detx2,dety2)
            #             # print("double check")
            #             # detxx+=detx
            #             # detyy+=dety
            #             flagg+=move_flag_color_22
            #             print("double check    flagg:",flagg)
            #         if flagg==time_check:
            #             move_flag_color_2_2=1
            #             break
            #         move_flag_color_2_2=1
            #         break
            print("xitiao22 okokokokokokokokok  move_flag_color_2:",move_flag_color_2)
            ####发送到位信息 根据颜色发送不同到位信息
            if circle_order[i] == 1:
                testdef.sendMessage(self.ser,57)
            elif circle_order[i] == 2:
                testdef.sendMessage(self.ser,64)
            elif circle_order[i] == 3:
                testdef.sendMessage(self.ser,65)
            time.sleep(0.01)
            move_flag_color_2=0
            # i = i+1  #继续下一个颜色
            # cv2.destroyAllWindows()
        ret=self.cap.grab()
        testdef.defaltCorrectxy()
        self.recv=b'st'  #完成功能后进入空循环


    def reset_state(self):
        # print("endendendendnendnendnendne")
        cv2.destroyAllWindows()
        ####初始化侧边二维码摄像头
        self.init_camera_code()  # 初始化摄像头
        if  not self.code_cap.isOpened():
            print("faillllllllll")
        ####初始化各个变量
        self.get_order=[]
        self.put_order=[]


    
    





