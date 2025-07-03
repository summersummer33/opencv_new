import cv2
import numpy as np
import math
import time
import serial 
import threading
import testfcn  # 导入封装好的功能处理器
# import testdef
import testdef_pro as testdef

# 初始化处理器
handler = testfcn.FunctionHandler()
handler.init_camera_code()  # 初始化摄像头
handler.init_camera_up()  # 初始化摄像头
recv=''

while True:
    # 接收串口消息
    recv_mess = testdef.receiveMessage(handler.ser)
    if recv_mess != None:
        print("recv_mess:",recv_mess)
    if recv_mess != None:
        #### 根据接收到的指令更新recv
        if recv_mess in [b'AA', b'BB1', b'BB2', b'CC12', b'CC3', b'CC4',  b'EE', 
                         b'FF', b'GG', b'HH', b'II', b'JJ', b'KK', b'LL', b'MM', b'NN'
                         , b'OO', b'PP', b'QQ',
                         b'st', b'end',
                         b'DD']:
            recv=recv_mess


    #############################################################################################
    ########################初赛正常流程使用代码（轻易不要改动！！！）###############################
    #############################################################################################

####识别二维码、条形码
    if recv == b'AA':
        handler.get_code()
        handler.init_camera_up()  # 初始化摄像头
        recv = b'st'

####识别转盘 夹取物料（正常流程
    #第一顺序
    elif recv == b'BB1':
        handler.get_from_plate(handler.get_order)
        recv = b'st'

    #第二顺序
    elif recv == b'BB2':
        handler.get_from_plate(handler.put_order)
        recv = b'st'

####识别圆环 放置物料
    #粗调+第一顺序
    elif recv == b'CC12':
        handler.cu_positioning()
        handler.xi_positioning(handler.get_order)
        recv = b'st'

    #粗调+第二顺序
    elif recv == b'CC3':
        handler.cu_positioning()
        handler.xi_positioning(handler.put_order)
        recv = b'st'

    #粗调
    elif recv == b'CC4':
        # handler.cu_positioning(50,100)
        handler.cu_positioning()
        recv = b'st'

####识别直线 在转盘旁调整车身
    elif recv == b'EE':
        handler.adjust_line_gray_yellow()
        recv = b'st'


############测试:粗调时先直线后xy位置
    elif recv == b'QQ':
        handler.cu_positioning_test()
        recv = b'st'

############测试：往转盘上放，先定位好，然后每个颜色来了就放
############有个问题：：：该代码原来是适配转盘上是小的色块的，判断色块停止时，右边有个长方形，圆没完全转出来，长方形也不动，会在圆没有转到位时就误判断为静止，
############但是呢，由于我们回去抓的很慢，这可以让我们提前转回去，效果更好。了吗？
############解决办法：加面积限定值（适配圆环）/限制到视野中间时再判断
    elif recv == b'HH':
        handler.plate_adjust_then_put(handler.get_order)
        recv = b'st'


    elif recv == b'LL':
        handler.plate_adjust_then_put_pre_color(handler.get_order)
        recv = b'st'

    #############################################################################################
    ##############################决赛功能备用代码################################################
    #############################################################################################


    #############################################################################################
    ####################################模拟赛使用################################################
    #############################################################################################



    #############################################################################################
    #################################空循环及清零部分#############################################
    #############################################################################################

####待机状态
    elif recv == b'st':
        pass

####全局标志位清零 可直接开始第二轮
    elif recv == b'end':
        handler.reset_state()  # 复位状态


    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

# 退出时清理资源
handler.cleanup()

