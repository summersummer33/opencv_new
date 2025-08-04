import cv2
import numpy as np
import math
import time
import serial 
import threading
import testfcn  # 导入封装好的功能处理器
# import testdef
import testdef_pro as testdef
import logging
import os

# # 获取当前时间戳字符串
# timestamp = time.strftime("%Y%m%d_%H%M%S")
# log_dir = 'log'
# if not os.path.exists(log_dir):
#     os.makedirs(log_dir)
# log_filename = os.path.join(log_dir, f"{timestamp}.txt")

# # 创建logger
# logger = logging.getLogger()
# logger.setLevel(print)

# # 文件Handler（带时间戳）
# file_handler = logging.FileHandler(log_filename, mode='a', encoding='utf-8')
# file_handler.setLevel(print)
# file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
# file_handler.setFormatter(file_formatter)

# # 控制台Handler（只显示内容，不显示日志级别）
# console_handler = logging.StreamHandler()
# console_handler.setLevel(print)
# console_formatter = logging.Formatter('%(message)s')  # 只显示内容
# console_handler.setFormatter(console_formatter)

# # 先移除所有旧Handler，防止重复
# if logger.hasHandlers():
#     logger.handlers.clear()

# # 添加Handler到logger
# logger.addHandler(file_handler)
# logger.addHandler(console_handler)


limit_cu_circle = 4
limit_cu_line = 0.5


def main():
    # 初始化处理器
    handler = testfcn.FunctionHandler()
    handler.init_camera_code()  # 初始化摄像头
    handler.init_camera_up()  # 初始化摄像头
    recv=''

    try:
        while True:
            # 接收串口消息
            recv_mess = testdef.receiveMessage(handler.ser)
            if recv_mess != None:
                print("recv_mess:",recv_mess)
            if recv_mess != None:
                #### 根据接收到的指令更新recv
                if recv_mess in [b'AA', b'BB1', b'BB2', b'CC12', b'CC3', b'CC4', b'CC5', b'EE', 
                                b'FF', b'GG', b'HH', b'II', b'JJ', b'KK12', b'KK3', b'LL1', b'LL2', b'MM', b'NN1', b'NN2',
                                b'OO', b'PP', b'QQ1', b'QQ2', b'RR',
                                b'st', b'end',
                                b'DD', b'II2']:
                    recv=recv_mess


            #############################################################################################
            ########################初赛正常流程使用代码（轻易不要改动！！！）###############################
            #############################################################################################

        ####识别二维码、条形码
            if recv == b'AA':
                # print("helloworld")
                start_time = time.time()
                #自己处理，用串口发送"ok\n"
                # data = 111
                # testdef.sendMessage4(handler.ser,data)
                time.sleep(0.01)
                handler.get_code()
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
                handler.cu_positioning(limit_circle=limit_cu_circle, limit_line=limit_cu_line)
                handler.xi_positioning(handler.get_order,run_time=3)
                recv = b'st'

            #粗调+第二顺序
            elif recv == b'CC3':
                handler.cu_positioning(limit_circle=limit_cu_circle, limit_line=limit_cu_line)
                handler.xi_positioning(handler.put_order,run_time=3)
                recv = b'st'

            #粗调-码垛-省赛计时使用
            elif recv == b'CC4':
                run_time = time.time()-start_time
                print("run_time:",run_time)
                if run_time < 155:
                    handler.cu_positioning(limit_circle=limit_cu_circle, limit_line=limit_cu_line)
                elif run_time > 155 and run_time < 160:
                    handler.cu_positioning(limit_circle=6, limit_line=1)
                elif run_time > 168:
                    handler.cu_positioning(limit_circle=200, limit_line=10)
                else: 
                    handler.cu_positioning(limit_circle=15, limit_line=3)

                # handler.cu_positioning()
                recv = b'st'

            #粗调-测试路径
            elif recv == b'CC5':
                handler.cu_positioning(limit_circle=limit_cu_circle, limit_line=limit_cu_line)
                recv = b'st'

        ####识别直线 在转盘旁调整车身
            elif recv == b'EE':
                handler.adjust_line_gray_yellow()
                recv = b'st'




            #############################################################################################
            ##############################决赛功能备用代码################################################
            #############################################################################################

        ############测试：往转盘上放，先定位好，然后每个颜色来了就放
        ############有个问题：：：该代码原来是适配转盘上是小的色块的，判断色块停止时，右边有个长方形，圆没完全转出来，长方形也不动，会在圆没有转到位时就误判断为静止，
        ############但是呢，由于我们回去抓的很慢，这可以让我们提前转回去，效果更好。了吗？
        ############解决办法：加面积限定值（适配圆环）/限制到视野中间时再判断

        ####识别转盘 放置物料 当次颜色放置
            elif recv == b'HH':
                handler.plate_adjust_then_put(handler.get_order,adjust_finely=1)
                recv = b'st'

        ####识别转盘 放置物料 前一个颜色放置
            elif recv == b'LL1':
                handler.plate_adjust_then_put_pre_color_pro(handler.get_order,adjust_finely=0)
                recv = b'st'

            elif recv == b'LL2':
                handler.plate_adjust_then_put_pre_color_pro(handler.put_order,adjust_finely=0)
                recv = b'st'

        ############测试:识别转盘 夹取物料（回到物料盘 次次检查
            elif recv==b'NN1':
                handler.get_from_plate_check_eachtime(handler.get_order, run_time=3, max_try=2)
                recv = b'st'

            elif recv==b'NN2':
                handler.get_from_plate_check_eachtime(handler.put_order, run_time=3, max_try=2)
                recv = b'st'

        ####在一条直线三个圆环处夹取物料 用于判定位置和颜色对应关系
            elif recv==b'II':
                handler.get_from_ground_in_line()
                recv = b'st'

            #用于调整路径
            elif recv==b'II2':
                handler.get_from_ground_in_line_for_test()
                recv = b'st'

        ####识别圆环 放置物料（物料夹不紧时，更新偏差值
            elif recv==b'KK12':
                handler.cu_positioning(limit_circle=limit_cu_circle, limit_line=limit_cu_line)
                handler.xi_positioning_update(handler.get_order, run_time=3)
                recv = b'st'

            elif recv==b'KK3':
                handler.cu_positioning(limit_circle=limit_cu_circle, limit_line=limit_cu_line)
                handler.xi_positioning_update(handler.put_order, run_time=3)
                recv = b'st'

            #############################################################################################
            ####################################模拟赛使用################################################
            #############################################################################################
        ####识别转盘 放置物料（黑色圆环）
            elif recv==b'OO':
                handler.plate_adjust_then_put_nocolor_ring(adjust_finely=1)
                recv = b'st'

            elif recv == b'PP':
                handler.plate_adjust_then_put_nocolor_ring_for_adjust(adjust_finely=1)
                recv = b'st'

        ####识别转盘 放置物料 前一个颜色放置 调整底盘
            elif recv == b'QQ1':
                handler.plate_adjust_then_put_pre_color_pro_move_car(handler.get_order, adjust_finely=1)
                recv = b'st'

            elif recv == b'QQ2':
                handler.plate_adjust_then_put_pre_color_pro_move_car(handler.put_order, adjust_finely=1)
                recv = b'st'

        ####识别转盘 放置物料 前一个颜色放置 精调机械臂 不会多放走
        ####一定要降下来稳后再发RR！！！
            elif recv == b'RR':
                handler.plate_adjust_then_put_pre_color_faster(handler.get_order)
                recv = b'st'

            #############################################################################################
            #################################空循环及清零部分#############################################
            #############################################################################################

        ####待机状态
            elif recv == b'st':
                pass

        ####全局标志位清零 可直接开始第二轮
            elif recv == b'end':
                handler.reset_state()  # 复位状态
                recv = b'st'


            if cv2.waitKey(20) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n程序被用户中断")
        print("程序被用户中断")
    finally:
        # 确保在任何情况下都能清理资源
        print("正在清理资源...")
        print("正在清理资源...")
        if handler.cap is not None:
            handler.cap.release()
        if handler.code_cap is not None:
            handler.code_cap.release()
        cv2.destroyAllWindows()
        handler.cleanup()
        print("资源清理完成")
        print("资源清理完成")


if __name__ == "__main__":
    main()

