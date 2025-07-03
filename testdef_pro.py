import os
from datetime import datetime
import cv2
import numpy as np
import math
import time
import serial 
from pyzbar.pyzbar import decode  


#颜色hsv
#绿色 hmin值太小会看见黄色 太大会看不见浅绿
dim_red_min =   [  0, 60 ,60]
dim_red_max =   [ 12,203, 255]
dim_green_min = [32,48,54]# 30 48 54   61/48/54 61 taida    #yuanhuan   nengkanqianlv
dim_green_max = [78,234,255]#78,234,255
dim_green_min1 = [40,48,54]# 30 48 54   61/48/54 61 taida    #zhuanpan   fanghuangse     40 48 54
dim_green_max1 = [78,234,255]#78,234,255
dim_blue_min =  [82,70,0]#100 60 80
dim_blue_max =  [120,255,255]#124 230 255
dim_red_min1 =   [  160, 50 ,50]
dim_red_max1 =   [ 180,255, 255]

#校正直线时分割黄、灰
dim_gray_min=[95,0,0] 
dim_gray_max=[180,255,255]

#转盘夹取物料 物料面积大小系数  0.039
block_area=0.039
#粗调时0.016
cutiao_center_circle=0.0097

#放的偏右了x值就+，偏下了y值就-

#new paw
#cedingzhi 30 13
#cedingzhi 39 -9
#42  -9
#粗调时高度偏差值(findcontours)
correct_x=40
correct_y=-7

#new paw
#cedingzhi 33 9
#celiangzhi 42
#houghcircles 43 7
#20thin---y=16
#用了很久  43  7 
#37  6
#45  7
#细调时高度的偏差值(houghcircles)
correct_x_hough=45
correct_y_hough=11
#存储默认值
correct_x_hough_default=correct_x_hough
correct_y_hough_default=correct_y_hough

#摄像头分辨率
frameWidth = 1280
frameHeight = 720

#houghcircles半径限制值/第5环实线
# 640/480-----140/155
# 800/600-----186/197
# 1280/720----218/23
houghradius_min=187
houghradius_max=212

#houghcircles半径限制值/第6环虚线
# 640/480-----
# 800/600-----
# 1280/720----
houghradius_min_6th=209
houghradius_max_6th=233

#滤波系数
g_prev_smoothed_circle = None
g_smooth_factor = 0.25
g_distance_threshold_factor = 1.0

# #粗调到位阈值（圆+直线）
# #前后左右
# limit_position_circle=4
# #直线斜率
# limit_position_line=0.5  #所有直线斜率

# #细调到位阈值（圆环-放下物料）
# limit_ring_1st=50
# limit_ring_2nd=3

###########################################################################################
###########################################################################################
###########################################################################################



#粗调 直线+圆（findContours 看中间圆环-绿色
def together_line_circle1(cap, limit_position_circle=4, limit_position_line=0.5):  #粗调 直线+圆（findContours 看中间圆环-绿色
    ret,frame = cap.read()

    res1 = frame.copy()
    h, w = res1.shape[:2]

    #####################line图像处理#################################
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   #ת Ҷ ͼ
    equalized = cv2.equalizeHist(gray)
    # cv2.imshow("junheng",equalized)
    # ret, thresh = cv2.threshold(equalized, 120, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    opened = cv2.morphologyEx(equalized, cv2.MORPH_OPEN, kernel)#      
    closed1 = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    # closed = cv2.morphologyEx(closed1, cv2.MORPH_CLOSE, kernel)
    blurred = cv2.GaussianBlur(closed1, (9, 9), 2)
    edges = cv2.Canny(blurred, 50, 150)
    # cv2.imshow("edges",edges)

    #####################circle图像处理#################################
    blurred_c = cv2.GaussianBlur(equalized, (9, 9), 2)
    # edges = cv2.Canny(blurred, 50, 150)
    edges1 = cv2.Canny(blurred_c, 50, 150)
    cv2.imshow("edges1",edges1)

    red_min   =  np.array(dim_red_min)
    red_max   =  np.array(dim_red_max)
    green_min =  np.array(dim_green_min)
    green_max =  np.array(dim_green_max)
    blue_min  =  np.array(dim_blue_min)   
    blue_max  =  np.array(dim_blue_max)  
    red_min1   = np.array(dim_red_min1)  
    red_max1   = np.array(dim_red_max1)
    hsv = cv2.cvtColor(res1, cv2.COLOR_BGR2HSV)
    mask12 = cv2.inRange(hsv,   red_min,   red_max)
    mask11 = cv2.inRange(hsv,   red_min1,   red_max1)
    mask2 = cv2.inRange(hsv, green_min, green_max)
    mask3 = cv2.inRange(hsv,  blue_min,  blue_max)
    mask1 = cv2.add(mask12,mask11)
    cv2.imshow("green",mask2)

    ####不看颜色框里线，避免被物料直边干扰
    #red
    x_r=0
    y_r=0
    w_r=0
    h_r=0
    contours_red, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    large_contours_red = []
    for contour in contours_red:
        area = cv2.contourArea(contour)
        if area > 100 :
            large_contours_red.append(contour)
    if large_contours_red:
        merged_contour_r = np.vstack(large_contours_red)
        x_r, y_r, w_r, h_r = cv2.boundingRect(merged_contour_r)
        edges[y_r:y_r + h_r, x_r:x_r + w_r] = 0
        cv2.rectangle(res1, (x_r, y_r), (x_r + w_r, y_r + h_r), (0, 0, 255), 2)

    #blue
    x_b=0
    y_b=0
    w_b=0
    h_b=0
    contours_blue, _ = cv2.findContours(mask3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    large_contours_blue = []
    for contour in contours_blue:
        area = cv2.contourArea(contour)
        if area > 100 :
            large_contours_blue.append(contour)
    if large_contours_blue:
        merged_contour_b = np.vstack(large_contours_blue)
        x_b, y_b, w_b, h_b = cv2.boundingRect(merged_contour_b)
        edges[y_b:y_b + h_b, x_b:x_b + w_b] = 0
        cv2.rectangle(res1, (x_b, y_b), (x_b + w_b, y_b + h_b), (255, 0, 0), 2)


    x_g=0
    y_g=0
    w_g=0
    h_g=0
    contours_green, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    large_contours_green = []
    for contour in contours_green:
        area = cv2.contourArea(contour)
        if area > 100 :
            large_contours_green.append(contour)
    if large_contours_green:
        merged_contour_g = np.vstack(large_contours_green)
        x_g, y_g, w_g, h_g = cv2.boundingRect(merged_contour_g)
        edges[y_g:y_g + h_g, x_g:x_g + w_g] = 0
        cv2.rectangle(res1, (x_g, y_g), (x_g + w_g, y_g + h_g), (0, 255, 0), 2)
 
    x_g_new = max(0, x_g - 50)
    y_g_new = max(0, y_g - 50) 
    w_g_new = min(frameWidth, x_g + w_g + 50) - x_g_new 
    h_g_new = min(frameHeight, y_g + h_g + 50) - y_g_new  
    cv2.rectangle(res1, (x_g_new, y_g_new), (x_g_new + w_g_new, y_g_new + h_g_new), (255, 255, 0), 2)

    img_green = edges1[y_g_new:(y_g_new + h_g_new), x_g_new:(x_g_new + w_g_new)]

    # img_green=edges1[y_g:(y_g+h_g),x_g:(x_g+w_g)]


    # total_mask = cv2.bitwise_or(mask1, cv2.bitwise_or(mask2, mask3))
    # cv2.imshow("mask",total_mask)
    # edges[total_mask > 0] = [0]
    cv2.imshow("edges",edges)

    #################识别直线，不看颜色框里线，避免被物料直边干扰
    lines = cv2.HoughLines(edges,1,np.pi/180,threshold =150)
    cnt = 0
    sumTheta = 0
    averageTheta = 0
    # global last_theta
    last_theta = 0
    if lines is not None:
        for line in lines:
            rho,theta = line[0]
            
            if ((np.abs(theta)>=1.1) & (np.abs(theta)<=2.2)):
                cnt = cnt + 1
                sumTheta = sumTheta + theta / 5.0
                # sumTheta = sumTheta + theta
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(res1,(x1,y1),(x2,y2),(0,0,255),2)
    if not (cnt == 0):
        averageTheta = 5.0 * sumTheta / cnt 
        # averageTheta = sumTheta / cnt
        last_theta =  averageTheta
    else :
        averageTheta = last_theta
    # print(averageTheta)
    averageTheta180=np.degrees(averageTheta)
    finaltheta=90-averageTheta180
    print("hudu:",averageTheta,"   jiaodu:",averageTheta180,"    jiajiao;",finaltheta)
    # cv2.imshow("line",frame)
    line_flag=0
    if abs(finaltheta)<limit_position_line:
        line_flag=1
    finaltheta=int(round(finaltheta))
    if (finaltheta==90 ):
        finaltheta=0

    #####################识别圆环
    contours_g, _ = cv2.findContours(img_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_circle_g = None
    largest_area_g = 0

    detx1=0
    dety1=0
    stop_flag = 0
    x_incolor=0
    y_incolor=0
    for contour in contours_g:
        area = cv2.contourArea(contour)
        if area > largest_area_g:
            largest_area_g = area
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
            if len(approx) > 7:  
                largest_circle_g = approx
    if largest_circle_g is not None and largest_area_g > cutiao_center_circle*w*h:    #0.016
        (x, y), radius = cv2.minEnclosingCircle(largest_circle_g)
        x=x+x_g_new
        y=y+y_g_new
        center = (int(x), int(y))
        radius = int(radius)
        cv2.circle(res1, center, 2, (0, 0, 255), 3)
        cv2.circle(res1, center, radius, (0, 255, 0), 2)  
        # center_text = f"({center[0]}, {center[1]}), radius: {radius}"
        # text_position = (center[0] + 10, center[1] - 10)
        # area_text = f"Area: {largest_area_g}"
        # cv2.putText(res1, center_text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        # cv2.putText(res1, area_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        x_incolor=x-correct_x-w/2
        y_incolor=h/2-y-correct_y
        detx1=int(round(x_incolor))
        dety1=int(round(y_incolor))
        print("cccccccccccccccccccccccccccccccccccccc")
        global flag_in
        flag_in=1
    else:
        cv2.putText(res1, 'No circle found', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        detx1=int(round(x_g_new + w_g_new/2 -w/2 -correct_x))
        dety1=int(round(h/2 - y_g_new - h_g_new/2 -correct_y))
        print("nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn")


    print("x_incolor:",x_incolor,"y_incolor:",y_incolor)
    print("detx1:",detx1,"dety1:",dety1)
    # det=4
    if abs(x_incolor)<limit_position_circle and abs(y_incolor)<limit_position_circle:
        if x_incolor == 0 and y_incolor==0:
            stop_flag=0
        else:
        # if abs(x_incolor)!= 0 or abs(y_incolor)!= 0:
            stop_flag = 1
            print("11111111111111111")
    cv2.imshow("res1",res1)
    frame=None
    cv2.waitKey(1)
    return finaltheta,line_flag,detx1,dety1,stop_flag

def circlePut_color(color_cap,color_number):  #细调第一步 颜色画框确保第五环能被看见
    '''细调第一步 颜色画框确保第五环能被看见'''

    ret,frame = color_cap.read()
    if not ret:
        print("无法读取摄像头图像")
        return None, None, None, None, None, None

    # 调用预处理函数
    closed, color_number = preprocess_image(frame, color_number)

    h, w = frame.shape[:2]
    x_center = 0
    y_center = 0
    detx_p=10000
    dety_p=10000
    flag_color_1 = 0
    cv2.imshow("closed",closed)

    #blue
    x_b=0
    y_b=0
    w_b=0
    h_b=0
    contours_, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    large_contours_ = []
    for contour in contours_:
        area = cv2.contourArea(contour)
        if area > 400 :
            large_contours_.append(contour)
            x_b1, y_b1, w_b1, h_b1 = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x_b1, y_b1), (x_b1 + w_b1, y_b1 + h_b1), (0, 0, 255), 2)
    if large_contours_:
        merged_contour_b = np.vstack(large_contours_)
        x_b, y_b, w_b, h_b = cv2.boundingRect(merged_contour_b)
        cv2.rectangle(frame, (x_b, y_b), (x_b + w_b, y_b + h_b), (255, 0, 0), 2)
        a = x_b + w_b / 2
        b = y_b + h_b / 2
        detx_p = int(a - w/2 - correct_x_hough)
        dety_p = int(h/2 - correct_y_hough - b)
        
    if abs(detx_p)<50 and abs(dety_p)<50:
        flag_color_1 =1
    if (detx_p==10000) and (dety_p==10000):
        detx_p=0
        dety_p=0
    if (abs(detx_p)>250) :
        detx_p=0
        dety_p=0
    cv2.imshow("frame", frame)
    print("detx_p:",detx_p,"dety_p:",dety_p,"flag_color_1:",flag_color_1)
    cv2.waitKey(1)
    return x_center/ w,y_center/h,frame,flag_color_1,detx_p,dety_p

def circlePut1(cap):  # 细调第二步 灰度houghcircles识别圆心
    '''细调第二步 灰度houghcircles识别圆心'''
    # success, frame = cap.read() # 读取多次是为了确保获取最新帧，但一次通常也够
    success, frame = cap.read()
    if not success or frame is None:
        print("Failed to read frame in circlePut1")
        return 0, 0, 0 # 返回默认值表示失败

    src1 = frame.copy()
    res1 = src1.copy()
    h, w = res1.shape[:2]

    # 转换到灰度图并进行伽马校正增强对比度
    gray = cv2.cvtColor(res1, cv2.COLOR_BGR2GRAY)
    gamma = 0.5
    invgamma = 1 / gamma
    gamma_image = np.array(np.power((gray / 255.0), invgamma) * 255, dtype=np.uint8)
    # cv2.imshow("gamma", gamma_image)

    # 高斯模糊
    blurred1 = cv2.GaussianBlur(gamma_image, (9, 9), 2)
    # 忽略图像边缘区域，防止误识别
    blurred1[:, :200] = 0
    blurred1[:, 1160:1280] = 0 # 假设图像宽度为1280，这里根据你的实际分辨率调整
    # cv2.imshow("blurred1", blurred1)

    # 使用HoughCircles检测圆
    # param1: Canny边缘检测的高阈值，低阈值是其一半
    # param2: 累加器阈值，越小表示检测到的圆越多
    circles = cv2.HoughCircles(blurred1, cv2.HOUGH_GRADIENT, 0.7, 70,
                               param1=100, param2=83, minRadius=houghradius_min, maxRadius=houghradius_max)

    largest_circle_raw = None  # 存储本次检测到的最大圆的原始数据
    # if circles is not None:
    #     circles = np.uint16(np.around(circles))
    #     # 找到本次检测到的最大圆
    #     for i in circles[0, :]:
    #         if largest_circle_raw is None or i[2] > largest_circle_raw[2]:
    #             largest_circle_raw = tuple(i) # 转换为元组以便于后续处理
    if circles is not None:
        circles_rounded = np.round(circles[0]).astype(np.int32)
        
        valid_circles = []
        for circle in circles_rounded:
            x, y, r = circle
            if (0 <= x < w and 0 <= y < h and 
                houghradius_min <= r <= houghradius_max):
                valid_circles.append((x, y, r))
        
        if valid_circles:
            largest_circle_raw = max(valid_circles, key=lambda c: c[2])

    # =====================================================================
    # 应用时间滤波（平滑处理）
    global g_prev_smoothed_circle, g_smooth_factor, g_distance_threshold_factor

    # 用于显示和计算的圆环数据
    circle_to_use = None

    if largest_circle_raw is not None:
        curr_x, curr_y, curr_r = largest_circle_raw

        if g_prev_smoothed_circle is None:
            # 如果是第一次检测到圆环，或者之前没有历史数据，直接使用当前检测结果
            g_prev_smoothed_circle = largest_circle_raw
        else:
            prev_x, prev_y, prev_r = g_prev_smoothed_circle

            # 计算当前检测到的圆环与上一次平滑圆环之间的距离
            distance = np.sqrt((curr_x - prev_x)**2 + (curr_y - prev_y)**2)

            # 设定一个动态的距离阈值，基于当前圆环或上一次平滑圆环的半径
            # 如果距离在阈值范围内，则进行平滑
            # 这里的 max(curr_r, prev_r) 确保阈值是基于较大的半径，更容忍跳变
            threshold = max(curr_r, prev_r) * g_distance_threshold_factor

            if distance < threshold:
                # 应用指数平滑
                smoothed_x = int(round(g_smooth_factor * prev_x + (1 - g_smooth_factor) * curr_x))
                smoothed_y = int(round(g_smooth_factor * prev_y + (1 - g_smooth_factor) * curr_y))
                smoothed_r = int(round(g_smooth_factor * prev_r + (1 - g_smooth_factor) * curr_r))
                g_prev_smoothed_circle = (smoothed_x, smoothed_y, smoothed_r)
            else:
                # 如果距离过大，表示圆环可能发生了较大跳变或重新出现
                # 此时重置平滑器，直接使用当前检测到的圆环数据作为新的起始点
                print(f"Warning: Large jump detected for circle. Distance: {distance:.2f}, Threshold: {threshold:.2f}. Resetting filter.")
                g_prev_smoothed_circle = largest_circle_raw
        
        circle_to_use = g_prev_smoothed_circle
    else:
        # 如果当前帧未检测到圆环，重置历史平滑数据
        g_prev_smoothed_circle = None

    # =====================================================================

    flag = 0  # 初始标志位，表示未检测到有效圆环
    detx = 0  # x方向偏差
    dety = 0  # y方向偏差
    detx1 = 0 # 整数化后的x偏差
    dety1 = 0 # 整数化后的y偏差
    radius = 0 # 圆环半径
    stop_flag = 0  # 停止标志位

    if circle_to_use is not None:
        flag = 1 # 检测到有效圆环，设置标志位
        
        # 绘制平滑后的圆心和圆
        cv2.circle(res1, (circle_to_use[0], circle_to_use[1]), circle_to_use[2], (0, 0, 255), 2)
        cv2.circle(res1, (circle_to_use[0], circle_to_use[1]), 2, (0, 0, 255), 3)
        
        radius = circle_to_use[2]
        radius_text = f"Radius: {radius}"
        radius_position = (circle_to_use[0] + 10, circle_to_use[1] + 20)
        cv2.putText(res1, radius_text, radius_position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # 计算偏差，使用平滑后的圆环坐标
        detx = circle_to_use[0] - w/2 - correct_x_hough
        dety = h/2 - circle_to_use[1] - correct_y_hough
        detx1 = int(round(detx))
        dety1 = int(round(dety))
        print("detx=", detx, "dety=", dety)
    else:
        # 如果未检测到圆环（或平滑后为空），显示提示信息
        cv2.putText(res1, 'no circle detected', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        # 此时detx1, dety1, radius 保持为0，flag为0

    cv2.imshow("2", res1) # 显示结果
    
    # 判断是否停止
    # 如果平滑后的中心点偏差在阈值内，则认为到位
    if abs(detx) < 3 and abs(dety) < 3 and flag == 1: # 确保有检测到圆环才算到位
        stop_flag = 1
    else:
        stop_flag = 0

    print("detx1=", detx1, "dety1=", dety1, "stop_flag:", stop_flag)
    cv2.waitKey(1)
    return detx1, dety1, stop_flag





def preprocess_image(frame, color_number=None):# 有色→findcontours可用前处理
    """
    图像预处理函数
    # 有色→findcontours可用前处理
    :param frame: 原始图像帧
    :param color_number: 颜色编号（1:红, 2:绿, 3:蓝），None表示自动检测
    :return: 预处理后的图像和掩膜
    """
    # 颜色阈值定义
    red_min = np.array(dim_red_min)
    red_max = np.array(dim_red_max)
    green_min = np.array(dim_green_min1)
    green_max = np.array(dim_green_max1)
    blue_min = np.array(dim_blue_min)
    blue_max = np.array(dim_blue_max)
    red_min1 = np.array(dim_red_min1)
    red_max1 = np.array(dim_red_max1)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 自动检测颜色
    if color_number is None:
        mask12 = cv2.inRange(hsv, red_min, red_max)
        mask11 = cv2.inRange(hsv, red_min1, red_max1)
        mask1 = cv2.add(mask12, mask11)
        mask2 = cv2.inRange(hsv, green_min, green_max)
        mask3 = cv2.inRange(hsv, blue_min, blue_max)
        
        red_pixels = cv2.countNonZero(mask1)
        green_pixels = cv2.countNonZero(mask2)
        blue_pixels = cv2.countNonZero(mask3)
        
        if red_pixels > blue_pixels and red_pixels > green_pixels:
            mask0 = mask1
            color_number = 1
        elif green_pixels > red_pixels and green_pixels > blue_pixels:
            mask0 = mask2
            color_number = 2
        else:
            mask0 = mask3
            color_number = 3
    # 指定颜色
    else:
        if color_number == 1:
            mask12 = cv2.inRange(hsv, red_min, red_max)
            mask11 = cv2.inRange(hsv, red_min1, red_max1)
            mask0 = cv2.add(mask12, mask11)
        elif color_number == 2:
            mask0 = cv2.inRange(hsv, green_min, green_max)
        elif color_number == 3:
            mask0 = cv2.inRange(hsv, blue_min, blue_max)
    
    # 应用掩膜
    res = cv2.bitwise_and(frame, frame, mask=mask0)
    
    # 后处理
    blured = cv2.blur(res, (5, 5))
    _, bright = cv2.threshold(blured, 10, 255, cv2.THRESH_BINARY)
    gray = cv2.cvtColor(bright, cv2.COLOR_BGR2GRAY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    opened = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
    closed1 = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    closed = cv2.morphologyEx(closed1, cv2.MORPH_CLOSE, kernel)
    
    return closed, color_number

def findBlockCenter_acquaint_color(color_cap):
    """打乱顺序夹取（获取颜色+调整）"""
    # 获取图像帧
    ret, frame = color_cap.read()
    if not ret:
        return 0, 0, None, 0, 0, 0, 0
    
    # 预处理（自动检测颜色）
    closed, color_number = preprocess_image(frame, color_number=None)
    
    # 分析轮廓（选择最下方的色块）
    h, w = frame.shape[:2]
    contours, _ = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    x_center, y_center = 0, 0
    flag = 0
    detx_p, dety_p = 0, 0
    selected_contour = None
    compare_value = -1  # 寻找最大y值（最下方）（（（为什么？

    for cnt in contours:
        (x1, y1, w1, h1) = cv2.boundingRect(cnt)
        area = w1 * h1
        if area > 0.016 * w * h:
            a = x1 + w1 / 2
            b = y1 + h1 / 2
            if y1 < compare_value:
                compare_value = y1
                x_center, y_center = a, b
                selected_contour = (x1, y1, w1, h1)
    # 绘制选中的轮廓
    if selected_contour:
        (x_, y_, w_, h_) = selected_contour
        cv2.rectangle(frame, (x_, y_), (x_ + w_, y_ + h_), (0, 0, 255), 2)
        detx_p = int(x_center - w/2 - correct_x_hough)
        dety_p = int(h/2 - correct_y_hough - y_center)
        if abs(detx_p)<12 and abs(dety_p)<12:
            flag=1
    # 显示结果
    print("detx_p:",detx_p,"dety_p:",dety_p,"flag:",flag)
    cv2.imshow("src1", frame)
    cv2.waitKey(1)    
    return x_center/w, y_center/h, frame, flag, detx_p, dety_p, color_number

def findBlockCenter(color_cap, color_number): #转盘处识别色块中心位置
    """转盘处识别色块中心位置"""
    # 获取图像帧
    ret, frame = color_cap.read()
    
    # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    # filename = f"photo_{timestamp}.jpg"
    
    # # �����һ����Ƭ
    # cv2.imwrite(filename, frame)
    
    # # ��ȡ�ļ��ľ���·������ӡ
    # saved_path = os.path.abspath(filename)
    # print(f"save to : {saved_path}")
    
    if not ret:
        return 0, 0, None, 0, 0, 0
    
    # 预处理（指定颜色）
    closed, _ = preprocess_image(frame, color_number=color_number)
    
    # 分析轮廓（选择最上方的色块）
    h, w = frame.shape[:2]
    contours, _ = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    x_center, y_center = 0, 0
    flag = 0
    detx_p, dety_p = 0, 0
    selected_contour = None
    compare_value = float('inf')  # 寻找最小y值（最上方）#避免绿色被转盘侧黄色干扰

    for cnt in contours:
        (x1, y1, w1, h1) = cv2.boundingRect(cnt)
        area = w1 * h1
        if area > block_area * w * h:  #0.039
            a = x1 + w1 / 2
            b = y1 + h1 / 2
            if y1 < compare_value:
                compare_value = y1
                x_center, y_center = a, b
                selected_contour = (x1, y1, w1, h1)
    # 绘制选中的轮廓
    if selected_contour:
        (x_, y_, w_, h_) = selected_contour
        cv2.rectangle(frame, (x_, y_), (x_ + w_, y_ + h_), (0, 0, 255), 2)
        flag = 1
        detx_p = int(x_center - w/2 - correct_x_hough)
        dety_p = int(h/2 - correct_y_hough - y_center)

    # 显示结果
    print("detx_p:",detx_p,"dety_p:",dety_p,"flag:",flag)
    cv2.imshow("src1", frame)
    cv2.waitKey(1)
    return x_center/w, y_center/h, frame, flag, detx_p, dety_p

def findBlockCenter_gray(color_cap): #在转盘上放物料（灰度处理）
    """在转盘上放物料（灰度处理）"""
    # 获取图像帧
    ret, frame = color_cap.read()
    if not ret:
        return 0, 0, None, 0, 0, 0, 0
    
    src1 = frame.copy()
    h, w = src1.shape[:2]
    
    # 灰度处理
    gray = cv2.cvtColor(src1, cv2.COLOR_BGR2GRAY)
    equalized = cv2.equalizeHist(gray)
    blurred = cv2.GaussianBlur(equalized, (9, 9), 2)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    opened = cv2.morphologyEx(blurred, cv2.MORPH_CLOSE, kernel)
    closed1 = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    closed = cv2.morphologyEx(closed1, cv2.MORPH_CLOSE, kernel)
    edges = cv2.Canny(closed, 50, 150)
    # cv2.imshow("blurred",blurred)
    # cv2.imshow("opened",opened)
    # cv2.imshow("closed",closed)
    # cv2.imshow("edges",edges)
    
    # 轮廓分析
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    largest_circle = None
    largest_area = 0
    flag = 0
    detx_p, dety_p = 0, 0
    x, y = 0, 0
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > largest_area:
            largest_area = area
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
            if len(approx) > 5:
                largest_circle = approx
    
    if largest_area > 10000 and largest_circle is not None:
        (x, y), radius = cv2.minEnclosingCircle(largest_circle)
        center = (int(x), int(y))
        radius = int(radius)
        
        # 绘制结果
        cv2.drawContours(src1, [largest_circle], 0, (0, 0, 255), 3)
        cv2.circle(src1, center, 2, (0, 0, 255), 3)
        cv2.circle(src1, center, radius, (0, 255, 0), 2)
        
        # 计算偏差
        detx_p = int(round(x - w/2 - correct_x_hough))
        dety_p = int(round(h/2 - correct_y_hough - y))
        flag = 1
    
    # 显示结果
    cv2.imshow("src1", src1)
    cv2.waitKey(1)
    
    return x/w, y/h, frame, flag, detx_p, dety_p, 0

def findBlockCenter_circle(color_cap,color_number):   #在转盘上放物料（转盘上是圆环（在每个圆环处夹着物料调整并放置
    """在转盘上放置物料（圆环检测）"""    
    ret,frame = color_cap.read()    
    
    red_min   =  np.array(dim_red_min)
    red_max   =  np.array(dim_red_max)
    green_min =  np.array(dim_green_min)
    green_max =  np.array(dim_green_max)
    blue_min  =  np.array(dim_blue_min)   
    blue_max  =  np.array(dim_blue_max)  
    red_min1   = np.array(dim_red_min1)  
    red_max1   = np.array(dim_red_max1)
    mask12 = cv2.inRange(hsv,   red_min,   red_max)
    mask11 = cv2.inRange(hsv,   red_min1,   red_max1)
    mask2 = cv2.inRange(hsv, green_min, green_max)
    mask3 = cv2.inRange(hsv,  blue_min,  blue_max)
    mask1 = cv2.add(mask12,mask11)
    if color_number == 1:
        mask0 = mask1
    elif color_number == 2:
        mask0 = mask2
    elif color_number == 3:
        mask0 = mask3

    src1 = frame.copy()
    res1 = src1.copy()
    hsv = cv2.cvtColor(src1, cv2.COLOR_BGR2HSV)     
    res1 = cv2.bitwise_and(src1, src1, mask=mask0) 
    cv2.imshow("res1",res1)

    h, w = res1.shape[:2]
    blured = cv2.blur(res1, (7, 7))
    blured = cv2.blur(res1, (5, 5))
    ret, bright = cv2.threshold(blured, 10, 255, cv2.THRESH_BINARY)
    
    gray = cv2.cvtColor(bright, cv2.COLOR_BGR2GRAY)
    equalized = cv2.equalizeHist(gray)
    h_g, w_g = gray.shape[:2]
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    opened = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
    closed1 = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    closed = cv2.morphologyEx(closed1, cv2.MORPH_CLOSE, kernel)
    blurred1 = cv2.GaussianBlur(equalized, (9, 9), 2)
    # cv2.imshow("junheng",blurred)
    # edges = cv2.Canny(blurred, 50, 150)
    # cv2.imshow("xitiaoedge:",edges)

    circles = cv2.HoughCircles(blurred1, cv2.HOUGH_GRADIENT, 0.7,70,
                            param1=100, param2=65, minRadius=houghradius_min, maxRadius=houghradius_max)    #5th circle
    # circles = cv2.HoughCircles(blurred1, cv2.HOUGH_GRADIENT, 0.7,70,
    #                         param1=100, param2=35, minRadius=houghradius_min_6th, maxRadius=houghradius_max_6th)    #6th circle
    flag = 0
    detx = 0 
    dety = 0
    detx1 = 0
    dety1 = 0
    x_center=0
    y_center=0
    flag_color_1=0
    largest_circle = None  #    ڴ洢   Բ    Ϣ
    stop_flag=0
    if circles is not None:
        flag = 1
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            if largest_circle is None or i[2] > largest_circle[2]:
                largest_circle = i 


        if largest_circle is not None:
            x_center=largest_circle[0]
            y_center=largest_circle[1]
            cv2.circle(src1, (largest_circle[0], largest_circle[1]), largest_circle[2], (0, 0, 255), 2)
            cv2.circle(src1, (largest_circle[0], largest_circle[1]), 2, (0, 0, 255), 3)
            center_text = f"({largest_circle[0]}, {largest_circle[1]})"
            text_position = (largest_circle[0] + 10, largest_circle[1] - 10)
            # cv2.putText(edges, center_text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            radius = largest_circle[2]
            radius_text = f"Radius: {radius}"
            radius_position = (largest_circle[0] + 10, largest_circle[1] + 20) 
            cv2.putText(src1, radius_text, radius_position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            detx = largest_circle[0] - w/2 -correct_x_hough
            dety = h/2 - largest_circle[1] -correct_y_hough
            detx1 = int(round(detx))
            dety1 = int(round(dety))
            flag_color_1 = 1
            # print("detx=",detx,"dety=",dety)
            # print("detx1=",detx1,"dety1=",dety1)
    else:
        cv2.putText(res1, 'no', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)   

    cv2.imshow("src1",src1)
    # print("detx_p:",detx_p,"dety_p:",dety_p,"flag_color_1:",flag_color_1)
    cv2.waitKey(1)
    return x_center/w, y_center/h, frame, flag_color_1



def detect_plate_stop(cap, detector_func, times, stop_threshold, 
                      check_direction=False, direction_threshold=0.02, 
                      **detector_args):
    """
    # 通用转盘停止检测函数
    :param camera_cap: 摄像头对象
    :param detector_func: 检测函数
    :param times: 采样次数
    :param stop_threshold: 停止阈值
    :param check_direction: 是否检查方向
    :param direction_threshold: 方向判断阈值
    :param detector_args: 传递给检测函数的参数
    :return: 是否停止的标志
    """
    cnt = 0
    x_add = 0
    y_add = 0
    get_blog = 0
    flag_stop = 0
    last_x, last_y = 0, 0
    turn_direction = None  # 初始化方向为None
    
    while cnt < times:
        # 调用检测函数获取位置信息
        result = detector_func(cap, **detector_args)
        
        # 提取位置信息（根据检测函数返回值不同）
        if len(result) >= 5:  # findBlockCenter 和 findBlockCenter_gray
            x, y, _, flag, _ = result[:5]
        else:  # findBlockCenter_circle
            x, y, _, flag = result[:4]
        
        x_add += x
        y_add += y
        last_x, last_y = x, y
        
        cv2.waitKey(1)
        cnt += 1
        get_blog += flag
    
    # 计算平均值
    x_avg = x_add / times
    y_avg = y_add / times
    print(x_avg,last_x,y_avg,last_y)
    
    # 检查停止条件
    if (abs(last_x - x_avg) < stop_threshold and 
        abs(last_y - y_avg) < stop_threshold and 
        get_blog == times):
        flag_stop = 1
    else:
        # 检查方向（如果需要）
        if check_direction and get_blog == times:
            # global turn_direction
            if (last_x - x_avg) > direction_threshold:
                turn_direction = True
            elif (last_x - x_avg) < -direction_threshold:
                turn_direction = False
        flag_stop = 0
    
    print(f"检测结果: flag={flag_stop}, 成功次数={get_blog}/{times}")
    if check_direction:
        return flag_stop,turn_direction
    else:
        return flag_stop

def detectPlate(cap, color_number):
    """检测转盘是否停止（从转盘上夹走物料）"""
    return detect_plate_stop(
        cap=cap,
        detector_func=findBlockCenter,
        times=4,
        stop_threshold=0.01,
        color_number=color_number
    )

def detectPlate_check(cap, color_number):
    """检测爪子是否成功抓起物料"""
    return detect_plate_stop(
        cap=cap,
        detector_func=findBlockCenter,
        times=3,
        stop_threshold=0.1,
        color_number=color_number
    )

def detectPlate_gray(cap):
    """检测转盘是否停止（灰度处理）-色块"""
    return detect_plate_stop(
        cap=cap,
        detector_func=findBlockCenter_gray,
        times=5,
        stop_threshold=0.01,
        check_direction=True
    )

def detectPlate_circle(cap, color_number):
    """检测转盘是否停止（圆环检测）"""
    return detect_plate_stop(
        cap=cap,
        detector_func=findBlockCenter_circle,
        times=3,
        stop_threshold=0.01,
        check_direction=True,
        color_number=color_number
    )




def detectLine(cap):   #直线检测

    ret,frame = cap.read()

    cnt_line = 0
    res1=frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   #ת Ҷ ͼ
    equalized = cv2.equalizeHist(gray)
    # cv2.imshow("junheng",equalized)
    # ret, thresh = cv2.threshold(equalized, 120, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    opened = cv2.morphologyEx(equalized, cv2.MORPH_CLOSE, kernel)#      
    closed1 = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    # closed = cv2.morphologyEx(closed1, cv2.MORPH_CLOSE, kernel)
    blurred = cv2.GaussianBlur(closed1, (9, 9), 2)
    edges = cv2.Canny(blurred, 50, 150)
    cv2.imshow("edges",edges)

    lines = cv2.HoughLines(edges,1,np.pi/180,threshold =150)#  ȡͼ е   
    cnt = 0
    sumTheta = 0
    averageTheta = 0
    # global last_theta
    last_theta = 0
    if lines is not None:
        for line in lines:#  ÿ   ߻     
            rho,theta = line[0]
            
            if ((np.abs(theta)>=1.1) & (np.abs(theta)<=2.2)):
                cnt = cnt + 1
                sumTheta = sumTheta + theta / 5.0
                # sumTheta = sumTheta + theta
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(res1,(x1,y1),(x2,y2),(0,0,255),2)

    if not (cnt == 0):
        averageTheta = 5.0 * sumTheta / cnt #  ýǶȵ ƽ 
        # averageTheta = sumTheta / cnt
        last_theta =  averageTheta
    else :
        averageTheta = last_theta
    # print(averageTheta)
    averageTheta180=np.degrees(averageTheta)
    finaltheta=90-averageTheta180
    print("hudu:",averageTheta,"   jiaodu:",averageTheta180,"    jiajiao;",finaltheta)
    cv2.imshow("line",res1)
    line_flag=0
    if abs(finaltheta)<0.5:
    # if abs(finaltheta)<1:
        line_flag=1
    finaltheta=int(round(finaltheta))
    if (finaltheta==90 ):
        finaltheta=0
    cv2.waitKey(1)
    return finaltheta,line_flag

def detectLine_gray(color_cap):   #直线检测（黄灰交界线
    gray_min = np.array(dim_gray_min)
    gray_max = np.array(dim_gray_max)


    ret,frame = color_cap.read()



    src1 = frame.copy()
    res1 = src1.copy()
    hsv = cv2.cvtColor(src1, cv2.COLOR_BGR2HSV)    
    mask_gray = cv2.inRange(hsv,   gray_min,   gray_max)

    res1 = cv2.bitwise_and(src1, src1, mask=mask_gray)   
    cv2.imshow("res1",res1)

    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   
    # equalized = cv2.equalizeHist(gray)
    # cv2.imshow("junheng",equalized)
    # ret, thresh = cv2.threshold(equalized, 120, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    opened = cv2.morphologyEx(res1, cv2.MORPH_CLOSE, kernel)
    closed1 = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    # closed = cv2.morphologyEx(closed1, cv2.MORPH_CLOSE, kernel)
    blurred = cv2.GaussianBlur(closed1, (9, 9), 2)
    edges = cv2.Canny(blurred, 50, 150)
    cv2.imshow("edges",edges)

    lines = cv2.HoughLines(edges,1,np.pi/180,threshold =120)
    cnt = 0
    sumTheta = 0
    averageTheta = 0
    # global last_theta
    last_theta = 0
    if lines is not None:
        for line in lines:
            rho,theta = line[0]
            
            if ((np.abs(theta)>=1.1) & (np.abs(theta)<=2.2)):
                cnt = cnt + 1
                sumTheta = sumTheta + theta / 5.0
                # sumTheta = sumTheta + theta
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
    if not (cnt == 0):
        averageTheta = 5.0 * sumTheta / cnt 
        # averageTheta = sumTheta / cnt
        last_theta =  averageTheta
    else :
        averageTheta = last_theta
    # print(averageTheta)
    averageTheta180=np.degrees(averageTheta)
    finaltheta=90-averageTheta180
    print("hudu:",averageTheta,"   jiaodu:",averageTheta180,"    jiajiao;",finaltheta)
    cv2.imshow("line",frame)
    line_flag=0
    # if(abs(finaltheta)<0.8  and abs(finaltheta)>0.1):
    if abs(finaltheta)<0.5:
    # if abs(finaltheta)<1:
        line_flag=1
    # if finaltheta<-0.5 and finaltheta>-1.5:
    #     line_flag=1
    finaltheta=int(round(finaltheta))
    if (finaltheta==90 ):
        finaltheta=0
    cv2.waitKey(1)
    return finaltheta,line_flag




def code(code_cap):  #识别二维码、条形码
    '''识别二维码、条形码'''
    ret,frame = code_cap.read()
    ret,frame = code_cap.read()
    ret,frame = code_cap.read()
    if code_cap.isOpened():
        print("222     successsssssssss")
    else:
        print("222     faillllllllll")
    count=0
    while ret == False and count<10:
        print("ret  fail")
        ret,frame = code_cap.read()
        count+=1
    cv2.imshow("frame",frame)
    barcodes = decode(frame)  
    flag = 0
    data = []
    # cv2.waitKey(10)

    for barcode in barcodes:
        data = barcode.data.decode("utf8")
        print(data)
    if len(barcodes)>0:
        flag = 1
    return data,flag

def sort(data):  #将二维码信息字符转为数字数组
    '''将二维码信息字符转为数字数组'''
    color_order = []
    print(data,type(data))
    for i in data:
        if ( i == '1'):
            color_order.append(1)
        elif ( i == '2'):
            color_order.append(2)
        elif ( i == '3'):
            color_order.append(3)
    return color_order




def serialInit():  #初始化串口通信
    Pi_serial  =  serial.Serial( port="/dev/ttyAMA2",
                              baudrate=115200,
                              bytesize=serial.EIGHTBITS,
                              parity=serial.PARITY_NONE,
                              stopbits=serial.STOPBITS_ONE,
                              )
    return Pi_serial

def receiveMessage(ser):  #接收信息
    count = ser.inWaiting()
    if count != 0:
        recv = ser.read(count) 
        # recv_data=recv.hex()
        recv_data=recv
        # if recv[0] == 0xAA and recv[1] == 0xBB and recv[-1] == 0xCC:
        #     recv_useful = recv[2]  
        #     recv_data=recv_useful.hex()
        # else:
        #     recv_data = None  
    else:
        recv_data = None
    ser.flushInput()
    time.sleep(0.01)
    return recv_data

def sendMessage(ser,data):  #发送到位信息（单个正数
    data_hex=hex(data)[2:]
    data_hex = data_hex.zfill(2)
    # data_hex=data.to_bytes(1,'big')
    data_hex=bytes.fromhex(data_hex)
    pre='?'.encode()
    data_1=pre+data_hex
    # print(data_hex)
    # data_pack = 'AA'+'BB'+data_hex+'CC'
    # data_pack =data_hex
    # ser.write(bytes.fromhex(data))
    # ser.write(bytes.fromhex(data_hex))
    ser.write(data_1)
    print(data_1)
    time.sleep(0.001)
    # ser.write(data_1)

    return 0

def sendMessage2(ser, data1, data2):   #发送圆环中心 分辨率为大数 偏差值会大于一个16进制的最大数字255
    # 处理 data1
    if data1 >= 0:
        signal1 = 1
    else:
        signal1 = 2
        data1 = abs(data1)  # 取绝对值
    # 处理 data2
    if data2 >= 0:
        signal2 = 1
    else:
        signal2 = 2
        data2 = abs(data2)  # 取绝对值
    # 将数据和符号位转换为字节串（4字节，大端序）
    signal1_bytes = signal1.to_bytes(1, 'big')  # 符号位占1字节
    data1_bytes = data1.to_bytes(2, 'big')      # 数据占2字节
    signal2_bytes = signal2.to_bytes(1, 'big')  # 符号位占1字节
    data2_bytes = data2.to_bytes(2, 'big')      # 数据占2字节
    end='!'.encode()
    # 发送数据
    data_pack = signal1_bytes + data1_bytes + signal2_bytes + data2_bytes + end
    ser.write(data_pack)
    print(data_pack.hex().upper())  # 打印十六进制格式
    time.sleep(0.001)

    return 0

def sendMessage3(ser, data):    #发送二维码信息
    if isinstance(data, list):
        length = len(data)
        # processed_data = []
        combined_data_hex=''
        for item in data:
            data_hex = hex(item)[2:]
            data_hex = data_hex.zfill(2)
            # data_hex=item.to_bytes(1,'big')
            # processed_data.append(data_hex)
            combined_data_hex += data_hex
        pre='*'.encode()
        combined_data_hex=bytes.fromhex(combined_data_hex)
        combined_data_hex=pre+combined_data_hex
        print(f"Array length: {length}")
        # combined_data_hex = '+'.join(processed_data)
        # print(combined_data_hex)
        # ser.write(bytes.fromhex(combined_data_hex))
        ser.write(combined_data_hex)
        print(combined_data_hex)
        # print(combined_data_hex)
    else:
        data_hex = hex(data)[2:]
        data_hex = data_hex.zfill(2)
        ser.write(bytes.fromhex(data_hex))
        print(f"Single data: {data}")

def sendMessage4(ser,data1):   #/没在用/ 发送直线角度（单个数字带正负 角度
    if data1>=0:
        signal1=1
    else :
        signal1=2
        data1=abs(data1)
    data_hex1=hex(data1)[2:]
    data_hex1 = data_hex1.zfill(2)
    signal_hex1=hex(signal1)[2:]
    signal_hex1 = signal_hex1.zfill(2)
    data_pack = signal_hex1+data_hex1
    ser.write(bytes.fromhex(data_pack))
    print("angle direction:",data_pack)
    time.sleep(0.001)

    return 0

def sendMessage5(ser, data_l, data_x, data_y):   #发送偏差值 粗调 直线+圆心偏差 大分辨率
    # 处理 data_l
    if data_l >= 0:
        signal_l = 1
    else:
        signal_l = 2
        data_l = abs(data_l)  # 取绝对值
    # 处理 data_x
    if data_x >= 0:
        signal_x = 1
    else:
        signal_x = 2
        data_x = abs(data_x)  # 取绝对值
    # 处理 data_y
    if data_y >= 0:
        signal_y = 1
    else:
        signal_y = 2
        data_y = abs(data_y)  # 取绝对值
    # 将数据和符号位转换为字节串（1字节，大端序）
    signal_l_bytes = signal_l.to_bytes(1, 'big')  # 符号位占1字节
    data_l_bytes = data_l.to_bytes(1, 'big')      # 数据占1字节
    signal_x_bytes = signal_x.to_bytes(1, 'big')  # 符号位占1字节
    data_x_bytes = data_x.to_bytes(2, 'big')      # 数据占2字节
    signal_y_bytes = signal_y.to_bytes(1, 'big')  # 符号位占1字节
    data_y_bytes = data_y.to_bytes(2, 'big')      # 数据占2字节
    end='!'.encode()
    # 发送数据
    data_pack = signal_l_bytes + data_l_bytes + signal_x_bytes + data_x_bytes + signal_y_bytes + data_y_bytes + end
    ser.write(data_pack)
    print("together:", data_pack.hex().upper())  # 打印十六进制格式
    time.sleep(0.001)
    return 0

def sendMessage6(ser, data):    #发送从右到左颜色（在一条直线三个圆环处夹取物料
    if isinstance(data, list):
        length = len(data)
        # processed_data = []
        combined_data_hex=''
        for item in data:
            data_hex = hex(item)[2:]
            data_hex = data_hex.zfill(2)
            # data_hex=item.to_bytes(1,'big')
            # processed_data.append(data_hex)
            combined_data_hex += data_hex
        pre='%'.encode()
        combined_data_hex=bytes.fromhex(combined_data_hex)
        combined_data_hex=pre+combined_data_hex
        print(f"Array length: {length}")
        # combined_data_hex = '+'.join(processed_data)
        # print(combined_data_hex)
        # ser.write(bytes.fromhex(combined_data_hex))
        ser.write(combined_data_hex)
        print(combined_data_hex)
        # print(combined_data_hex)
    else:
        data_hex = hex(data)[2:]
        data_hex = data_hex.zfill(2)
        ser.write(bytes.fromhex(data_hex))
        print(f"Single data: {data}")
