import os
from datetime import datetime
import cv2
import numpy as np
import math
import time
import serial 
from pyzbar.pyzbar import decode  
import logging

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
# block_area=0.039
block_area=0.018
#粗调时0.016
cutiao_center_circle=0.0097

#!放的偏右了x值就+，偏下了y值就-

#new paw
#cedingzhi 30 13
#cedingzhi 39 -9
#42  -9
#40  -7
#30  7 #jiangxialai qian
#粗调时高度偏差值(findcontours)
correct_x=40
correct_y=14

#new paw
#cedingzhi 33 9
#celiangzhi 42
#houghcircles 43 7
#20thin---y=16
#用了很久  43  7 
#37  6
#45  7
#细调时高度的偏差值(houghcircles)
correct_x_hough=36
correct_y_hough=14
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
prev_centers = []#圆环中心


# #粗调到位阈值（圆+直线）
# #前后左右
# limit_position_circle=4
# #直线斜率
# limit_position_line=0.5  #所有直线斜率

# #细调到位阈值（圆环-放下物料）
# limit_ring_1st=50
# limit_ring_2nd=3

# 全局变量，用于在 circlePut1 函数调用之间保持状态
g_circle_put_state = {
    "prev_detx": None,
    "prev_dety": None
}
# 全局变量，用于在 together_line_circle1 函数调用之间保持状态
g_together_state = {
    "prev_detx": None,
    "prev_dety": None,
}

def reset_circle_put_state():
    global g_circle_put_state
    g_circle_put_state = {"prev_detx": None, "prev_dety": None}

def reset_together_state():
    global g_together_state
    g_together_state = {"prev_detx": None, "prev_dety": None}

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
    # cv2.imshow("img_green",img_green)

    # img_green=edges1[y_g:(y_g+h_g),x_g:(x_g+w_g)]


    # total_mask = cv2.bitwise_or(mask1, cv2.bitwise_or(mask2, mask3))
    # cv2.imshow("mask",mask3)
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


def together_line_circle_det(cap, limit_position_circle, limit_position_line):  #粗调 直线+圆（findContours 看中间圆环-绿色
    '''加入帧间差值判断'''
    global g_together_state  # 声明使用全局变量
    VELOCITY_CIRCLE = 5   # 圆心位置变化阈值

    ret,frame = cap.read()

    res1 = frame.copy()
    h, w = res1.shape[:2]

    #####################line图像处理#################################s
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
    # cv2.imshow("img_green",img_green)

    # img_green=edges1[y_g:(y_g+h_g),x_g:(x_g+w_g)]


    # total_mask = cv2.bitwise_or(mask1, cv2.bitwise_or(mask2, mask3))
    # cv2.imshow("mask",mask3)
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
    # finaltheta=int(round(finaltheta))
    if (finaltheta==90 ):
        finaltheta=0
    finaltheta=finaltheta * 10
    finaltheta=int(round(finaltheta))

    # TARGET_ANGLE = 0.4  # 定义目标角度
    # theta_error = finaltheta - TARGET_ANGLE
    # print("finaltheta:",finaltheta,"theta_error:",theta_error)
    # if abs(theta_error) < limit_position_line:
    #     line_flag=1
    # theta_to_return = int(round(theta_error*10))
    # if (finaltheta==90 ):
    #     theta_to_return=0
    # print("theta_to_return:",theta_to_return/10,"line_flag:",line_flag)

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
        # x_incolor=x-correct_x_hough-w/2
        # y_incolor=h/2-y-correct_y_hough
        detx1=int(round(x_incolor))
        dety1=int(round(y_incolor))
        # print("cccccccccccccccccccccccccccccccccccccc")
        global flag_in
        flag_in=1
    else:
        cv2.putText(res1, 'No circle found', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        detx1=int(round(x_g_new + w_g_new/2 -w/2 -correct_x))
        dety1=int(round(h/2 - y_g_new - h_g_new/2 -correct_y))
        # print("nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn")


    # print("x_incolor:",x_incolor,"y_incolor:",y_incolor)
    print("detx1:",detx1,"dety1:",dety1)
    # det=4
    vel_x, vel_y = 0, 0
    if (g_together_state["prev_detx"] and g_together_state["prev_dety"]) is not None: # 确保有历史数据
        vel_x = x_incolor - g_together_state["prev_detx"]
        vel_y = y_incolor - g_together_state["prev_dety"]
        print("vel_x:", vel_x, "vel_y:", vel_y)
    # 4. 更新历史状态，为下一次调用做准备
    g_together_state["prev_detx"] = x_incolor
    g_together_state["prev_dety"] = y_incolor

    if abs(x_incolor)<limit_position_circle and abs(y_incolor)<limit_position_circle:
        if x_incolor == 0 and y_incolor==0:
            stop_flag=0
        else:
            is_circle_vel_ok = abs(vel_x) < VELOCITY_CIRCLE and abs(vel_y) < VELOCITY_CIRCLE

            # 3. 最终决定 line_flag 和 move_flag
            # 只有当位置和速度都满足条件时，才认为该项到位
            stop_flag = 1 if is_circle_vel_ok else 0

    


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
    cv2.imshow("blurred1", blurred1)

    # 使用HoughCircles检测圆
    # param1: Canny边缘检测的高阈值，低阈值是其一半
    # param2: 累加器阈值，越小表示检测到的圆越多
    # circles = cv2.HoughCircles(blurred1, cv2.HOUGH_GRADIENT, 0.7, 70,
    #                            param1=100, param2=83, minRadius=houghradius_min, maxRadius=houghradius_max)
    circles = cv2.HoughCircles(blurred1, cv2.HOUGH_GRADIENT, 0.7, 70,
                               param1=100, param2=87, minRadius=houghradius_min, maxRadius=houghradius_max)

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
    coords_text = ''
    coords_text = f"({detx1:.2f},{dety1:.2f},R={r:.2f})"
    cv2.putText(res1, coords_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, (0, 255, 255), 2)
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

def circlePut_det(cap):  # 细调第二步 灰度houghcircles识别圆心
    '''细调第二步 灰度houghcircles识别圆心
        加入对帧间速度检测
    '''
    # success, frame = cap.read() # 读取多次是为了确保获取最新帧，但一次通常也够

    global g_circle_put_state # 声明我们将使用全局变量
    # --- 新增参数 (这些值需要您根据实际情况调试) ---
    POSITION_THRESHOLD = 3   # 位置偏差阈值 (像素)
    VELOCITY_THRESHOLD = 5   # “速度”阈值 (像素/帧)，即两帧偏差的变化量
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
    cv2.imshow("blurred1", blurred1)

    # ###################### 使用CLAHE增强对比度
    
    # lab = cv2.cvtColor(frame, cv2.COLOR_LAB2BGR)
    
    # # 分离L, a, b通道
    # l, a, b = cv2.split(lab)
    
    # # 对L（亮度）通道应用CLAHE
    # clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    # cl = clahe.apply(l)
    
    # # 合并处理后的L通道和原始的a,b通道
    # limg = cv2.merge((cl, a, b))
    
    # # 将图像从LAB转换回BGR
    # enhanced_frame = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    # gray = cv2.cvtColor(enhanced_frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("enhanced_frame", enhanced_frame)
    # blurred2 = cv2.GaussianBlur(gray, (9, 9), 2)
    # cv2.imshow("blurred", blurred2)



    # 使用HoughCircles检测圆
    # param1: Canny边缘检测的高阈值，低阈值是其一半
    # param2: 累加器阈值，越小表示检测到的圆越多
    # circles = cv2.HoughCircles(blurred1, cv2.HOUGH_GRADIENT, 0.7, 70,
    #                            param1=100, param2=83, minRadius=houghradius_min, maxRadius=houghradius_max)
    circles = cv2.HoughCircles(blurred1, cv2.HOUGH_GRADIENT, 0.7, 70,
                               param1=100, param2=87, minRadius=houghradius_min, maxRadius=houghradius_max)

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
    coords_text = ''
    coords_text = f"({detx1:.2f},{dety1:.2f},R={radius:.2f})"
    cv2.putText(res1, coords_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, (0, 255, 255), 2)
    cv2.imshow("2", res1) # 显示结果
    
    if flag == 1:
        # 只有找到了圆，才进行后续判断
        
        # 1. 判断位置是否到位
        position_ok = abs(detx) < POSITION_THRESHOLD and abs(dety) < POSITION_THRESHOLD
        
        # 2. 计算并判断"速度"是否到位
        vel_x, vel_y = 0, 0
        if g_circle_put_state["prev_detx"] is not None:
            vel_x = detx - g_circle_put_state["prev_detx"]
            vel_y = dety - g_circle_put_state["prev_dety"]
        
        velocity_ok = abs(vel_x) < VELOCITY_THRESHOLD and abs(vel_y) < VELOCITY_THRESHOLD
        
        # 3. 最终决定 stop_flag
        if position_ok and velocity_ok:
            stop_flag = 1
            # print(f"STABLE & CENTERED! Pos:({detx},{dety}), Vel:({vel_x},{vel_y})")
        else:
            stop_flag = 0
            # print(f"Adjusting... Pos:({detx},{dety}), Vel:({vel_x},{vel_y})")

        # 4. 更新历史状态，为下一次调用做准备
        g_circle_put_state["prev_detx"] = detx
        g_circle_put_state["prev_dety"] = dety
        
    else:
        # 如果当前帧没有找到圆，重置历史状态，防止用旧数据误判
        cv2.putText(res1, 'no circle detected', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        g_circle_put_state["prev_detx"] = None
        g_circle_put_state["prev_dety"] = None

    print("detx1=", detx1, "dety1=", dety1, "stop_flag:", stop_flag)
    cv2.waitKey(1)
    return detx1, dety1, stop_flag


def circlePut_hzw(cap,r_min=houghradius_min,r_max=houghradius_max,smooth_factor=0.4):  #黄自文的//细调第二步
    """
    获取圆环的中心坐标
    
    参数:
    frame : 输入的图像帧
    r_min (int): 圆环内径的最小半径
    r_max (int): 圆环外径的最大半径

    返回:
    圆环的中心坐标(x,y,r)
    """
    ret, frame = cap.read() 
    h, w = frame.shape[:2]
    median = cv2.medianBlur(frame,3)
    grayImg = cv2.cvtColor(median,cv2.COLOR_BGR2GRAY)
    # cv2.imshow("grayImg",grayImg)
    grayImg = cv2.GaussianBlur(grayImg,(5,5),0)
    cannyImg = cv2.Canny(grayImg,50,150)
    #cv2.imshow("cannyImg",cannyImg)
    circle_size=0
    pre_list=[]
    circles_list=[]
    res_list=[]

    circles_pre = cv2.HoughCircles(grayImg, cv2.HOUGH_GRADIENT_ALT, 1.5, 80, 
                                    param1=240, param2=0.80, 
                                    minRadius=r_min, maxRadius=r_max)
    # 检查第一次检测结果
    if circles_pre is None:
        # print("未检测到初始圆环")
        return 0, 0, 0
    
    circles_pre = np.round(circles_pre[0, :]).astype("int")
    pre_list = [(x, y, r) for (x, y, r) in circles_pre]
    circle_size = len(pre_list)
    
    
    # 第二次检测
    circles = cv2.HoughCircles(grayImg, cv2.HOUGH_GRADIENT_ALT, 1.8, 30, 
                                param1=340, param2=0.95, 
                            minRadius=r_min, maxRadius=r_max)
    if circles is None:
        # print("未检测到精细圆环")
        return 0, 0, 0
    
    circles = np.round(circles[0, :]).astype("int")
    
    # 初始化分类列表
    circles_list = [[] for _ in range(circle_size)]
    
    # 匹配逻辑
    for (x, y, r) in circles:
        if circle_size > 0:
            distances = [(i, (x-pre_x)**2 + (y-pre_y)**2) for i, (pre_x, pre_y, _) in enumerate(pre_list)]
            if distances:
                closest_idx, closest_dist = min(distances, key=lambda item: item[1])
                # 只有当距离小于阈值时才匹配
                if closest_dist < (r * 0.5)**2:  # 距离阈值为半径的一半
                    circles_list[closest_idx].append((x, y, r))
                else:
                    # 距离太远，可能是新的圆，创建新分组
                    if len(circles_list) < 4:  # 限制最大分组数
                        circles_list.append([(x, y, r)])
                        circle_size += 1
    
    # 滤波得到精细坐标
    for i in range(circle_size):
        x_sum = 0
        y_sum = 0
        r_sum = 0  # 添加半径求和
        size = len(circles_list[i])
        
        for x, y, r in circles_list[i]:
            # cv2.circle(frame, (x, y), r, (0, 255, 0), 1)
            x_sum += x
            y_sum += y
            r_sum += r  # 累加半径
        
        if(size != 0):
        # 使用中值滤波而非平均值
            # 在计算圆心时
            if size >= 3:  # 至少需要3个点才能计算中值
                # 分别提取x,y,r坐标列表
                x_list = [x for x,_,_ in circles_list[i]]
                y_list = [y for _,y,_ in circles_list[i]]
                r_list = [r for _,_,r in circles_list[i]]
                
                # 计算中值
                x_med = sorted(x_list)[len(x_list)//2]
                y_med = sorted(y_list)[len(y_list)//2]
                r_med = sorted(r_list)[len(r_list)//2]
                
                res_list.append((x_med, y_med, r_med))

            else:
                # 点太少，使用平均值
                x_avg = x_sum / size
                y_avg = y_sum / size
                r_avg = r_sum / size
                res_list.append((x_avg, y_avg, r_avg))
            
    # 在返回结果前应用时间滤波
    res_list = apply_temporal_filter(res_list,smooth_factor)
    detx = 0
    dety = 0
    detx1 = 0
    dety1 = 0
    stop_flag = 0
    flag = 0

    # 修改顶部坐标显示，增加半径信息
    if len(res_list) > 0:
        coords_text = " "
        for idx, (x, y, r) in enumerate(res_list):  # 注意这里增加了r
            flag = 1
            coords_text += f"({x:.2f},{y:.2f},R={r:.2f})"
            cv2.circle(frame, (int(x), int(y)), int(r), (0, 0, 255), 2)
            cv2.circle(frame, (int(x), int(y)), 2, (0, 255, 255), 3)
            if idx < len(res_list) - 1:
                coords_text += ", "
        # cv2.putText(frame, coords_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
        #             0.7, (0, 255, 255), 2)
        x, y, r = res_list[0]
        # 计算偏差，使用平滑后的圆环坐标
        detx = x - w/2 - correct_x_hough
        dety = h/2 - y - correct_y_hough
        detx1 = int(round(detx))
        dety1 = int(round(dety))
        print("detx=", detx1, "dety=", dety1)
    coords_text = ''
    coords_text = f"({detx1:.2f},{dety1:.2f},R={r:.2f})"
    cv2.putText(frame, coords_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, (0, 255, 255), 2)
    if abs(detx) < 3 and abs(dety) < 3 and flag == 1: # 确保有检测到圆环才算到位
        stop_flag = 1
    cv2.imshow("frame_hzw",frame)
    cv2.waitKey(1)
    return detx1, dety1, stop_flag


def circlePut_ds(cap,r_min=houghradius_min,r_max=houghradius_max,smooth_factor=0.4):
    # 预处理（保持不变）
    ret, frame = cap.read()
    h, w = frame.shape[:2]
    median = cv2.medianBlur(frame, 3)
    grayImg = cv2.cvtColor(median, cv2.COLOR_BGR2GRAY)
    grayImg = cv2.GaussianBlur(grayImg, (5, 5), 0)
    
    # 多尺度霍夫检测（替代两次检测）
    circles_list = []
    for dp in [1.2, 1.5, 1.8]:
        circles = cv2.HoughCircles(
            grayImg, cv2.HOUGH_GRADIENT_ALT, 
            dp, 50,  # 动态minDist
            param1=300, param2=0.85,
            minRadius=r_min, maxRadius=r_max
        )
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            circles_list.extend([(x, y, r) for (x, y, r) in circles])
    
    # if not circles_list:
    #     return []
    
    # 使用简单聚类替代DBSCAN
    centers = np.array([c[:2] for c in circles_list])
    
    # 动态eps：基于图像尺寸
    eps = max(15, frame.shape[1] * 0.02)
    labels = simple_cluster(centers, eps=eps, min_samples=2)
    
    res_list = []
    unique_labels = set(labels)

    for label in unique_labels:
        if label == -1:  # 跳过噪声点
            continue
            
        # 获取当前簇的所有圆
        indices = np.where(labels == label)[0]
        cluster_circles = [circles_list[i] for i in indices]
        
        # 提取坐标和半径
        xs = [x for x, _, _ in cluster_circles]
        ys = [y for _, y, _ in cluster_circles]
        rs = [r for _, _, r in cluster_circles]
        
        # 计算加权中心（大圆权重更高）
        weights = np.array(rs) ** 2
        x_center = np.average(xs, weights=weights)
        y_center = np.average(ys, weights=weights)
        
        # 选择最接近中值的半径
        r_median = np.median(rs)
        closest_r = min(rs, key=lambda r: abs(r - r_median))
        
        res_list.append((x_center, y_center, closest_r))
    
    # 时间滤波（保持您的实现）
    res_list = apply_temporal_filter(res_list, smooth_factor)
    detx = 0
    dety = 0
    detx1 = 0
    dety1 = 0
    stop_flag = 0
    flag = 0

    # 修改顶部坐标显示，增加半径信息
    if len(res_list) > 0:
        coords_text = " "
        for idx, (x, y, r) in enumerate(res_list):  # 注意这里增加了r
            flag = 1
            coords_text += f"({x:.2f},{y:.2f},R={r:.2f})"
            cv2.circle(frame, (int(x), int(y)), int(r), (0, 0, 255), 2)
            cv2.circle(frame, (int(x), int(y)), 2, (0, 255, 255), 3)
            if idx < len(res_list) - 1:
                coords_text += ", "
        # cv2.putText(frame, coords_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
        #             0.7, (0, 255, 255), 2)
        x, y, r = res_list[0]
        # 计算偏差，使用平滑后的圆环坐标
        detx = x - w/2 - correct_x_hough
        dety = h/2 - y - correct_y_hough
        detx1 = int(round(detx))
        dety1 = int(round(dety))
        print("detx=", detx1, "dety=", dety1)
    coords_text = ''
    coords_text = f"({detx1:.2f},{dety1:.2f},R={r:.2f})"
    cv2.putText(frame, coords_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, (0, 255, 255), 2)
    if abs(detx) < 3 and abs(dety) < 3 and flag == 1: # 确保有检测到圆环才算到位
        stop_flag = 1
    cv2.imshow("frame_ds",frame)
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
    mask12 = cv2.inRange(hsv, red_min, red_max)
    mask11 = cv2.inRange(hsv, red_min1, red_max1)
    mask1 = cv2.add(mask12, mask11)
    mask2 = cv2.inRange(hsv, green_min, green_max)
    mask3 = cv2.inRange(hsv, blue_min, blue_max)
    mask0 = None
    
    
    
    # 自动检测颜色
    if color_number is None:
        red_pixels = cv2.countNonZero(mask1)
        green_pixels = cv2.countNonZero(mask2)
        blue_pixels = cv2.countNonZero(mask3)
        
        pixel_counts = {1: red_pixels, 2: green_pixels, 3: blue_pixels}
        # 找到像素数最多的颜色
        if max(pixel_counts.values()) > 0: # 确保至少有一个颜色被检测到
            color_number = max(pixel_counts, key=pixel_counts.get)
            if color_number == 1: mask0 = mask1
            elif color_number == 2: mask0 = mask2
            else: mask0 = mask3

    # 指定颜色
    else:
        if color_number == 1:
            mask0 = mask1
        elif color_number == 2:
            mask0 = mask2
        elif color_number == 3:
            mask0 = mask3
    
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
    cv2.imshow("closed",closed)
    
    # 分析轮廓（选择最下方的色块）
    h, w = frame.shape[:2]
    src1 = frame.copy()
    contours, _ = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # x_center, y_center = 0, 0
    # flag = 0
    # detx_p, dety_p = 0, 0
    # selected_contour = None
    # compare_value = -1  # 寻找最大y值（最下方）（（（为什么？

    # for cnt in contours:
    #     (x1, y1, w1, h1) = cv2.boundingRect(cnt)
    #     area = w1 * h1
    #     if area > 0.016 * w * h:
    #         a = x1 + w1 / 2
    #         b = y1 + h1 / 2
    #         if y1 > compare_value:
    #             compare_value = y1
    #             x_center, y_center = a, b
    #             selected_contour = (x1, y1, w1, h1)
    # # 绘制选中的轮廓
    # if selected_contour:
    #     (x_, y_, w_, h_) = selected_contour
    #     cv2.rectangle(frame, (x_, y_), (x_ + w_, y_ + h_), (0, 0, 255), 2)
    #     detx_p = int(x_center - w/2 - correct_x_hough)
    #     dety_p = int(h/2 - correct_y_hough - y_center)
    #     if abs(detx_p)<12 and abs(dety_p)<12:
    #         flag=1

    #     # 绘制调试信息
    #     cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 2)
    #     cv2.putText(frame, f"Color: {color_number}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    #     cv2.putText(frame, f"Delta: ({detx_p}, {dety_p})", (x1, y1 + h1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    # # 显示结果
    # print("detx_p:",detx_p,"dety_p:",dety_p,"flag:",flag)
    # cv2.imshow("frame", frame)


    num = 0
    a_sum=0
    b_sum=0
    x_center = 0
    y_center = 0
    c = 0
    detx=10000
    dety=10000
    detx_p=0
    dety_p=0
    flag = 0
    for cnt343 in contours:
        (x1, y1, w1, h1) = cv2.boundingRect(cnt343)  
        area = cv2.contourArea(cnt343)
        if w1*h1 > 0.016*w*h:
        # if area > 0.07*w*h:
            a = x1 + w1 / 2
            b = y1 + h1 / 2
            a_sum +=a
            b_sum +=b
            num += 1
            # print("color",num,":",a/w, b/h)
            # s=(x1+w1)*(y1+h1)
            
            cv2.rectangle(src1, (x1, y1), (x1 + w1, y1 + h1), (0, 0, 255), 2)
            cv2.putText(src1, 'color', (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            # area_text=f"{area}"
            area_text=f"{w1*h1}"
            cv2.putText(src1, area_text, (x1+60, y1 +h1+ 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            center_text = f"({a}, {b})"
            cv2.putText(src1, center_text, (x1, y1+h1+5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            color_text=f"{color_number}"
            cv2.putText(src1, color_text, (x1, y1+h1+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            
            if num == 1 or c < y1:
                x_center = a
                y_center = b
                c = y1
            # flag_color_1 = 1
            detx = a - w/2 - correct_x_hough
            dety = h/2 - correct_y_hough - b
            detx_p = int(detx)
            dety_p = int(dety)
            print("detx:",detx,"dety:",dety)
    if abs(detx)<12 and abs(dety)<12:
        flag=1
    if detx==10000 and dety==10000:
        detx_p=0
        dety_p=0
        flag=0
    cv2.imshow("src1",src1)
    cv2.waitKey(1)    
    return x_center/w, y_center/h, frame, flag, detx_p, dety_p, color_number

def findBlockCenter(color_cap, color_number, is_check=0, is_get_from_plate=0): #转盘处识别色块中心位置
    """
    转盘处识别色块中心位置
    :is_check: 是否检查（转盘上只看爪子里位置）
    :is_get_from_plate: 是否从转盘上获取（若绿色黄色混淆则切掉左右三角）
    """
    # 获取图像帧
    ret, frame = color_cap.read()

    if not ret:
        return 0, 0, None, 0, 0, 0
    
    # 预处理（指定颜色）
    closed, _ = preprocess_image(frame, color_number=color_number)
    if is_check:
        x1, y1 = 473, 152  # 左上角坐标 (x, y)
        x2, y2 = 894, 560  # 右下角坐标 (x, y)
        # 创建一个全零的掩码（与图像同尺寸）
        mask = np.zeros_like(closed)
        # 将目标矩形区域设为 1（或 255，根据图像类型）
        mask[y1:y2, x1:x2] = 1  # 单通道：1；三通道： (1, 1, 1)
        closed = closed * mask  # 利用广播机制
    # 分析轮廓（选择最上方的色块）
    h, w = frame.shape[:2]
    if is_get_from_plate == 1 and color_number == 2:
        # 创建左下角和右下角的三角形掩码
        mask = np.ones_like(closed) * 255  # 创建全白掩码
        #左下角点 (x, y)，底边右端点 ，左边上端点
        left_triangle = np.array([[0, h], [240, h], [0, 270]])
        right_triangle = np.array([[w-240, h], [w, h], [w, 270]])
        cv2.fillPoly(mask, [left_triangle, right_triangle], 0)
        closed = cv2.bitwise_and(closed, mask)
        
        # 在frame上绘制三角形
        cv2.fillPoly(frame, [left_triangle], (0, 255, 0))  # 绿色填充
        cv2.fillPoly(frame, [right_triangle], (0, 255, 0))  # 绿色填充
        # 绘制三角形轮廓
        cv2.polylines(frame, [left_triangle], True, (255, 0, 0), 2)  # 蓝色轮廓
        cv2.polylines(frame, [right_triangle], True, (255, 0, 0), 2)  # 蓝色轮廓
        

    cv2.imshow("closed",closed)
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
    # print("detx_p:",detx_p,"dety_p:",dety_p,"flag:",flag)
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
    gamma = 0.5
    invgamma = 1 / gamma
    gamma_image = np.array(np.power((gray / 255.0), invgamma) * 255, dtype=np.uint8)
    # equalized = cv2.equalizeHist(gray)
    blurred = cv2.GaussianBlur(gamma_image, (9, 9), 2)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    opened = cv2.morphologyEx(blurred, cv2.MORPH_OPEN, kernel)
    closed1 = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    closed = cv2.morphologyEx(closed1, cv2.MORPH_CLOSE, kernel)
    edges = cv2.Canny(closed, 50, 150)
    closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    cv2.imshow("blurred",blurred)
    cv2.imshow("opened",opened)
    cv2.imshow("closed",closed)
    cv2.imshow("edges",closed_edges)
    
    # 轮廓分析
    contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
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
    
    if largest_area > 1000 and largest_circle is not None:
        (x, y), radius = cv2.minEnclosingCircle(largest_circle)
        center = (int(x), int(y))
        radius = int(radius)
        
        # 绘制结果
        cv2.drawContours(src1, [largest_circle], 0, (0, 0, 255), 3)
        cv2.circle(src1, center, 2, (0, 0, 255), 3)
        cv2.circle(src1, center, radius, (0, 255, 0), 2)
        center_text = f"({center[0]}, {center[1]}), radius: {radius}"
        text_position = (center[0] + 10, center[1] - 10)
        area_text=f"({largest_area})"
        cv2.putText(src1, center_text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.putText(src1, area_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
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
    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    # opened = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
    # closed1 = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    # closed = cv2.morphologyEx(closed1, cv2.MORPH_CLOSE, kernel)
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

def findGoodsCenter(color_cap,color_number):  #爪子夹不紧时 识别所抓物料中心值
    '''爪子夹不紧时 识别所抓物料中心值'''
    # 获取图像帧
    ret, frame = color_cap.read()
    
    if not ret:
        return 0, 0, None, 0, 0, 0
    
    # 预处理（指定颜色）
    closed, _ = preprocess_image(frame, color_number=color_number)
    cv2.imshow("closed",closed)
    # 分析轮廓（选择最上方的色块）
    h, w = frame.shape[:2]
    contours, _ = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    num = 0
    x_center = 0
    y_center = 0
    c = 0
    detx=10000
    dety=10000
    detx_p=0
    dety_p=0
    flag_color_1 = 0
    #左上 460 140    右下 910 540？？？
    for cnt343 in contours:
        (x1, y1, w1, h1) = cv2.boundingRect(cnt343)  
        area = cv2.contourArea(cnt343)
        if w1*h1 > 0.05*w*h:
        # if area > 0.07*w*h:
            peri = cv2.arcLength(cnt343, True)
            approx = cv2.approxPolyDP(cnt343, 0.02 * peri, True)
            cv2.drawContours(frame, [approx], 0, (0, 0, 255), 3)
            (x, y), radius = cv2.minEnclosingCircle(approx)
            center = (int(x), int(y))
            radius = int(radius) 
            cv2.circle(frame, center, 2, (0, 0, 255), 3)
            cv2.circle(frame, center, radius, (0, 255, 0), 2)
            a = x1 + w1 / 2
            b = y1 + h1 / 2
            num += 1
            # area_text=f"{w1*h1}"
            # cv2.putText(frame, area_text, (x1+60, y1 +h1+ 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if num == 1 or c < y1:
                x_center = a
                y_center = b
                c = y1
            # x_center=int(round(x))
            # y_center=int(round(y))
            flag_color_1 = 1
            detx = x - w/2 
            dety = h/2 - y
            detx_p = int(round(detx))
            dety_p = int(round(dety))
            print("detx:",detx,"dety:",dety)
    cv2.imshow("frame",frame)
    cv2.waitKey(1)
    return x_center,y_center,flag_color_1,detx_p,dety_p


def updateCorrectxy(color_cap,color_number):   #爪子夹不紧时 更新中心位置偏差值
    detx_,dety_,flag_=0,0,0
    for i in range(3):
        x_update,y_update,flag_update,detx_update,dety_update=findGoodsCenter(color_cap,color_number)
        if flag_update==1:
            detx_+=detx_update
            dety_+=dety_update
            flag_+=flag_update
    detx_ave=detx_/flag_
    dety_ave=dety_/flag_
    global correct_x_hough
    global correct_y_hough
    correct_x_hough=detx_ave
    correct_y_hough=dety_ave
    print("correct_x_update:",correct_x_hough,"correct_y_update",correct_y_hough)
    # return 0

def defaltCorrectxy():
    global correct_x_hough
    global correct_y_hough
    correct_x_hough=correct_x_hough_default
    correct_y_hough=correct_y_hough_default




def detect_plate_stop_before(cap, detector_func, times, stop_threshold, 
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
        
        if flag:
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
        if x_avg >0.2 and x_avg<0.8:
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
    
def detect_plate_stop(cap, detector_func, times, stop_threshold, 
                               min_success_rate=1, # 新增：最低成功率阈值
                               check_direction=False, direction_threshold=0.02, 
                               **detector_args):
    """
    # 通用转盘停止检测函数 (修订版)
    :param cap: 摄像头对象
    :param detector_func: 检测函数
    :param times: 采样次数
    :param stop_threshold: 停止阈值 (现在表示最大坐标偏移)
    :param min_success_rate: 允许的最低检测成功率
    :param check_direction: 是否检查方向
    :param direction_threshold: 方向判断阈值
    :param detector_args: 传递给检测函数的参数
    :return: 是否停止的标志
    """
    detected_positions = []  # 用于存储所有成功检测到的(x, y)坐标
    
    for _ in range(times):
        result = detector_func(cap, **detector_args)
        
        # 统一提取 x, y, flag
        if len(result) >= 4:
            x, y, _, flag = result[:4]
        else:
            # 如果检测函数返回格式不匹配，则认为失败
            flag = 0
            x, y = 0, 0
            
        if flag == 1:
            detected_positions.append((x, y))
        
        cv2.waitKey(1)

    # --- 判断逻辑开始 ---
    
    success_count = len(detected_positions)
    min_detections = int(times * min_success_rate)
    
    print(f"成功检测到 {success_count}/{times} 次")
    
    # 1. 检查成功次数是否达标
    if success_count < min_detections:
        print("检测成功率太低，判断为运动中。")
        # 即使成功率低，如果开启了方向检测，我们仍然可以尝试判断方向
        if check_direction and success_count >= 2:
            first_x = detected_positions[0][0]
            last_x = detected_positions[-1][0]
            if (last_x - first_x) > direction_threshold:
                return 0, True # 顺时针（假设x增大为顺时针）
            elif (last_x - first_x) < -direction_threshold:
                return 0, False # 逆时针
        if check_direction :
            return 0,None
        else:
            return 0

    # 2. 计算所有成功检测点的平均位置
    # 使用 np.mean 可以方便地计算 x 和 y 的平均值
    x_coords = [pos[0] for pos in detected_positions]
    y_coords = [pos[1] for pos in detected_positions]
    x_avg = np.mean(x_coords)
    y_avg = np.mean(y_coords)
    
    # 3. 检查所有点是否都靠近平均点 (使用最大偏移量)
    max_deviation = 0
    for x, y in detected_positions:
        deviation = np.sqrt((x - x_avg)**2 + (y - y_avg)**2) # 欧氏距离
        if deviation > max_deviation:
            max_deviation = deviation
            
    print(f"坐标平均值: ({x_avg:.4f}, {y_avg:.4f}), 最大偏移: {max_deviation:.4f}")

    flag_stop = 0
    turn_direction = None # 默认方向为None

    # 4. 判断是否停止
    if max_deviation < stop_threshold:
        # 增加一个额外条件，防止在图像边缘附近误判
        if 0.2 < x_avg < 0.8 :
            flag_stop = 1
            print("检测到停止！")
    else:
        # 5. 如果没有停止，则判断方向
        if check_direction:
            first_x = detected_positions[0][0]
            last_x = detected_positions[-1][0]
            if (last_x - first_x) > direction_threshold:
                turn_direction = True # 顺时针
                print("判断为运动方向：顺时针")
            elif (last_x - first_x) < -direction_threshold:
                turn_direction = False # 逆时针
                print("判断为运动方向：逆时针")

    if check_direction:
        return flag_stop, turn_direction
    else:
        return flag_stop


def detectPlate(cap, color_number):
    """检测转盘是否停止（从转盘上夹走物料）"""
    stop_flag = detect_plate_stop(
        cap=cap,
        detector_func=findBlockCenter,
        times=5,
        stop_threshold=0.01,
        min_success_rate = 0.8,
        color_number=color_number
    )
    return stop_flag 

def detectPlate_check(cap, color_number):
    """检测爪子是否成功抓起物料"""
    stop_flag = detect_plate_stop(
        cap=cap,
        detector_func=findBlockCenter,
        times=3,
        stop_threshold=0.01,
        color_number=color_number,
        is_check=1
    )
    return stop_flag

def detectPlate_gray(cap):
    """检测转盘是否停止（灰度处理）-色块"""
    stop_flag, direction =  detect_plate_stop(
        cap=cap,
        detector_func=findBlockCenter_gray,
        times=5,
        stop_threshold=0.01,
        min_success_rate = 0.8,
        check_direction=True
    )
    return stop_flag, direction

def detectPlate_nocolor_ring(cap):
    """检测转盘是否停止（灰度处理）-无颜色圆环"""
    stop_flag= detect_plate_stop(
        cap=cap,
        detector_func=enhance_and_find_ring_new,
        times=5,
        stop_threshold=0.01,
        min_success_rate = 0.8,
        check_direction=False
    )
    return stop_flag

def detectPlate_circle(cap, color_number):
    """检测转盘是否停止（圆环检测）"""
    stop_flag, direction = detect_plate_stop(
        cap=cap,
        detector_func=findBlockCenter_circle,
        times=3,
        stop_threshold=0.01,
        check_direction=True,
        color_number=color_number
    )
    return stop_flag, direction



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


    # if abs(finaltheta)<0.5:
    #     line_flag=1
    # finaltheta=int(round(finaltheta))
    # if (finaltheta==90 ):
    #     finaltheta=0


    TARGET_ANGLE = 0  # 定义目标角度
    theta_error = finaltheta - TARGET_ANGLE
    if abs(theta_error) < 0.5:
        line_flag=1
    theta_to_return=int(round(theta_error*10))
    if (finaltheta==90 ):
        theta_to_return=0
    print("theta_error:",theta_error,"theta_to_return:",theta_to_return/10)

    cv2.waitKey(1)
    return theta_to_return,line_flag

def apply_temporal_filter(current_results,smooth_factor=0.4):
    """
    应用时间滤波器，对当前检测结果进行平滑处理。
    
    参数:
    current_results (list): 当前检测结果列表，每个元素为(x, y, r)
    
    返回:
    list: 经过时间滤波处理后的检测结果列表
    """
    global prev_centers

    if not prev_centers:  # 首次检测
        prev_centers = current_results.copy()
        return current_results

    filtered_results = []

    # 为每个当前检测结果找到对应的历史结果
    for curr_x, curr_y, curr_r in current_results:
        # 查找最近的历史点
        best_match = None
        min_dist = float('inf')
        
        for i, (prev_x, prev_y, prev_r) in enumerate(prev_centers):
            dist = ((curr_x - prev_x)**2 + (curr_y - prev_y)**2)**0.5
            if dist < min_dist:
                min_dist = dist
                best_match = (i, prev_x, prev_y, prev_r)
        
        # 如果找到匹配点且距离合理，应用平滑
        if best_match and min_dist < curr_r*0.25:  # 阈值可调整
            i, prev_x, prev_y, prev_r = best_match
            # 指数平滑
            smooth_x = smooth_factor * prev_x + (1 - smooth_factor) * curr_x
            smooth_y = smooth_factor * prev_y + (1 - smooth_factor) * curr_y
            smooth_r = smooth_factor * prev_r + (1 - smooth_factor) * curr_r
            
            filtered_results.append((smooth_x, smooth_y, smooth_r))
            # 更新历史点
            prev_centers[i] = (smooth_x, smooth_y, smooth_r)
        else:
            # 新检测点，直接添加
            filtered_results.append((curr_x, curr_y, curr_r))
            prev_centers.append((curr_x, curr_y, curr_r))
    
    # 移除未匹配的历史点
    if filtered_results:
        new_prev_centers = []
        for prev_point in prev_centers:
            for curr_point in filtered_results:
                px, py, pr = prev_point
                cx, cy, cr = curr_point
                if ((px - cx)**2 + (py - cy)**2)**0.5 < cr*0.25:
                    new_prev_centers.append(prev_point)
                    break
        prev_centers = new_prev_centers
    
    return filtered_results


def simple_cluster(points, eps=20, min_samples=2):
    """
    简单的聚类实现（替代DBSCAN）
    :param points: 点集，形状为(N,2)
    :param eps: 邻域半径
    :param min_samples: 最小样本数
    :return: 簇标签列表
    """
    labels = np.zeros(len(points)) - 1  # 初始化为-1（噪声）
    cluster_id = 0
    
    for i in range(len(points)):
        if labels[i] != -1:  # 已分类
            continue
            
        # 找到邻域内的点
        neighbors = []
        for j in range(len(points)):
            if np.linalg.norm(points[i] - points[j]) < eps:
                neighbors.append(j)
                
        if len(neighbors) < min_samples:
            labels[i] = -1  # 标记为噪声
        else:
            # 创建新簇
            for n in neighbors:
                if labels[n] == -1:  # 只分配未分类的点
                    labels[n] = cluster_id
            cluster_id += 1
            
    return labels


# 添加全局变量控制是否已保存图片
has_saved_image = False

def code(code_cap):  #识别二维码、条形码
    '''识别二维码、条形码'''
    global has_saved_image
    ret,frame = code_cap.read()
    ret,frame = code_cap.read()
    ret,frame = code_cap.read()
    if code_cap.isOpened():
        # # 只在未保存过图片时保存
        # if not has_saved_image and ret:
        #     timestamp = time.strftime("%Y%m%d_%H%M%S")
        #     if not os.path.exists('captured_images'):
        #         os.makedirs('captured_images')
        #     cv2.imwrite(f'captured_images/code_image_{timestamp}.jpg', frame)
        #     print(f"保存图片: code_image_{timestamp}.jpg")
        #     has_saved_image = True
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
    cv2.waitKey(1)

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
    """
    使用readline()安全地读取一行数据。
    会自动处理消息分片到达的问题。
    """
    # readline()会阻塞直到收到'\n'或超时
    # .strip()会去除首尾的空白符，包括'\n'和可能存在的空格
    recv_data = ser.readline().strip()
    print("receivemessage:",recv_data)
    if not recv_data:
        return None
    return recv_data
    # count = ser.inWaiting()
    # if count != 0:
    #     recv = ser.read(count) 
    #     recv_data=recv
    # else:
    #     recv_data = None
    # if recv_data != None:
    #     print("receivemessage:",recv_data)
    # ser.flushInput()
    # time.sleep(0.01)
    # return recv_data

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
    data_l = min(abs(data_l), 255)
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





def find_inner_circle_on_cylinder(cap, color_number, hough=1):
    """
    在一个帧中寻找红、绿、蓝物料，并检测其顶部小圆柱的圆形边缘。
    """
    ret, frame = cap.read()
    # 1. 颜色分割以定位物体 (与之前相同)
    red_min = np.array(dim_red_min)
    red_max = np.array(dim_red_max)
    green_min = np.array(dim_green_min1)
    green_max = np.array(dim_green_max1)
    blue_min = np.array(dim_blue_min)
    blue_max = np.array(dim_blue_max)
    red_min1 = np.array(dim_red_min1)
    red_max1 = np.array(dim_red_max1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask12 = cv2.inRange(hsv, red_min, red_max)
    mask11 = cv2.inRange(hsv, red_min1, red_max1)
    mask1 = cv2.add(mask12, mask11)
    mask2 = cv2.inRange(hsv, green_min, green_max)
    mask3 = cv2.inRange(hsv, blue_min, blue_max)
    if color_number == 1:
        mask0 = mask1
    elif color_number == 2:
        mask0 = mask2
    elif color_number == 3:
        mask0 = mask3
    
    combined_mask = mask0
    # combined_mask = cv2.bitwise_or(mask_red, mask_green)
    # combined_mask = cv2.bitwise_or(combined_mask, mask_blue)
    
    kernel = np.ones((7, 7), np.uint8)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

    # 2. 寻找每个物料的轮廓，并创建ROI
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    output_frame = frame.copy()
    detx = 0
    dety = 0
    flag = 0

    for cnt in contours:
        if cv2.contourArea(cnt) < 5000:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        roi_original = frame[y:y+h, x:x+w]
        
        if roi_original.size == 0:
            continue

        cv2.rectangle(output_frame, (x, y), (x+w, y+h), (255, 255, 0), 2)
        # 3. 在ROI内进行细节增强
        gray_roi = cv2.cvtColor(roi_original, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced_roi = clahe.apply(gray_roi)
        blurred_roi = cv2.medianBlur(enhanced_roi, 5) # 中值滤波对椒盐噪声效果好
        
        
        cv2.imshow("blurred", blurred_roi)
        

        # 4. **核心步骤：使用霍夫圆变换检测圆**
        # cv2.HoughCircles(image, method, dp, minDist, param1, param2, minRadius, maxRadius)
        # - image: 输入的灰度图
        # - method: 检测方法，一般用 cv2.HOUGH_GRADIENT
        # - dp: 累加器分辨率与图像分辨率的反比。dp=1 表示同样的分辨率。dp=2 表示累加器是图像的一半。
        # - minDist: 检测到的圆心之间的最小距离。这是为了防止在同一个圆上检测到多个"邻居"圆。
        # - param1: Canny边缘检测的高阈值（低阈值是它的一半）。
        # - param2: 累加器阈值。这个值越小，能检测到的圆越多（包括假的）。
        # - minRadius, maxRadius: 圆半径的最小和最大值。这是非常有用的过滤器！
        
        #使用霍夫圆办法
        if hough == 1:
            circles = cv2.HoughCircles(
                blurred_roi,
                cv2.HOUGH_GRADIENT,
                dp=1.2,
                minDist=h,  # 在一个ROI里只找一个圆，所以minDist设为ROI的高度即可
                param1=100, # Canny边缘检测的高阈值
                param2=30,  # 累加器阈值，这个值需要仔细调
                minRadius=int(w / 8), # 根据你的物料大致尺寸设定
                maxRadius=int(w / 3)  # 根据你的物料大致尺寸设定
            )
            # 绘制检测到的圆
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    center_x = i[0] + x
                    center_y = i[1] + y
                    radius = i[2]
                    detx = int(round(center_x - w/2 - correct_x_hough))
                    dety = int(round(h/2 - center_x - correct_y_hough))
                    cv2.circle(output_frame, (center_x, center_y), 3, (0, 255, 0), -1)
                    cv2.circle(output_frame, (center_x, center_y), radius, (0, 0, 255), 3)
                flag = 1
        #多边形逼近
        else:
            edges_roi = cv2.Canny(blurred_roi, 40, 120)
            kernel = np.ones((7, 7), np.uint8)
            closed_edges = cv2.morphologyEx(edges_roi, cv2.MORPH_CLOSE, kernel)
            cv2.imshow("edges",closed_edges)
            inner_contours, _ = cv2.findContours(closed_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for inner_cnt in inner_contours:
                area = cv2.contourArea(inner_cnt)
                if area < 100: # 过滤小噪声
                    continue
                
                perimeter = cv2.arcLength(inner_cnt, True)
                if perimeter == 0:
                    continue
                    
                circularity = (4 * np.pi * area) / (perimeter * perimeter)
                
                # 筛选出圆度接近1的轮廓
                if 0.85 < circularity < 1.1: # 设定一个合理的范围
                    # 这就是你要找的圆！
                    # 转换坐标并绘制
                    inner_cnt[:, :, 0] += x
                    inner_cnt[:, :, 1] += y
                    detx = int(round(inner_cnt[:, :, 0] - w/2 - correct_x_hough))
                    dety = int(h/2 - round(inner_cnt[:, :, 1] - correct_y_hough))
                    cv2.drawContours(output_frame, [inner_cnt], -1, (255, 0, 255), 2)
                    flag = 1
    cv2.imshow("frame",output_frame)
    cv2.waitKey(1)
    return flag, detx, dety


def enhance_and_find_ring(cap):
    """
    从摄像头捕获图像，识别白纸上的黑色细环，并直接显示处理过程和结果。
    
    该函数直接操作传入的摄像头对象，并在内部处理图像显示，无返回值。
    
    :param cap: cv2.VideoCapture 对象，即打开的摄像头。
    """

    
    # 1. 从摄像头捕获一帧图像
    ret, frame = cap.read()
    if not ret:
        print("错误：无法从摄像头读取帧。")
        return 0, 0, None, 0, 0, 0  # 返回默认值

    # 创建一个副本用于绘制，以保留原始图像的清洁
    output_frame = frame.copy()
    h,w =frame.shape[:2]

    # --- 图像处理流程 ---

    # 2. 灰度化
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
    # 5. 黑帽操作，突出比周围暗的细小结构 (黑色细环)
    #    内核尺寸应略大于环的线宽
    kernel_blackhat = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
    blackhat = cv2.morphologyEx(blurred, cv2.MORPH_BLACKHAT, kernel_blackhat)
    edges = cv2.Canny(blackhat, 50, 150)
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel_close)
    contours, hierarchy = cv2.findContours(closed_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    found_rings = []
    detx = 0
    dety = 0
    is_found = 0
    x_center = 0
    y_center = 0

    if hierarchy is not None:
        # 遍历所有轮廓，寻找有子轮廓的父轮廓（即环）
        for i, contour in enumerate(contours):
            # hierarchy 格式: [Next, Previous, First_Child, Parent]
            h = hierarchy[0][i]
            # 条件：这是一个最外层的轮廓(没有父级) 并且 它有一个子轮廓
            if h[3] == -1 and h[2] != -1:
                child_contour = contours[h[2]]
                outer_area = cv2.contourArea(contour)
                inner_area = cv2.contourArea(child_contour)
                
                # 面积筛选：排除太小或太大的噪声
                if outer_area < 500 or inner_area < 100:
                    continue

                # 圆度筛选：确保形状接近圆形 (可选但强烈推荐)
                peri_outer = cv2.arcLength(contour, True)
                if peri_outer == 0: continue
                circularity_outer = (4 * np.pi * outer_area) / (peri_outer * peri_outer)
                
                if circularity_outer > 0.7: # 阈值可调，越接近1越圆
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    found_rings.append({
                        'center': (int(x), int(y)),
                        'radius': int(radius),
                        'outer_contour': contour,
                        'area': outer_area
                    })

    # --- 绘制结果 ---
    if found_rings:
        # 如果找到多个符合条件的环，选择面积最大的那个
        best_ring = max(found_rings, key=lambda r: r['area'])
        print("bestring:", best_ring)
        
        center = best_ring['center']
        print("center:", center)
        radius = best_ring['radius']
        cv2.drawContours(output_frame, [best_ring['outer_contour']], -1, (0, 255, 0), 2)
        cv2.circle(output_frame, center, 5, (0, 0, 255), -1)
        cv2.circle(output_frame, center, radius, (0, 0, 255), 2)
        
        info_text = f"Center: {center}, R: {radius}"
        cv2.putText(output_frame, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        # detx = int(round(center[0] - w/2 - correct_x))
        # dety = int(round(h/2 - center[1] - correct_y))
        # 使用 np.round() 来进行四舍五入
        # 确保center是标量值
        center_x: int
        center_y: int
        center_x, center_y = center
        print("center_x, center_y:", center_x, center_y)
        # 使用 astype 确保是标量值
        center_x = center_x.astype(int) if hasattr(center_x, 'astype') else int(center_x)
        center_y = center_y.astype(int) if hasattr(center_y, 'astype') else int(center_y)
        # detx = int(round(center_x - w/2 - correct_x))
        # dety = int(round(h/2 - center_y - correct_y))
        detx = int(center_x - w/2 - correct_x)
        dety = int(h/2 - center_y - correct_y)
        is_found = 1
        x_center = center_x
        y_center = center_y

    # --- 显示所有图像 ---
    # cv2.imshow("Original", frame)
    cv2.imshow("Detected Ring", output_frame)
    # cv2.imshow("Grayscale", gray)
    # cv2.imshow("Blackhat (Ring Highlighted)", blackhat)
    # cv2.imshow("Closed Edges (For Contours)", closed_edges)
    cv2.waitKey(1)
    return x_center/w, y_center/h, output_frame, is_found, detx, dety


def enhance_and_find_ring_new(cap):
    """
    从摄像头捕获图像，识别最佳的圆环，并返回其相关信息。
    本函数风格类似于 findBlockCenter，不使用字典存储中间结果。
    """
    
    # 1. 从摄像头捕获一帧图像
    ret, frame = cap.read()
    if not ret:
        print("错误：无法从摄像头读取帧。")
        # 返回与成功时相同数量和类型的默认值
        return 0.0, 0.0, None, 0, 0, 0

    # 创建一个副本用于绘制
    output_frame = frame.copy()
    h, w = frame.shape[:2]

    # --- 图像处理流程 (保持不变) ---
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
    kernel_blackhat = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
    blackhat = cv2.morphologyEx(blurred, cv2.MORPH_BLACKHAT, kernel_blackhat)
    edges = cv2.Canny(blackhat, 50, 150)
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel_close)
    contours, hierarchy = cv2.findContours(closed_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # --- 初始化用于存储“最佳”圆环信息的变量 ---
    best_center_x = 0
    best_center_y = 0
    best_radius = 0
    best_area = 0  # 用于比较找到最大面积的环
    best_contour = None
    is_found = 0

    if hierarchy is not None:
        # --- 遍历所有轮廓，直接寻找并更新“最佳”圆环 ---
        for i, contour in enumerate(contours):
            h_info = hierarchy[0][i]
            # 条件：最外层轮廓且有子轮廓
            if h_info[3] == -1 and h_info[2] != -1:
                child_contour = contours[h_info[2]]
                outer_area = cv2.contourArea(contour)
                inner_area = cv2.contourArea(child_contour)
                
                # 面积筛选
                if outer_area < 500 or inner_area < 100:
                    continue

                # 圆度筛选
                peri_outer = cv2.arcLength(contour, True)
                if peri_outer == 0: continue
                circularity_outer = (4 * np.pi * outer_area) / (peri_outer * peri_outer)
                
                if circularity_outer > 0.7:
                    # --- 找到了一个合格的候选圆环 ---
                    # 如果当前这个环的面积比我们记录的“最佳”面积还要大
                    if outer_area > best_area:
                        # 更新所有“最佳”信息
                        best_area = outer_area
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        best_center_x = int(x)
                        best_center_y = int(y)
                        best_radius = int(radius)
                        best_contour = contour
    
    # --- 根据是否找到最佳圆环来计算最终结果 ---
    detx = 0
    dety = 0
    
    if best_contour is not None:
        # 如果 best_contour 不是 None，说明我们至少找到了一个合格的环
        is_found = 1
        
        # 使用记录下来的最佳圆环信息进行计算
        detx = int(best_center_x - w/2 - correct_x)
        dety = int(h/2 - best_center_y - correct_y)
        
        # 在图像上绘制最终选定的那个最佳圆环
        best_center_tuple = (best_center_x, best_center_y)
        cv2.drawContours(output_frame, [best_contour], -1, (0, 255, 0), 2)
        cv2.circle(output_frame, best_center_tuple, 5, (0, 0, 255), -1)
        cv2.circle(output_frame, best_center_tuple, best_radius, (0, 0, 255), 2)
        
        info_text = f"Center: {best_center_tuple}, R: {best_radius}"
        cv2.putText(output_frame, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

    # --- 显示和返回 ---
    cv2.imshow("Detected Ring", output_frame)
    cv2.waitKey(1)
    
    # 无论是否找到，都返回6个值，以确保接口统一
    # 如果没找到，返回的将是初始化的0值
    return best_center_x/w, best_center_y/h, output_frame, is_found, detx, dety







def display_contour_areas(cap, color_number=None, min_area_threshold=500):
    """
    实时检测物体，并在屏幕上显示其面积信息。
    如果提供了 color_number，则按颜色分割。
    如果 color_number 为 None，则按灰度进行分割。

    :param frame: 原始摄像头图像。
    :param color_number: 颜色编号 (1:红, 2:绿, 3:蓝) 或 None。
    :param min_area_threshold: 最小面积过滤阈值。
    :return: 处理后带有标注信息的图像。
    """
    ret, frame = cap.read()
    if not ret:
        return
    mask = None
    
    # --- 1. 根据 color_number 决定处理方式 ---
    if color_number is not None and color_number in [1, 2, 3]:
        # --- 颜色处理流程 ---
        # 调用已有的预处理函数来获取二值掩膜
        print(f"模式: 颜色分割 (颜色: {color_number})")
        mask, _ = preprocess_image(frame, color_number=color_number)
    else:
        # --- 灰度处理流程 ---
        print("模式: 灰度分割")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        gamma = 0.5
        invgamma = 1 / gamma
        gamma_image = np.array(np.power((gray / 255.0), invgamma) * 255, dtype=np.uint8)
        # cv2.imshow("gamma", gamma_image)

        # 高斯模糊
        blurred1 = cv2.GaussianBlur(gamma_image, (9, 9), 2)
        
        # 对灰度阈值结果进行形态学操作，清理噪点
        kernel = np.ones((5, 5), np.uint8)
        closed = cv2.morphologyEx(blurred1, cv2.MORPH_CLOSE, kernel)
        opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel)
        edges = cv2.Canny(opened, 50, 150)
        mask = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        kernel1 = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel1)

        cv2.imshow("mask", mask)

    if mask is None:
        return frame # 如果掩膜生成失败，返回原图

    # 创建一个副本用于绘制，避免在原始帧上直接操作
    output_frame = frame.copy()
    
    # --- 2. 寻找、分析和绘制轮廓 (这部分逻辑是共用的) ---
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        contour_area = cv2.contourArea(cnt)
        if contour_area < min_area_threshold:
            continue
            
        # --- 计算两种外接矩形 ---
        
        # 1. 最小面积（可旋转）外接矩形
        min_rect = cv2.minAreaRect(cnt)
        min_rect_area = min_rect[1][0] * min_rect[1][1]
        box = cv2.boxPoints(min_rect)
        box = np.int0(box)
        
        # 2. 垂直（不可旋转）外接矩形
        x, y, w, h = cv2.boundingRect(cnt)
        bounding_rect_area = w * h
        
        # --- 在图像上绘制和标注 ---
        
        # 绘制真实轮廓 (绿色)
        cv2.drawContours(output_frame, [cnt], -1, (0, 255, 0), 2)
        # # 绘制最小外接矩形 (红色)
        # cv2.drawContours(output_frame, [box], 0, (0, 0, 255), 2)
        # 绘制垂直外接矩形 (蓝色)
        cv2.rectangle(output_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # 准备文本
        text_contour = f"Contour: {contour_area:.0f}"
        text_min_rect = f"MinRect: {min_rect_area:.0f}"
        text_bound_rect = f"BoundRect: {bounding_rect_area:.0f}"
        
        # 在质心位置显示文本
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = int(min_rect[0][0]), int(min_rect[0][1])

        cv2.putText(output_frame, text_contour, (cx - 80, cy - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # cv2.putText(output_frame, text_min_rect, (cx - 80, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(output_frame, text_bound_rect, (cx - 80, cy + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
    cv2.imshow("output_frame", output_frame)
    cv2.waitKey(1)