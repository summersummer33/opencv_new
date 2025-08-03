import cv2
import numpy as np
import math
import serial 



frameWidth = 1280
frameHeight = 720
color_cap = cv2.VideoCapture("/dev/up_video1",cv2.CAP_V4L2)
color_cap.set(3, frameWidth)
color_cap.set(4, frameHeight)
color_cap.set(cv2.CAP_PROP_BRIGHTNESS,10)
color_cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
color_cap.set(cv2.CAP_PROP_EXPOSURE, float(0.2)) 

dim_red_min =   [  0, 60 ,60]
dim_red_max =   [ 12,203, 255]
dim_green_min = [40,48,54]# 60 60
dim_green_max = [78,234,255]
# dim_green_min = [61,48,54]# 30 48 54   61/48/54 61 taida    #zhuanpan   fanghuangse
# dim_green_max = [78,234,255]#78,234,255
dim_blue_min =  [82,105,0]#100 60 80
dim_blue_max =  [120,255,255]#124 230 255
dim_red_min1 =   [  160, 50 ,50]
dim_red_max1 =   [ 180,255, 255]
color_number=3

correct_x = 0
correct_y = 0


while True:


    success, frame = color_cap.read()
    # cv2.imshow("origin",frame)

    # corrected_frame=undistortion(frame,mtx,dist)
    color_number =3  #color portion
    cv2.imshow("corrected_frame",frame)
    # cv2.imshow("Result", img)
    red_min   =  np.array(dim_red_min)   
    red_max   =  np.array(dim_red_max)
    green_min =  np.array(dim_green_min)
    green_max =  np.array(dim_green_max)
    blue_min  =  np.array(dim_blue_min)   
    blue_max  =  np.array(dim_blue_max)  
    red_min1   = np.array(dim_red_min1)  
    red_max1   = np.array(dim_red_max1)
    src1 = frame.copy()
    res1 = src1.copy()
    h, w = res1.shape[:2]
    hsv = cv2.cvtColor(src1, cv2.COLOR_BGR2HSV)    
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
    res1 = cv2.bitwise_and(src1, src1, mask=mask0)   # Ӧ���ɰ�
    cv2.imshow("res1",res1)


    gray = cv2.cvtColor(res1, cv2.COLOR_BGR2GRAY)   #ת�Ҷ�ͼ
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    edges = cv2.Canny(blurred, 50, 150)
    # circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 0.7,70,
    #                         param1=100, param2=150, minRadius=50, maxRadius=0)    #ʶ��Բ��
    flag = 0
    detx = 0 #�����Ĳ��
    dety = 0
    

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    opened = cv2.morphologyEx(blurred, cv2.MORPH_CLOSE, kernel)
    closed1 = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    closed = cv2.morphologyEx(closed1, cv2.MORPH_CLOSE, kernel)
    edges1 = cv2.Canny(closed1, 50, 100)
    cv2.imshow("closed",closed)
    cv2.imshow("edges1",edges1)
    contours, _ = cv2.findContours(edges1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# ��ʼ������
    largest_circle = None
    largest_area = 0

    move_flag = 0


    for contour in contours:
    # �������������
    # ���������С����
        area = cv2.contourArea(contour)
        # print("area:",area)
        if area > largest_area:
            largest_area = area
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
            if len(approx) > 2:  # Բ�εĽ��ƶ���α���Ӧ�ô���8
                largest_circle = approx
                # print("largest area:",largest_area)
                if largest_area > 10000 :
                    cv2.drawContours(res1, [largest_circle], 0, (0, 0, 255), 3)
                    # ����Բ�ĺͰ뾶
                    (x, y), radius = cv2.minEnclosingCircle(largest_circle)
                    center = (int(x), int(y))
                    radius = int(radius) 
                    detx = x - w/2 - correct_x   #681   348
                    dety = h/2 - y - correct_y
                    print("detx=",detx,"dety=",dety)
                    detx = int(round(detx))
                    dety = int(round(dety))
                    cv2.circle(res1, center, 2, (0, 0, 255), 3)
                    # ����Բ
                    cv2.circle(res1, center, radius, (0, 255, 0), 2)
                    center_text = f"({center[0]}, {center[1]}), radius: {radius}"
                    text_position = (center[0] + 10, center[1] - 10)
                    area_text=f"({largest_area})"
                    cv2.putText(res1, center_text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    cv2.putText(res1, area_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    # detx_bytes=detx.to_bytes(4,'little')
                    # dety_bytes=dety.to_bytes(4,'little')
                    # packed_data = detx_bytes + dety_bytes
                    # ser.write(packed_data)
                    print('  detx:',detx,'  dety:',dety,'radius:',radius)
                    if detx>0 and dety>0:
                        move_flag = 3
                        # ser.write(b'3')
                    elif detx>0 and dety<0:
                        move_flag = 2
                        # ser.write(b'2')
                    elif detx<0 and dety>0:
                        move_flag = 4
                        # ser.write(b'4')
                    elif detx<0 and dety<0:
                        move_flag = 1 
                        # ser.write(b'1')
                    # move_flag = hex(move_flag)
                    # move_byte=move_flag.to_bytes(4,'')
                    # ser.write(b'')
                else:
                    cv2.putText(res1, 'no', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    # print("no")
                    # ser.write(b'no circle')
                








    cv2.imshow("res1",res1)
    





    if cv2.waitKey(1) & 0xFF == ord('q'):
        break