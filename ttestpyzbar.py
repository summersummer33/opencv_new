import time
import cv2
from pyzbar import pyzbar
#二维码动态识别
camera=cv2.VideoCapture(0)

while True:
    grabbed,frame = camera.read()
    dst = frame
    
    # 扫描二维码
    text = pyzbar.decode(dst)

    for texts in text:
        textdate = texts.data.decode('utf-8')
        print('条码内容:'+textdate)
        
    cv2.imshow('dst',dst)
    if cv2.waitKey(1) & 0xFF == ord('q'):  
        break
 
camera.release()
cv2.destroyAllWindows()
print("123")