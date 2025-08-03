import serial
import time
#open serial
ser = serial.Serial("/dev/ttyAMA2", 115200)#set up serial
def main():
    while True:
        # 获得接收缓冲区字符
        # count = ser.inWaiting()
        # if count != 0:
        #     # 读取内容并回显
        #     recv = ser.read(count)  #树莓派串口接收数据
        #     print(recv)
        #     ser.write(recv)         #树莓派串口发送数据
        # # 清空接收缓冲区
        # ser.flushInput()
        # # 必要的软件延时
        # time.sleep(0.1)
        recv_data = ser.readline().strip()
        print("receivemessage:",recv_data)
        # if not recv_data:
        #     return None
        # return recv_data
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        if ser != None:
            ser.close()
