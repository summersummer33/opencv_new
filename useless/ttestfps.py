import cv2
import time

def get_realtime_fps(camera_index=0, window_title="Camera FPS"):
    # 打开摄像头
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(3, 1280)#��
    cap.set(4, 720)#��
    # cap.set(3, 640)#��
    # cap.set(4, 480)#��
    if not cap.isOpened():
        print("Error: 无法打开摄像头")
        return

    # 初始化变量
    frame_count = 0
    start_time = time.time()
    fps = 0

    while True:
        # 读取帧
        ret, frame = cap.read()
        if not ret:
            print("Error: 无法获取帧")
            break

        # 帧计数器
        frame_count += 1

        # 计算经过的时间
        elapsed_time = time.time() - start_time

        # 每秒计算一次帧率
        if elapsed_time >= 1.0:
            fps = frame_count / elapsed_time
            frame_count = 0
            start_time = time.time()

        # 在画面上显示FPS
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 显示画面
        cv2.imshow(window_title, frame)

        # 按q退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    get_realtime_fps()