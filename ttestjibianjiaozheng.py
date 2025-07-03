#encoding=utf-8
import numpy as np
import cv2

def undistortion(img, mtx, dist):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # print('roi ', roi)

    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # crop the image
    x, y, w, h = roi
    if roi != (0, 0, 0, 0):
        dst = dst[y:y + h, x:x + w]

    return dst

if __name__ == '__main__':
    cap = cv2.VideoCapture("/dev/up_video",cv2.CAP_V4L2)

    # 锟斤拷锟截标定锟斤拷锟斤拷
    try:
        npzfile = np.load('calibrate.npz')
        mtx = npzfile['mtx']
        dist = npzfile['dist']
    except IOError:
        print("Calibration file not found. Please calibrate the camera first.")
        exit()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 锟斤拷锟斤拷锟斤拷锟?        
        corrected_frame = undistortion(frame, mtx, dist)

        # 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷识锟斤拷锟斤拷锟斤拷拇锟斤拷锟?        # 锟斤拷锟界：锟斤拷锟斤拷锟解、锟斤拷锟斤拷识锟斤拷锟?        # 确锟斤拷使锟斤拷corrected_frame锟斤拷为锟斤拷锟斤拷

        cv2.imshow('Corrected Frame', corrected_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()