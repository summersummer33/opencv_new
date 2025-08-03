# new_testmain_pro.py (最终修正版)

import cv2
import time
import queue
import new_testfcn as testfcn
import new_testdef_pro as testdef
import threading
from functools import partial

# 导入我们的工具
from new_thread_util import StoppableThread, TaskInterruptedException

# --- 全局常量 ---
CMD_STOP = b'stop'

limit_cu_circle = 4
limit_cu_line = 0.5


def run_and_wait_for_task(handler, target_func_with_params):
    """在一个可中断的线程中运行一个已经包含了所有参数的函数对象。"""
    # 清空可能残留的旧'stop'指令
    while not handler.command_queue.empty():
        try:
            handler.command_queue.get_nowait()
        except queue.Empty:
            break

    task_thread = StoppableThread(target=target_func_with_params)
    task_thread.start()
    
    task_name = getattr(target_func_with_params, 'func', target_func_with_params).__name__
    print(f"--- 任务 '{task_name}' 已启动，等待其完成或被中断... ---")

    while task_thread.is_alive():
        try:
            cmd = handler.command_queue.get(timeout=0.05)
            if cmd == CMD_STOP:
                print(f">>> 收到中止指令，正在停止任务 '{task_name}'...")
                task_thread.raise_exception()
                task_thread.join(timeout=1.0)
                break
            else:
                print(f"警告：任务执行期间收到非中止指令 '{cmd}'，已忽略。")
        except queue.Empty:
            pass

    try:
        task_thread.join(timeout=0.1)
        print(f"--- 任务 '{task_name}' 正常完成。 ---")
    except TaskInterruptedException:
        print(f"--- 任务 '{task_name}' 已被成功中止。 ---")
    except Exception as e:
        print(f"!!! 任务 '{task_name}' 因异常而终止: {e} !!!")


def main():
    handler = testfcn.FunctionHandler()
    handler.init_camera_code()
    handler.init_camera_up()
    handler.start_serial_listener()

    try:
        while True:
            print("\n>>> 系统待机，等待新的任务指令...")
            try:
                cmd = handler.command_queue.get()
            except KeyboardInterrupt:
                break

            if cmd == CMD_STOP:
                print("在待机状态收到 'stop' 指令，已忽略。")
                continue

            # --- 指令到函数的映射表 (完整且统一使用 partial/lambda) ---
            task_map = {
                b'AA':   partial(handler.get_code),
                b'BB1':  partial(handler.get_from_plate, handler.get_order),
                b'BB2':  partial(handler.get_from_plate, handler.put_order),
                b'CC12': lambda: (handler.cu_positioning(limit_cu_circle, limit_cu_line), handler.xi_positioning(handler.get_order, run_time=3)),
                b'CC3':  lambda: (handler.cu_positioning(limit_cu_circle, limit_cu_line), handler.xi_positioning(handler.put_order, run_time=3)),
                b'CC4':  lambda: (handler.cu_positioning(limit_cu_circle, limit_cu_line), handler.xi_positioning(handler.get_order, run_time=3)),
                b'CC5':  lambda: (handler.cu_positioning(limit_cu_circle, limit_cu_line)), # 简化：CC5只做粗调
                b'EE':   partial(handler.adjust_line_gray_yellow),
                b'HH':   partial(handler.plate_adjust_then_put, handler.get_order),
                b'LL1':  partial(handler.plate_adjust_then_put_pre_color_pro, plate_order=handler.get_order, adjust_finely=0),
                b'LL2':  partial(handler.plate_adjust_then_put_pre_color_pro, plate_order=handler.put_order, adjust_finely=0),
                b'NN1':  partial(handler.get_from_plate_check_eachtime, handler.get_order, run_time=3),
                b'NN2':  partial(handler.get_from_plate_check_eachtime, handler.put_order, run_time=3),
                b'II':   partial(handler.get_from_ground_in_line),
                b'OO':   partial(handler.plate_adjust_then_put_nocolor_ring, adjust_finely=1),
                b'PP':   partial(handler.plate_adjust_then_put_nocolor_ring_for_adjust, adjust_finely=1),
                b'QQ1':  partial(handler.plate_adjust_then_put_pre_color_pro, plate_order=handler.get_order, adjust_finely=1),
                b'QQ2':  partial(handler.plate_adjust_then_put_pre_color_pro, plate_order=handler.put_order, adjust_finely=1),
                b'end':  partial(handler.reset_state),
            }

            if cmd in task_map:
                target_func_with_params = task_map[cmd]
                
                if cmd == b'end':
                    print(">>> 执行全局状态复位...")
                    target_func_with_params()
                else:
                    run_and_wait_for_task(handler, target_func_with_params)
            else:
                print(f"错误：收到未知的任务指令 '{cmd}'")
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n程序被用户中断。")
    finally:
        print("正在执行最终清理...")
        handler.cleanup()
        if handler.cap: handler.cap.release()
        if handler.code_cap: handler.code_cap.release()
        cv2.destroyAllWindows()
        print("程序已退出。")

if __name__ == "__main__":
    main()