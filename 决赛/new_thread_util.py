# thread_util.py

import threading
import ctypes

class TaskInterruptedException(BaseException):
    """自定义的任务中断异常。使用BaseException是为了避免被普通的'except Exception'捕获。"""
    pass

class StoppableThread(threading.Thread):
    """一个可以从外部被引发异常来停止的线程类。"""
    
    def _get_tid(self):
        """获取线程ID，这是实现中断的关键。"""
        if not self.is_alive():
            raise threading.ThreadError("线程未启动或已结束")
        
        # 从线程对象的内部属性直接获取ID
        if hasattr(self, "_thread_id"):
            return self._thread_id
        
        # 如果内部属性不存在，则遍历活动线程列表查找
        for tid, tobj in threading._active.items():
            if tobj is self:
                self._thread_id = tid
                return tid
        
        raise AssertionError("无法确定线程ID")

    def raise_exception(self):
        """在目标线程中引发一个 TaskInterruptedException 异常来中断它。"""
        try:
            tid = self._get_tid()
            res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(tid), ctypes.py_object(TaskInterruptedException))
            if res == 0:
                # 这种情况可能发生在线程刚结束但is_alive()还没来得及更新时
                print(f"警告: 尝试中断线程 {tid} 时，线程ID无效或已不存在。")
            elif res > 1:
                # 如果返回了多个目标，说明有严重问题，撤销操作
                ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(tid), 0)
                raise SystemError("PyThreadState_SetAsyncExc 调用失败")
        except Exception as e:
            print(f"中断线程时发生错误: {e}")

    def run(self):
        """重写run方法，用于捕获线程内部的异常。"""
        self.exc = None
        try:
            super().run()
        except BaseException as e:
            # 捕获所有异常，包括我们注入的中断
            self.exc = e

    def join(self, timeout=None):
        """重写join方法，在线程结束后重新引发内部异常。"""
        super().join(timeout)
        if self.exc:
            # 如果线程是因为异常结束的，我们把这个异常在主线程中再次抛出
            # 这样主线程就能知道任务是成功还是失败了
            raise self.exc