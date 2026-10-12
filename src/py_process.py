import multiprocessing
import time
import os
print(multiprocessing.get_start_method())
def sing(num,name):
    print(f"唱的进程编号为：{os.getpid()}") # 获取当前进程编号
    print(f"唱的父进程编号为：{os.getppid()}") # 获取当前父进程编号
    for i in range(num):
        print(f"{name}在唱歌")
        time.sleep(1.0)
def dance(num,name):
    print(f"跳的进程编号为：{os.getpid()}") # 获取当前进程编号  
    print(f"跳的父进程编号为：{os.getppid()}") # 获取当前父进程编号    
    for i in range(num):
        print(f"{name}在跳舞")
        time.sleep(1.0)
# 主进程、父进程
if __name__ == '__main__':
    print(f"主进程、父进程的编号为：{os.getpid()}")
    # 元组传参，元组方式传参，如果只有一个参数必须要加逗号，注意传参顺序
    sing_process = multiprocessing.Process(target=sing,args=(3,"鸽鸽"))
    # 字典传参
    dance_process = multiprocessing.Process(target=dance,kwargs={"num":2,"name":"哥哥"})
    sing_process.start()
    dance_process.start()

    sing_process.join()
    dance_process.join()

    print("主进程结束")

