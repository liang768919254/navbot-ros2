import multiprocessing
import time

def chat(num,name):
    for i in range(num):
        print(f"和{name}在聊天")
        time.sleep(0.2)

if __name__ == '__main__':
    # 默认主进程会等待所有子进程结束后再结束。而我们需要在主进程结束后，子进程也结束。
    # 设置守护主进程：可以实现主进程结束，子进程也结束
    # 设置守护主进程 方法1：daemon = True
    # chat_process = multiprocessing.Process(target=chat,args=(10,"哥哥"),daemon=True)
    # chat_process.start()
    # time.sleep(1.0)
    # print("主进程、父进程结束！")

    # 设置守护进程 方法2：chat_process.daemon = True
    chat_process = multiprocessing.Process(target=chat,args=(10,"哥哥"))
    chat_process.daemon = True
    chat_process.start()
    time.sleep(1.0)
    print("主进程、父进程结束！")    



