import time
from multiprocessing import Process, Queue, Value, Lock, Array
import sys
import subprocess

def tilt(run_flag):
    start_time = time.time()
    while run_flag.value :
        print("1")
        time.sleep(1)

def wheels(run_flag):
    start_time = time.time()
    while run_flag.value :
        print("2")
        time.sleep(1)        
        
if __name__ == '__main__': 
    run_flag = Value('i',1)
    p0 = Process(target=tilt, args=(run_flag))
    p1 = Process(target=wheels, args=(run_flag))
    p0.start()
    p1.start()
    run_flag.value = 0
    print("finish sleep")
    p0.join()
    p1.join()