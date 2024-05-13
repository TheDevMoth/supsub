import time
# import pymavlink
from pymavlink import mavutil
from threading import Thread, current_thread

master = mavutil.mavlink_connection('udpin:192.168.2.1:14555')

# Make sure the connection is valid
print("Waiting for heartbeat from vehicle...")
master.wait_heartbeat()
print("Heartbeat from vehicle received.")

def receive(parent_thread):
    while True:
        if not parent_thread.is_alive():
            break
        try:
            master.recv_match()
        except:
            pass
        time.sleep(0.1)
        
parent_thread = current_thread()
t = Thread(target=receive, args=(parent_thread,))
t.start()
print("Messages Thread started")

time.sleep(10)

for msg in master.messages.values():
    print(msg)
    print()