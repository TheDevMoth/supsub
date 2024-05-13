import time
# import pymavlink
from pymavlink import mavutil
from threading import Thread, current_thread

master = mavutil.mavlink_connection('udpin:192.168.2.1:14555')

# Make sure the connection is valid
print("Waiting for heartbeat from vehicle...")
master.wait_heartbeat()
print("Heartbeat from vehicle received.")

# start a new thread to receive messages
# def receive(parent_thread):
#     while True:
#         if not parent_thread.is_alive():
#             break
#         try:
#             master.recv_match()
#         except:
#             pass
#         time.sleep(0.1)
        
# parent_thread = current_thread()
# t = Thread(target=receive, args=(parent_thread,))
# t.start()
# print("Messages Thread started")

# print the any incoming "ATTITUDE" messages
print("Waiting for the vehicle to arm")
while not master.motors_armed:
    master.arducopter_arm()
    master.motors_armed_wait()
print('Armed!')

while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    print(msg)


