import time
# import pymavlink
from pymavlink import mavutil
from threading import Thread, current_thread

master = mavutil.mavlink_connection('udpin:192.168.2.1:14555')

# Make sure the connection is valid
print("Waiting for heartbeat from vehicle...")
master.wait_heartbeat()
print("Heartbeat from vehicle received.")

def ensure_mode(master, desired_mode):
    """Ensure the submarine is in the desired control mode."""
    # Ensure we have mode mappings available
    if master.mode_mapping() is None:
        print("Mode mapping is not available.")
        exit(1)

    if desired_mode in master.mode_mapping():
        current_mode = master.flightmode
        if current_mode != desired_mode:
            mode_id = master.mode_mapping()[desired_mode]
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
            print(f"Setting mode to {desired_mode}.")
            # Wait until mode change is confirmed
            while True:
                ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
                if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    print(f"Mode set to {desired_mode}")
                    break
    else:
        print(f"Desired mode {desired_mode} not available.")
        exit(1)
        
ensure_mode(master, "MANUAL")

# turn on the lights
print("Turning on lights...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
    9, 0, 0, 0, 0, 0, 0)
time.sleep(2)


