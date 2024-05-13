from pymavlink import mavutil
import time

def setup_connection(ip='192.168.2.1', port=14555):
    """Create and return a MAVLink connection using UDP."""
    return mavutil.mavlink_connection(f'udpin:{ip}:{port}')

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

def move(master, forward=0, down=0, turnleft=0, button=0):
    """Send a manual control command to move the vehicle forward."""
    print("Sending command to move forward...")
    master.mav.manual_control_send(
        master.target_system,
        -forward,  # Positive x value to move forward
        0,    # Y value for lateral movement (none)
        500+down,    # Z value for vertical movement (none)
        turnleft,    # R value for rotation (none)
        button     # No buttons pressed
    )

def main():
    """Main function to execute the vehicle control."""
    master = setup_connection()
    
    # Wait for the first heartbeat to ensure the connection is established
    print("Waiting for heartbeat from vehicle...")
    master.wait_heartbeat()
    print("Heartbeat from vehicle received.")

    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print('Disarmed!')

    # Ensure the vehicle is in a manual control mode
    ensure_mode(master, "MANUAL")
    print("Waiting for the vehicle to arm")

    while not master.motors_armed:
        master.arducopter_arm()
        master.motors_armed_wait()
    print('Armed!')
    
    # Command to move forward continuously
    try:
        while True:
            move(master)
            time.sleep(1)  # Command refresh rate
    except KeyboardInterrupt:
        print("Stopped by user.")

    # Disarm the vehicle
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print('Disarmed!')

if __name__ == "__main__":
    main()
