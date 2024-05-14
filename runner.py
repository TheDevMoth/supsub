from pymavlink import mavutil
import time

def main():
    """Main function to execute the vehicle control."""
    link = setup_connection(ip="192.168.2.1", port=14555)
    
    disarm(link)

    # Ensure the vehicle is in a manual control mode
    ensure_mode(link, "MANUAL")
    # ensure_mode(master, "STABILIZE")

    arm(link)
    
    # Command to move forward continuously
    try:
        while True:
            move(link, forward=500, down=0, turnleft=0)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped by user.")

    # Disarm the vehicle
    disarm(link)

def setup_connection(ip='0.0.0.0', port=14550):
    """Create and return a MAVLink connection using UDP."""
    link = mavutil.mavlink_connection(f'udpin:{ip}:{port}')

    # Wait for the first heartbeat to ensure the connection is established
    print("Waiting for heartbeat from vehicle...")
    link.wait_heartbeat()
    print("Heartbeat from vehicle received.")

    return link

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
            print(f"Setting mode to {desired_mode}.")
            # Wait until mode change is confirmed
            while True:
                master.mav.set_mode_send(
                    master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
                ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
                if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    print(f"Mode set to {desired_mode}")
                    break
                else:
                    print(f"Failed to set mode to {desired_mode}. Retrying...")
                    print(f"Received message: {ack_msg}")
    else:
        print(f"Desired mode {desired_mode} not available.")
        exit(1)

def arm(link):
    while not link.motors_armed:
        print("sending arm command")
        link.arducopter_arm()
        time.sleep(1)
    print("Vehicle armed.")
    
    # master.arducopter_arm()
    # print("Waiting for the vehicle to arm")
    # master.motors_armed_wait()
    # print('Armed!')

def disarm(link):
    link.arducopter_disarm()
    link.motors_disarmed_wait()
    print('Disarmed!')

def move(master, forward=0, down=0, turnleft=0, button=0):
    """Send a manual control command to move the vehicle forward."""
    print("Sending command to move...")
    master.mav.manual_control_send(
        master.target_system,
        -forward,  # Positive x value to move forward
        0,    # Y value for lateral movement (none)
        500+down,    # Z value for vertical movement (none)
        turnleft,    # R value for rotation (none)
        button     # No buttons pressed
    )

if __name__ == "__main__":
    main()
