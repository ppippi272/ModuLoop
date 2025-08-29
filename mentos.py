# 성공
# Step 1: Initialize Robot and Gripper
def initialize_robot_and_gripper():
    print("▶ Executing: initialize_robot_and_gripper()")
    import rtde_control
    import rtde_receive
    import RTDE

    # Connect to the robot control interface
    print("Connecting to robot control interface at IP 192.168.1.101")
    rtde_c = rtde_control.RTDEControlInterface("192.168.1.101")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.101")

    # Connect to the gripper
    print("Connecting to gripper at IP 192.168.1.101, port 63352")
    gripper = RTDE.RobotiqCModelURCap()
    gripper.connect('192.168.1.101', 63352)

    # Set the TCP offset for the gripper
    print("Setting TCP offset for the gripper")
    rtde_c.setTcp([0.0, 0.0, 0.175, 0.0, 0.0, 0.0])

    return rtde_c, rtde_r, gripper

# Initialize the robot and gripper
rtde_c, rtde_r, gripper = initialize_robot_and_gripper()

# Generalized Pick and Place Function
def pick_and_place(rtde_c, rtde_r, gripper, mentos_position, mentos_orientation, plate_position):
    print("▶ Executing: pick_and_place()")
    
    approach_height = 0.1

    # Calculate the approach position above the Mentos
    print("Calculating approach position above Mentos")
    approach_position = mentos_position.copy()
    approach_position[2] += approach_height

    # Move to the approach position
    print(f"Approaching Mentos at position: {approach_position}")
    joint_positions = rtde_c.getInverseKinematics(approach_position + [-0.0624, -3.1108, 0.2682])
    joint_positions[5] = mentos_orientation
    rtde_c.moveJ(joint_positions)

    # Move down to grasp the Mentos
    print("Moving down to grasp Mentos")
    current_tcp = rtde_r.getActualTCPPose()
    current_tcp[2] -= approach_height
    rtde_c.moveL(current_tcp)

    # Close the gripper to grasp the Mentos
    print("Closing gripper to grasp Mentos")
    gripper.move_and_wait_for_pos(255, 255, 255)

    # Lift the Mentos
    print("Lifting Mentos")
    current_tcp[2] += approach_height
    rtde_c.moveL(current_tcp)

    # Calculate the placement position on the plate
    print("Calculating placement position on plate")
    placement_position = plate_position.copy()
    placement_position[2] += 0.1

    # Move to the placement position above the plate
    print(f"Moving to placement position above plate: {placement_position}")
    joint_positions = rtde_c.getInverseKinematics(placement_position + [-0.0624, -3.1108, 0.2682])
    rtde_c.moveJ(joint_positions)

    # Lower to place the Mentos on the plate
    print("Lowering to place Mentos on plate")
    current_tcp = rtde_r.getActualTCPPose()
    current_tcp[2] -= 0.1
    rtde_c.moveL(current_tcp)

    # Open the gripper to release the Mentos
    print("Opening gripper to release Mentos")
    gripper.move_and_wait_for_pos(0, 255, 255)

    # Lift the gripper after placing the Mentos
    print("Lifting gripper after placing Mentos")
    current_tcp[2] += 0.1
    rtde_c.moveL(current_tcp)

# Define the positions and orientations for the Mentos
middle_mentos_position = [0.04947953982933424, -0.47348421159481374, 0.035169425167874305]
middle_mentos_orientation = -1.5345
right_mentos_position = [0.1, -0.5, 0.035]
right_mentos_orientation = -1.5345
top_mentos_position = [0.2, -0.4, 0.035]
top_mentos_orientation = -1.5345

# Define the plate position
plate_position = [-0.14505833509812266, -0.39026389609595497, 0.038436734619264602]

# Execute the function for each Mentos
pick_and_place(rtde_c, rtde_r, gripper, middle_mentos_position, middle_mentos_orientation, plate_position)
pick_and_place(rtde_c, rtde_r, gripper, right_mentos_position, right_mentos_orientation, plate_position)
pick_and_place(rtde_c, rtde_r, gripper, top_mentos_position, top_mentos_orientation, plate_position)