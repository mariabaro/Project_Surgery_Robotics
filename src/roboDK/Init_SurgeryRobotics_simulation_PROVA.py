from robodk.robolink import *
from robodk.robomath import *
import time
import math
import tkinter as tk
import threading
import socket
import json
import os

# Define the relative and absolute path to the RoboDK project file
relative_path = "src/roboDK/SurgeryRobotics.rdk"
absolute_path = os.path.abspath(relative_path)
# Constants
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 1024
ROBOT_NAME = 'UR5e'
ZERO_YAW_TOOL = 0
ZERO_YAW_GRIPPER = 0
READ_INTERVAL_S = 0.01

Endowrist_rpy = None
Gripper_rpy = None
Servo_torques = None
data_lock = threading.Lock()# semaphor to manage data from 2 threads

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
#print(f"Listening on {UDP_IP}:{UDP_PORT}")

# Initialize RoboDK
def initialize_robodk(absolute_path):
    RDK = Robolink()
    time.sleep(2)  # wait for RoboDK to be ready
    RDK.AddFile(absolute_path)
    robot = RDK.Item(ROBOT_NAME)
    base = RDK.Item(f'{ROBOT_NAME} Base')
    endowrist = RDK.Item('Endowrist')
    gripper = RDK.Item('Gripper')
    needle = RDK.Item('Needle')
    Init_target = RDK.Item('Init')
    robot.setPoseFrame(base)
    robot.setPoseTool(endowrist)
    gripper_init = TxyzRxyz_2_Pose([0, 5, -105, 0, 0, 0])
    gripper.setParent(endowrist)
    gripper.setPose(gripper_init)
    needle_init = TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0])
    needle.setParent(gripper)
    needle.setPose(needle_init)
    robot.setSpeed(50)
    robot.MoveL(Init_target)
    return RDK, robot, base, gripper, needle

# Transformation Endowrist to base
def endowrist2base_orientation(roll, pitch, yaw):
    roll2 = (roll + 90) % 360
    pitch2 = pitch % 360
    yaw2 = yaw % 360
    return roll2, pitch2, yaw2

# Function to update the label with text
def update_text_label(label, tool_orientation, gripper_orientation, status_message, torque_values):
    torque_text = "\n".join([f"{k}: {v:.2f}" for k, v in torque_values.items()]) if torque_values else "No torque data"
    full_text = f"Tool orientation: {tool_orientation}\nGripper orientation: {gripper_orientation}\n{status_message}\nTorque Values: {torque_values}"
    label.after(0, lambda: label.config(text=full_text))

def update_torque_color(label, torque_values):
    if not torque_values:
        color = "gray"
    else:
        max_torque = max(torque_values.values())
        if max_torque < 2.0:
            color = "green"
        elif max_torque < 4.0:
            color = "yellow"
        else:
            color = "red"
    label.after(0, lambda: label.config(bg=color))

# Function to read UDP data and update the global variable
def read_data_UDP():
    global Endowrist_rpy, Gripper_rpy, data_lock
    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE) 
            try:
                received_data = json.loads(data.decode())
                device_id = received_data.get("device")
                if device_id == "G3_Endo":
                    with data_lock:
                        Endowrist_rpy = received_data
                elif device_id == "G3_Gri":
                    with data_lock:
                        Gripper_rpy = received_data
            except json.JSONDecodeError:
                print("Error decoding JSON data")
        except socket.error as e:
            #print(f"Socket error in UDP reader: {e}")
            sock.close()
            print("Socket closed.")
            break

def send_ur5_command(robot, pose_matrix):
    if not USE_REAL_UR5:
        return
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((UR5_IP, UR5_PORT))
        joints = robot.SolveIK(pose_matrix)
        ur_cmd = f"movej([{','.join(map(str, joints))}], a=1.2, v=0.25)\n"
        s.send(ur_cmd.encode())
        s.close()
    except Exception as e:
        print(f"UR5 socket error: {e}")

# Function to process the latest UDP data and move the robot
def move_robot(robot, gripper, needle, text_label, torque_indicator):
    global ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, Endowrist_rpy, Gripper_rpy, Servo_torques, data_lock

    while True:
        with data_lock:
            current_Endowrist_rpy = Endowrist_rpy
            current_Gripper_rpy = Gripper_rpy
            current_torques = Servo_torques

        endowrist_orientation_msg = ""
        gripper_orientation_msg = ""
        status_message = ""

        if current_Endowrist_rpy:
            e_roll = current_Endowrist_rpy.get("roll")
            e_pitch = current_Endowrist_rpy.get("pitch")
            e_yaw = current_Endowrist_rpy.get("yaw")
            s3 = current_Endowrist_rpy.get("s3")
            s4 = current_Endowrist_rpy.get("s4")

            endo_roll, endo_pitch, endo_yaw = endowrist2base_orientation(e_roll, e_pitch, e_yaw)
            endowrist_pose = robot.Pose()
            Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(endowrist_pose)
            endowrist_pose_new = transl(Xr, Yr, Zr) * rotz(math.radians(ZERO_YAW_TOOL)) * rotz(math.radians(endo_yaw)) * roty(math.radians(endo_pitch)) * rotx(math.radians(endo_roll))

            if robot.MoveL_Test(robot.Joints(), endowrist_pose_new) == 0:
                if USE_REAL_UR5:
                    send_ur5_command(robot, endowrist_pose_new)
                else:
                    robot.MoveL(endowrist_pose_new, True)
                endowrist_orientation_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                status_message = ""
            else:
                endowrist_orientation_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                status_message = "Robot cannot reach the position"

            # Relative Z movement via buttons S3/S4
            if s3 == 0 or s4 == 0:
                current_pose = robot.Pose()
                Tz = transl(0, 0, 5) if s3 == 0 else transl(0, 0, -5)
                new_pose = current_pose * Tz
                status_message = "⬆ S3 pressed: moving up" if s3 == 0 else "⬇ S4 pressed: moving down"
                if robot.MoveL_Test(robot.Joints(), new_pose) == 0:
                    if USE_REAL_UR5:
                        send_ur5_command(robot, new_pose)
                    else:
                        robot.MoveL(new_pose, True)
                else:
                    status_message = "❌ Cannot move further in Z"

        if current_Gripper_rpy:
            g_roll = current_Gripper_rpy.get("roll")
            g_pitch = current_Gripper_rpy.get("pitch")
            g_yaw = current_Gripper_rpy.get("yaw")
            s1 = current_Gripper_rpy.get("s1")
            s2 = current_Gripper_rpy.get("s2")

            gripper_pose = gripper.Pose()
            Xg, Yg, Zg, rg, pg, yg = Pose_2_TxyzRxyz(gripper_pose)
            gripper_pose_new = transl(Xg, Yg, Zg) * rotz(math.radians(ZERO_YAW_GRIPPER)) * rotz(math.radians(g_yaw)) * roty(math.radians(g_pitch)) * rotx(math.radians(g_roll))
            gripper.setPose(gripper_pose_new)
            gripper_orientation_msg = f"R={round(g_roll)} P={round(g_pitch)} W={round((g_yaw+ZERO_YAW_GRIPPER)%360)}"

            # Needle control
            if s1 == 0:
                needle.setParentStatic(robot.Item('UR5e Base'))
                status_message = "🟢 S1 pressed: needle released"
            elif s1 == 1:
                needle.setParent(gripper)
                needle.setPose(TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0]))
                status_message = "🔵 S1 released: needle grasped"

        # Update GUI
        update_text_label(text_label, endowrist_orientation_msg, gripper_orientation_msg, status_message, current_torques)
        update_torque_color(torque_indicator, current_torques)

        time.sleep(READ_INTERVAL_S)

def on_closing():
    global root, sock
    print("Closing...")
    try:
        sock.close()
        print("Ending Socket")
        #initialize_robodk()
        #print("Program INITIALIZED")
    except Exception as e:
        #print(f"Error al tancar el socket: {e}")
        pass
    root.destroy()

# Update functions for sliders
def set_zero_yaw_tool(value):
    global ZERO_YAW_TOOL
    ZERO_YAW_TOOL = float(value)

def set_zero_yaw_gripper(value):
    global ZERO_YAW_GRIPPER
    ZERO_YAW_GRIPPER = float(value)

# Main function
def main():
    global root, ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, robot, gripper, base, text_label, absolute_path
    
    RDK, robot, base, gripper, needle = initialize_robodk(absolute_path)

    root = tk.Tk()
    root.title("Suture Process")
    root.protocol("WM_DELETE_WINDOW", on_closing) # Proper clossing
    text_label = tk.Label(root, text="", wraplength=300)
    text_label.pack(padx=20, pady=20)
    
    torque_indicator = tk.Label(root, text="Torque Level", width=20, height=2, bg="gray")
    torque_indicator.pack(pady=10)

    # Add sliders for ZERO_YAW_TOOL and ZERO_YAW_GRIPPER
    tool_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Tool Yaw",
                                    command=lambda value: set_zero_yaw_tool(float(value)), length=200)
    tool_yaw_slider.set(ZERO_YAW_TOOL)
    tool_yaw_slider.pack()

    gripper_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Gripper Yaw",
                                        command=lambda value: set_zero_yaw_gripper(float(value)), length=200)
    gripper_yaw_slider.set(ZERO_YAW_GRIPPER)
    gripper_yaw_slider.pack()

    # Start the UDP reading thread
    udp_thread = threading.Thread(target=read_data_UDP)
    udp_thread.daemon = True
    udp_thread.start()

    # Start the robot movement thread
    robot_thread = threading.Thread(target=move_robot, args=(robot, gripper, needle, text_label))
    robot_thread.daemon = True
    robot_thread.start()

    root.mainloop()
    print("Pop-up menu closed")
    RDK.CloseRoboDK()
    print("RoboDK closed")

if __name__ == "__main__":
    main()