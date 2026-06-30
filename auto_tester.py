#!/usr/bin/env python3
import rospy
import actionlib
import csv
import time
import math
import os
import psutil
import threading
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty

# Konfigurasi pengujian
NUM_TRIALS = 20
CSV_FILENAME = "hasil_pengujian_metrik.csv"

# Data start dan goal pose
ROBOT_NAME = "CAD_ASR_WITH_OMNI"
START_X = 1.566994468493967
START_Y = -1.2907059555081593
START_Z = 0.05
START_ROLL = 1.5708  
START_PITCH = 0.0
START_YAW = 0.0 

GOAL_X = 8.5
GOAL_Y = 5.5
GOAL_W = 1.0 

# Konfigurasi robot
ROBOT_RADIUS = 0.3 # Radius berdasarkan footprint robot yang telah ditentukan

# Variabel Global untuk Tracking
current_trajectory = []
min_clearance = float('inf')
is_moving = False
replan_count = 0  # Counter untuk deteksi replanning

# Variabel Global untuk Resource Monitoring
cpu_records = []
ram_records = []

def resource_monitor():
    """Berjalan di background thread untuk mencatat beban CPU dan RAM KHUSUS node move_base"""
    global cpu_records, ram_records, is_moving
    
    # Mencari proses move_base di OS
    move_base_proc = None
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if 'move_base' in proc.info['name'] or (proc.info['cmdline'] and 'move_base' in ' '.join(proc.info['cmdline'])):
                move_base_proc = psutil.Process(proc.info['pid'])
                break
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
            
    if not move_base_proc:
        rospy.logwarn("Proses 'move_base' tidak ditemukan! Pastikan node move_base berjalan.")
        
    while is_moving:
        if move_base_proc:
            try:
                # CPU usage khusus node ini
                cpu = move_base_proc.cpu_percent(interval=0.2)
                # RAM usage khusus node ini (RSS = Resident Set Size dalam MB)
                ram = move_base_proc.memory_info().rss / (1024 * 1024)
                
                cpu_records.append(cpu)
                ram_records.append(ram)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        else:
            time.sleep(0.2)

def odom_callback(msg):
    global current_trajectory, is_moving
    if not is_moving:
        return
        
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    current_trajectory.append((x, y))

def scan_callback(msg):
    global min_clearance, is_moving
    if not is_moving:
        return

    # Filter: Abaikan semua pantulan yang bernilai di bawah atau sama dengan radius robot
    valid_ranges = [r for r in msg.ranges if r > ROBOT_RADIUS and r < msg.range_max]
    
    if valid_ranges:
        closest_point = min(valid_ranges)
        
        # Jarak bersih dari batas terluar robot ke rintangan
        dist_to_edge = closest_point - ROBOT_RADIUS
        
        if dist_to_edge < min_clearance:
            min_clearance = dist_to_edge

def plan_callback(msg):
    """Mendeteksi setiap kali planner membuat atau memperbarui jalur (replanning)"""
    global replan_count, is_moving
    if is_moving:
        replan_count += 1

def reset_robot_position():
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state_msg = ModelState()
        state_msg.model_name = ROBOT_NAME
        state_msg.pose.position.x = START_X
        state_msg.pose.position.y = START_Y
        state_msg.pose.position.z = START_Z + 0.05
        
        q = quaternion_from_euler(START_ROLL, START_PITCH, START_YAW)
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]

        state_msg.twist.linear.x = 0.0
        state_msg.twist.linear.y = 0.0
        state_msg.twist.linear.z = 0.0
        state_msg.twist.angular.x = 0.0
        state_msg.twist.angular.y = 0.0
        state_msg.twist.angular.z = 0.0
        
        set_state(state_msg)
        rospy.loginfo("Robot di-reset ke titik start.")
        rospy.sleep(2.0) 
    except rospy.ServiceException as e:
        rospy.logerr(f"Gagal me-reset robot: {e}")

def clear_costmaps():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_srv()
        rospy.loginfo("Costmap di-bersihkan.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service clear costmaps gagal: {e}")

def send_goal_and_track(client, x, y, w, trial_name):
    global current_trajectory, min_clearance, is_moving, cpu_records, ram_records, replan_count
    
    # Reset tracking
    current_trajectory = []
    min_clearance = float('inf')
    cpu_records = []
    ram_records = []
    replan_count = 0  # Reset replan counter ke 0 setiap trial baru
    is_moving = True
    
    monitor_thread = threading.Thread(target=resource_monitor)
    monitor_thread.start()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w

    rospy.loginfo("Mengirim goal ke move_base...")
    start_time = rospy.Time.now().to_sec()
    client.send_goal(goal)
    
    # Batasi waktu tunggu maksimal 90.0 detik
    finished_within_time = client.wait_for_result(rospy.Duration(90.0)) 
    end_time = rospy.Time.now().to_sec()
    
    is_moving = False 
    monitor_thread.join()
    
    duration = end_time - start_time
    
    # Kalkulasi Trajektori & Jarak Tempuh
    path_length = 0.0
    for i in range(1, len(current_trajectory)):
        dx = current_trajectory[i][0] - current_trajectory[i-1][0]
        dy = current_trajectory[i][1] - current_trajectory[i-1][1]
        path_length += math.hypot(dx, dy)
        
    avg_cpu = sum(cpu_records) / len(cpu_records) if cpu_records else 0.0
    peak_ram = max(ram_records) if ram_records else 0.0
        
    # Evaluasi Status Berdasarkan Waktu dan State Move Base
    if not finished_within_time:
        rospy.logwarn(f"[{trial_name}] Waktu habis (90 detik)! Goal dibatalkan.")
        client.cancel_goal()  # Paksa robot berhenti bergerak di Gazebo
        status_str = "Failed_Timeout"
    else:
        state = client.get_state()
        success = (state == actionlib.GoalStatus.SUCCEEDED)
        if success:
            # Plan pertama adalah rute awal. Jika > 1, berarti robot melakukan replanning.
            status_str = "Success_Replanned" if replan_count > 1 else "Success_Direct"
        else:
            status_str = "Failed_Stuck_Or_Crash"

    # Simpan titik trajektori ke CSV terpisah
    os.makedirs("trajectories", exist_ok=True)
    traj_filename = f"trajectories/{trial_name}.csv"
    with open(traj_filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["X", "Y"])
        writer.writerows(current_trajectory)

    return status_str, duration, min_clearance, path_length, avg_cpu, peak_ram

if __name__ == '__main__':
    rospy.init_node('auto_tester_node')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/scan', LaserScan, scan_callback) # Ganti /scan jika nama topik LiDAR Anda berbeda
    rospy.Subscriber('/pso_path', Path, plan_callback)  # Memantau path spesifik untuk deteksi replanning
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    scenario_input = input("Pilih Skenario rintangan (1 atau 2): ")
    planner_name = input("Masukkan nama local planner (contoh: DWA, APF, TEB): ")
    scenario_name = f"Scen{scenario_input}_{planner_name.upper()}"
    
    rospy.loginfo("Menunggu action server move_base aktif...")
    client.wait_for_server()
    
    file_exists = os.path.isfile(CSV_FILENAME)
    with open(CSV_FILENAME, mode='a', newline='') as file:
        writer = csv.writer(file)
        
        if not file_exists:
            writer.writerow(['Scenario', 'Trial', 'Status', 'Duration (s)', 'Min Clearance (m)', 'Path Length (m)', 'Avg CPU (%)', 'Peak RAM (MB)'])
        
        for trial in range(1, NUM_TRIALS + 1):
            rospy.loginfo(f"\n--- {scenario_name} | Trial {trial}/{NUM_TRIALS} ---")
            trial_name = f"{scenario_name}_trial{trial}"
            
            reset_robot_position()
            clear_costmaps()
            
            status, duration, clearance, path_length, avg_cpu, peak_ram = send_goal_and_track(client, GOAL_X, GOAL_Y, GOAL_W, trial_name)
            
            clearance_val = f"{clearance:.3f}" if clearance != float('inf') else "N/A"
            writer.writerow([scenario_name, trial, status, f"{duration:.2f}", clearance_val, f"{path_length:.2f}", f"{avg_cpu:.1f}", f"{peak_ram:.1f}"])
            
            rospy.loginfo(f"Hasil Akhir: {status} | Waktu: {duration:.2f}s | Jarak: {path_length:.2f}m | CPU: {avg_cpu:.1f}% | RAM: {peak_ram:.1f} MB | Replan: {replan_count-1}x")
            
    rospy.loginfo(f"Pengambilan data untuk {scenario_name} selesai!")