import rosbag
import csv
import glob
import os

def extract_all_topics():
    folder_path = 'Real_Trials'
    bag_files = glob.glob(os.path.join(folder_path, '*.bag'))
    
    if not bag_files:
        print(f"Tidak ada file .bag yang ditemukan di '{folder_path}'.")
        return

    print(f"Ditemukan {len(bag_files)} file bag. Memulai ekstraksi...\n")

    for bag_file in bag_files:
        base_name = os.path.splitext(os.path.basename(bag_file))[0]
        print(f"Memproses: {bag_file}")
        
        amcl_buffer = []
        vel_buffer = []
        path_buffer = []

        start_time = None
        first_moving_time = None 
        last_moving_time = 0.0   
        total_record_time = 0.0

        # Ekstrak data dan catat waktu pergerakan robot
        bag = rosbag.Bag(bag_file)
        for topic, msg, t in bag.read_messages(topics=['/amcl_pose', '/cmd_vel', '/pso_path']):
            
            if start_time is None:
                start_time = t.to_sec()
                
            relative_time = t.to_sec() - start_time
            total_record_time = relative_time 
            
            if topic == '/amcl_pose':
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                amcl_buffer.append([relative_time, x, y])
                
            elif topic == '/cmd_vel':
                vx = msg.linear.x
                wz = msg.angular.z
                vel_buffer.append([relative_time, vx, wz])
                
                # Deteksi pergerakan untuk memotong waktu diam
                if abs(vx) > 0.001 or abs(wz) > 0.001:
                    if first_moving_time is None:
                        first_moving_time = relative_time
                    last_moving_time = relative_time 
                
            elif topic == '/pso_path':
                for i, pose_stamped in enumerate(msg.poses):
                    px = pose_stamped.pose.position.x
                    py = pose_stamped.pose.position.y
                    path_buffer.append([relative_time, i, px, py])

        bag.close()

        # Tentukan rentang waktu pemotongan (cutoff)
        if first_moving_time is None:
            first_moving_time = 0.0 
            
        # Berikan buffer 1 detik agar transisi pergerakan tidak terpotong
        start_cutoff = max(0.0, first_moving_time - 1.0) 
        end_cutoff = last_moving_time + 1.0 
        trimmed_duration = end_cutoff - start_cutoff

        print(f"  Durasi total : {total_record_time:.2f} detik")
        print(f"  Waktu gerak  : Mulai {first_moving_time:.2f}s | Berhenti {last_moving_time:.2f}s")
        print(f"  Waktu bersih : {trimmed_duration:.2f} detik")

        # Simpan ke CSV berdasarkan rentang waktu yang sudah disesuaikan
        
        # 1. Tulis AMCL
        amcl_csv = os.path.join(folder_path, f"real_{base_name}_amcl.csv")
        with open(amcl_csv, 'w', newline='') as f_amcl:
            w_amcl = csv.writer(f_amcl)
            w_amcl.writerow(['Time_sec', 'X', 'Y'])  
            c_amcl = 0
            for row in amcl_buffer:
                if start_cutoff <= row[0] <= end_cutoff:
                    w_amcl.writerow(row)
                    c_amcl += 1

        # 2. Tulis CMD_VEL
        vel_csv = os.path.join(folder_path, f"real_{base_name}_cmd_vel.csv")
        with open(vel_csv, 'w', newline='') as f_vel:
            w_vel = csv.writer(f_vel)
            w_vel.writerow(['Time_sec', 'Linear_X', 'Angular_Z'])
            c_vel = 0
            for row in vel_buffer:
                if start_cutoff <= row[0] <= end_cutoff:
                    w_vel.writerow(row)
                    c_vel += 1

        # 3. Tulis PSO_PATH
        path_csv = os.path.join(folder_path, f"real_{base_name}_pso_path.csv")
        with open(path_csv, 'w', newline='') as f_path:
            w_path = csv.writer(f_path)
            w_path.writerow(['Time_sec', 'Waypoint_Index', 'X', 'Y'])
            c_path = 0
            for row in path_buffer:
                if start_cutoff <= row[0] <= end_cutoff:
                    w_path.writerow(row)
                    c_path += 1
        
        print(f"  Tersimpan    : {c_amcl} data AMCL, {c_vel} data CMD_VEL, {c_path} data Path\n")

    print("Selesai mengekstrak semua file bag.")

if __name__ == "__main__":
    extract_all_topics()