import rosbag
import csv
import glob
import os

def extract_all_topics():
    folder_path = 'Sim_Trials'
    bag_files = glob.glob(os.path.join(folder_path, '*.bag'))
    
    if not bag_files:
        print(f"Tidak ada file .bag ditemukan pada direktori '{folder_path}'.")
        return

    print(f"Ditemukan {len(bag_files)} file bag. Memulai proses ekstraksi...\n")

    for bag_file in bag_files:
        base_name = os.path.splitext(os.path.basename(bag_file))[0]
        print(f"Memproses file: {bag_file}")
        
        amcl_buffer, vel_buffer, path_buffer = [], [], []
        start_time = None
        first_moving_time = None 
        last_moving_time = 0.0   
        total_time = 0.0

        # Membaca isi bag dan menyimpannya ke buffer
        bag = rosbag.Bag(bag_file)
        for topic, msg, t in bag.read_messages(topics=['/amcl_pose', '/cmd_vel', '/pso_path']):
            
            if start_time is None:
                start_time = t.to_sec()
                
            rel_t = t.to_sec() - start_time
            total_time = rel_t 
            
            if topic == '/amcl_pose':
                amcl_buffer.append([rel_t, msg.pose.pose.position.x, msg.pose.pose.position.y])
                
            elif topic == '/cmd_vel':
                vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
                vel_buffer.append([rel_t, vx, vy, wz])
                
                # Deteksi fase pergerakan untuk menentukan waktu potong data
                if abs(vx) > 0.001 or abs(vy) > 0.001 or abs(wz) > 0.001:
                    if first_moving_time is None:
                        first_moving_time = rel_t
                    last_moving_time = rel_t
                
            elif topic == '/pso_path':
                for i, pose in enumerate(msg.poses):
                    path_buffer.append([rel_t, i, pose.pose.position.x, pose.pose.position.y])

        bag.close()

        # Menentukan rentang waktu untuk pemotongan data (dengan buffer 1 detik)
        start_cut = max(0.0, (first_moving_time or 0.0) - 1.0) 
        end_cut = last_moving_time + 1.0 
        
        print(f"  Durasi total : {total_time:.2f}s | Gerak: {first_moving_time:.2f}s - {last_moving_time:.2f}s")

        # Fungsi pembantu untuk menyimpan data ke CSV
        def save_to_csv(data, filename, header):
            path = os.path.join(folder_path, filename)
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                count = 0
                for row in data:
                    if start_cut <= row[0] <= end_cut:
                        writer.writerow(row)
                        count += 1
            return count

        # Menyimpan hasil ke file CSV
        c1 = save_to_csv(amcl_buffer, f"real_{base_name}_amcl.csv", ['Time_sec', 'X', 'Y'])
        c2 = save_to_csv(vel_buffer, f"real_{base_name}_cmd_vel.csv", ['Time_sec', 'Linear_X', 'Linear_Y', 'Angular_Z'])
        c3 = save_to_csv(path_buffer, f"real_{base_name}_pso_path.csv", ['Time_sec', 'Waypoint_Index', 'X', 'Y'])
        
        print(f"  Berhasil tersimpan: {c1} data AMCL, {c2} data CMD_VEL, {c3} data Path\n")

    print("Seluruh proses ekstraksi telah selesai.")

if __name__ == "__main__":
    extract_all_topics()