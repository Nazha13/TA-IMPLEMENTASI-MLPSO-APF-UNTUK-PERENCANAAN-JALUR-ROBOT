import pandas as pd
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import os
import re
from scipy.spatial.distance import cdist

def analyze_sim_vs_real():
    # Setup path
    sim_folder = './Sim_Trials'
    real_folder = './Real_Trials'
    sim_metrics_file = os.path.join(sim_folder, 'hasil_pengujian_metrik.csv')
    map_file_path = './src/asr_navigation/maps/grit_1_edited_2.pgm'  
    global_path_file = os.path.join(real_folder, 'real_trial_1_pso_path.csv') 
    
    RESOLUTION = 0.05    
    ORIGIN_X = -13.313419  
    ORIGIN_Y = -14.406021  

    print("Scan data simulasi & real...")

    # Load referensi path PSO
    df_global = None
    global_coords = None
    if os.path.exists(global_path_file):
        df_global = pd.read_csv(global_path_file)
        global_coords = df_global[['X', 'Y']].to_numpy()
        print("Global path loaded.")
    else:
        print(f"Warning: Global path ga ketemu di {global_path_file}")

    # Fungsi buat ngitung cross-track error
    def calculate_deviations(traj_coords, ref_coords):
        if ref_coords is None or len(traj_coords) == 0:
            return 0.0, 0.0
        distances = cdist(traj_coords, ref_coords)
        min_distances = np.min(distances, axis=1)
        return np.max(min_distances), np.mean(min_distances)

    # Fungsi buat narik data (bisa untuk sim & real)
    def extract_trials_data(folder, sim_times=None, is_sim=False):
        results = []
        amcl_files = glob.glob(os.path.join(folder, '*_amcl.csv'))
        
        # Fallback ke trajectories lama kalo bag belom diekstrak
        if not amcl_files and is_sim:
            amcl_files = glob.glob(os.path.join(folder, 'trajectories', '*.csv'))
            
        for f in amcl_files:
            df = pd.read_csv(f)
            if 'X' not in df.columns or 'Y' not in df.columns:
                continue
                
            coords = df[['X', 'Y']].to_numpy()
            length = np.sum(np.hypot(np.diff(coords[:, 0]), np.diff(coords[:, 1])))
            
            # Cari nomor trial
            match = re.search(r'trial_?(\d+)', os.path.basename(f))
            trial_num = int(match.group(1)) if match else 0
            
            # Hitung durasi
            time_sec = 0.0
            if 'Time_sec' in df.columns:
                time_sec = df['Time_sec'].iloc[-1] - df['Time_sec'].iloc[0]
            if is_sim and sim_times and trial_num in sim_times:
                time_sec = sim_times[trial_num] # Override pake data metrik
                
            max_dev, mean_dev = calculate_deviations(coords, global_coords)
            
            # Tarik cmd_vel & sinkronisasi waktu
            v_x, v_y, v_theta, t_array = np.array([]), np.array([]), np.array([]), np.array([])
            cmd_files = glob.glob(os.path.join(folder, f'*trial_{trial_num}_cmd_vel.csv')) + \
                        glob.glob(os.path.join(folder, f'*trial{trial_num}_cmd_vel.csv'))
            
            if cmd_files:
                df_cmd = pd.read_csv(cmd_files[0])
                if 'Standard' in df_cmd.columns or 'Standard.1' in df_cmd.columns:
                    df_cmd = pd.read_csv(cmd_files[0], header=1)
                    
                if 'Linear_X' in df_cmd.columns and 'Angular_Z' in df_cmd.columns:
                    raw_vx = df_cmd['Linear_X'].to_numpy()
                    raw_vtheta = df_cmd['Angular_Z'].to_numpy()
                    raw_t = df_cmd['Time_sec'].to_numpy()
                    raw_vy = df_cmd['Linear_Y'].to_numpy() if 'Linear_Y' in df_cmd.columns else np.zeros_like(raw_vx)
                    
                    # Sinkronisasi: cari index pas robot mulai ngegas
                    moving_idx = np.where((np.abs(raw_vx) > 0.001) | (np.abs(raw_vy) > 0.001) | (np.abs(raw_vtheta) > 0.001))[0]
                    
                    if len(moving_idx) > 0:
                        start_i = moving_idx[0]
                        v_x = raw_vx[start_i:]
                        v_y = raw_vy[start_i:]
                        v_theta = raw_vtheta[start_i:]
                        # Nolin waktu
                        t_array = raw_t[start_i:] - raw_t[start_i] 
                    else:
                        v_x, v_y, v_theta, t_array = raw_vx, raw_vy, raw_vtheta, raw_t
                    
            results.append({
                'file': f, 'x': coords[:, 0], 'y': coords[:, 1], 
                'length': length, 'time': time_sec,
                'max_dev': max_dev, 'mean_dev': mean_dev,
                'v_x': v_x, 'v_y': v_y, 'v_theta': v_theta, 't_array': t_array
            })
        return results

    # 1. Ambil log durasi simulasi
    sim_times = {}
    if os.path.exists(sim_metrics_file):
        df_sim_metrics = pd.read_csv(sim_metrics_file)
        for _, row in df_sim_metrics.iterrows():
            sim_times[row['Trial']] = row['Duration (s)']

    # 2. Jalanin ekstraksi
    sim_results = extract_trials_data(sim_folder, sim_times=sim_times, is_sim=True)
    real_results = extract_trials_data(real_folder, is_sim=False)

    if not sim_results or not real_results:
        print("Error: Dataset kosong/kurang lengkap")
        return

    # 3. Summary statistik
    def get_stats(data_list):
        return {
            'len_min': np.min([d['length'] for d in data_list]),
            'len_max': np.max([d['length'] for d in data_list]),
            'len_avg': np.mean([d['length'] for d in data_list]),
            'len_std': np.std([d['length'] for d in data_list], ddof=1),
            
            'time_min': np.min([d['time'] for d in data_list]),
            'time_max': np.max([d['time'] for d in data_list]),
            'time_avg': np.mean([d['time'] for d in data_list]),
            'time_std': np.std([d['time'] for d in data_list], ddof=1),
            
            'max_dev_avg': np.mean([d['max_dev'] for d in data_list]), 
            'max_dev_peak': np.max([d['max_dev'] for d in data_list]),
            'max_dev_std': np.std([d['max_dev'] for d in data_list], ddof=1),
             
            'mean_dev_avg': np.mean([d['mean_dev'] for d in data_list]),
            'mean_dev_std': np.std([d['mean_dev'] for d in data_list], ddof=1)
        }

    sim_stats = get_stats(sim_results)
    real_stats = get_stats(real_results)

    # Ngeprint hasil
    print("\n--- Analisis Data: Simulasi vs Real-World ---")
    metrics = [
        ("Waktu Eksekusi (detik)", "time_min", "time_max", "time_avg", "time_std"),
        ("Jarak Tempuh (meter)", "len_min", "len_max", "len_avg", "len_std")
    ]
    
    for title, key_min, key_max, key_avg, key_std in metrics:
        print(f"\n{title}")
        print(f"  Simulasi   -> Min: {sim_stats[key_min]:.2f} | Max: {sim_stats[key_max]:.2f} | Mean: {sim_stats[key_avg]:.2f} ± {sim_stats[key_std]:.2f}")
        print(f"  Real-World -> Min: {real_stats[key_min]:.2f} | Max: {real_stats[key_max]:.2f} | Mean: {real_stats[key_avg]:.2f} ± {real_stats[key_std]:.2f}")
    
    print("\nDeviasi terhadap Global Path (Cross-Track Error)")
    print("  [Simulasi]")
    print(f"  - Mean Deviasi Harian : {sim_stats['mean_dev_avg']:>6.3f} ± {sim_stats['mean_dev_std']:.3f} m")
    print(f"  - Mean Puncak Deviasi : {sim_stats['max_dev_avg']:>6.3f} ± {sim_stats['max_dev_std']:.3f} m")
    print(f"  - Ekstrem Absolut     : {sim_stats['max_dev_peak']:>6.3f} m")
    
    print("\n  [Real-World]")
    print(f"  - Mean Deviasi Harian : {real_stats['mean_dev_avg']:>6.3f} ± {real_stats['mean_dev_std']:.3f} m")
    print(f"  - Mean Puncak Deviasi : {real_stats['max_dev_avg']:>6.3f} ± {real_stats['max_dev_std']:.3f} m")
    print(f"  - Ekstrem Absolut     : {real_stats['max_dev_peak']:>6.3f} m")
    print("-" * 50 + "\n")

    # 4. Plot Trajektori (Spasial)
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    
    map_img = None
    if os.path.exists(map_file_path):
        map_img = np.flipud(plt.imread(map_file_path))
        h, w = map_img.shape
        extent_bounds = [ORIGIN_X, ORIGIN_X + w*RESOLUTION, ORIGIN_Y, ORIGIN_Y + h*RESOLUTION]

    # Cari trial representatif (mendekati rata-rata)
    def get_representative(data_list):
        if not data_list: return None
        avg_len = np.mean([d['length'] for d in data_list])
        return min(data_list, key=lambda d: abs(d['length'] - avg_len))

    datasets = [
        {'title': 'Simulasi', 'data': sim_results, 'rep': get_representative(sim_results), 'ax': axes[0]},
        {'title': 'Riil', 'data': real_results, 'rep': get_representative(real_results), 'ax': axes[1]}
    ]

    for ds in datasets:
        ax = ds['ax']
        
        if map_img is not None:
            ax.imshow(map_img, cmap='gray', origin='lower', extent=extent_bounds)
        else:
            ax.grid(True)
            
        # Bikin kotak obstacle (rotasi -45 deg)
        CENTER_X = 4.23
        CENTER_Y = 1.13
        BOX_SIZE = 0.7
        
        x_bottom_left = CENTER_X - (BOX_SIZE / 2)
        y_bottom_left = CENTER_Y - (BOX_SIZE / 2)
        
        rect = plt.Rectangle((x_bottom_left, y_bottom_left), BOX_SIZE, BOX_SIZE, color='black', alpha=0.8, zorder=2)
        t = transforms.Affine2D().rotate_deg_around(CENTER_X, CENTER_Y, -45) + ax.transData
        rect.set_transform(t)
        ax.add_patch(rect)

        if df_global is not None:
            gx, gy = global_coords[:, 0], global_coords[:, 1]
            ax.plot(gx, gy, color='lime', linewidth=2, linestyle='--', zorder=3, label='Global Path (MLPSO)')
            ax.scatter(gx[0], gy[0], color='black', marker='o', s=150, zorder=8, label='Start (Global)')
            ax.scatter(gx[-1], gy[-1], color='black', marker='*', s=300, edgecolors='black', linewidths=1.0, zorder=8, label='Goal (Global)')

        # Plot background trial lainnya
        for i, run in enumerate(ds['data']):
            lbl_traj = 'Riwayat Trial' if i == 0 else None
            lbl_end = 'Endpoint Trajektori' if i == 0 else None
            
            ax.plot(run['x'], run['y'], color='gray', alpha=0.2, linewidth=1.5, zorder=4, label=lbl_traj)
            ax.scatter(run['x'][-1], run['y'][-1], color='red', marker='x', s=60, zorder=7, label=lbl_end)

        # Plot trial utama
        if ds['rep']:
            ax.plot(ds['rep']['x'], ds['rep']['y'], color='blue', linewidth=2.5, zorder=6, label='Rata-rata Trajektori')

        ax.set_title(ds['title'], fontsize=14, fontweight='bold')
        ax.set_xlabel('X Coordinates (meters)', fontsize=12)
        ax.set_ylabel('Y Coordinates (meters)', fontsize=12)
        ax.legend(loc='lower right', framealpha=0.9, fontsize=10)
        ax.axis('equal')
        
        if map_img is not None:
            ax.set_xlim(-0.5, 8.0)  
            ax.set_ylim(-2.5, 6.0)  

    plt.tight_layout()
    plt.savefig('Sim2Real_Separated_Analysis.png', dpi=300, bbox_inches='tight')

    # 5. Visualisasi cmd_vel (Kiri: Simulasi | Kanan: Riil)
    fig2, axes2 = plt.subplots(3, 2, figsize=(14, 9)) 
    
    vel_metrics = [
        ('v_x', 'Linear Vel X (m/s)'),
        ('v_y', 'Linear Vel Y (m/s)'),
        ('v_theta', 'Angular Vel Z (rad/s)')
    ]

    for col_idx, ds in enumerate(datasets):
        # Set judul untuk tiap kolom di sini (row_idx 0 saja biar ga dobel)
        axes2[0, col_idx].set_title(ds['title'], fontsize=14, fontweight='bold', pad=15)
        
        for row_idx, (vel_key, y_label) in enumerate(vel_metrics):
            ax = axes2[row_idx, col_idx]
            
            has_data = any(len(run.get('t_array', [])) > 0 for run in ds['data'])
            if not has_data:
                ax.text(0.5, 0.5, 'Data ga tersedia', ha='center', va='center', transform=ax.transAxes, fontsize=12)
                ax.axis('off')
                continue

            for run in ds['data']:
                if run != ds['rep'] and len(run.get('t_array', [])) > 0:
                    if len(run.get(vel_key, [])) == len(run['t_array']):
                        ax.plot(run['t_array'], run[vel_key], color='gray', alpha=0.3, linewidth=1.2)
            
            if ds['rep'] and len(ds['rep'].get('t_array', [])) > 0:
                if len(ds['rep'].get(vel_key, [])) == len(ds['rep']['t_array']):
                    ax.plot(ds['rep']['t_array'], ds['rep'][vel_key], color='blue', linewidth=2.0, label='Representative Trial')
            
            ax.set_ylabel(y_label)
            ax.grid(True, linestyle='--', alpha=0.6)
            
            if row_idx == 2:
                ax.set_xlabel('Time (s)')
                
            if row_idx == 0 and col_idx == 1 and ds['rep']:
                 ax.legend(loc='upper right', framealpha=0.9)

    plt.tight_layout()
    plt.savefig('Sim2Real_Velocity_Analysis.png', dpi=300, bbox_inches='tight')
    
    plt.show()

if __name__ == "__main__":
    analyze_sim_vs_real()