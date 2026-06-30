import pandas as pd
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import os
import re
from scipy.spatial.distance import cdist

def analyze_sim_vs_real():
    # Setup path file dan parameter lingkungan
    sim_traj_folder = './Sim_Trials/trajectories'
    sim_metrics_file = './Sim_Trials/hasil_pengujian_metrik.csv'
    real_folder = 'Real_Trials'
    map_file_path = './src/asr_navigation/maps/grit_1_edited_2.pgm'  
    global_path_file = './Real_Trials/real_trial_1_pso_path.csv' 
    
    RESOLUTION = 0.05    
    ORIGIN_X = -13.313419  
    ORIGIN_Y = -14.406021  

    print("Memindai data simulasi dan real-world...")

    # Load data global path (PSO) sebagai rujukan
    df_global = None
    global_coords = None
    if os.path.exists(global_path_file):
        df_global = pd.read_csv(global_path_file)
        global_coords = df_global[['X', 'Y']].to_numpy()
        print("Global path (PSO) berhasil dimuat.")
    else:
        print(f"Peringatan: Global path tidak ditemukan di: {global_path_file}")

    # Helper: Hitung deviasi (cross-track error) trajektori terhadap global path
    def calculate_deviations(traj_coords, ref_coords):
        if ref_coords is None or len(traj_coords) == 0:
            return 0.0, 0.0
        distances = cdist(traj_coords, ref_coords)
        min_distances = np.min(distances, axis=1)
        return np.max(min_distances), np.mean(min_distances)

    # Ekstrak data simulasi
    sim_times = {}
    if os.path.exists(sim_metrics_file):
        df_sim_metrics = pd.read_csv(sim_metrics_file)
        for _, row in df_sim_metrics.iterrows():
            sim_times[row['Trial']] = row['Duration (s)']

    sim_files = glob.glob(os.path.join(sim_traj_folder, '*.csv'))
    sim_results = []
    
    for f in sim_files:
        df = pd.read_csv(f)
        if 'X' in df.columns and 'Y' in df.columns:
            coords = df[['X', 'Y']].to_numpy()
            length = np.sum(np.hypot(np.diff(coords[:, 0]), np.diff(coords[:, 1])))
            
            # Cocokkan nomor trial untuk mendapatkan waktu eksekusi
            match = re.search(r'trial(\d+)\.csv', os.path.basename(f))
            time_sec = 0.0
            if match:
                trial_num = int(match.group(1))
                time_sec = sim_times.get(trial_num, 0.0)
            
            max_dev, mean_dev = calculate_deviations(coords, global_coords)
            
            sim_results.append({
                'file': f, 'x': coords[:, 0], 'y': coords[:, 1], 
                'length': length, 'time': time_sec,
                'max_dev': max_dev, 'mean_dev': mean_dev
            })
    
    if not sim_results:
        print("Error: Data trajektori simulasi tidak ditemukan!")
        return

    # Ekstrak data hardware / real-world
    amcl_files = glob.glob(os.path.join(real_folder, '*_amcl.csv'))
    real_results = []
    
    for f in amcl_files:
        df = pd.read_csv(f)
        if 'X' in df.columns and 'Y' in df.columns and 'Time_sec' in df.columns:
            coords = df[['X', 'Y']].to_numpy()
            length = np.sum(np.hypot(np.diff(coords[:, 0]), np.diff(coords[:, 1])))
            
            # Hitung delta waktu dari file log AMCL
            time_sec = df['Time_sec'].iloc[-1] - df['Time_sec'].iloc[0] 
            max_dev, mean_dev = calculate_deviations(coords, global_coords)
            
            real_results.append({
                'file': f, 'x': coords[:, 0], 'y': coords[:, 1], 
                'length': length, 'time': time_sec,
                'max_dev': max_dev, 'mean_dev': mean_dev
            })
            
    if not real_results:
        print("Error: Data _amcl.csv tidak ditemukan di folder real-world!")
        return

    # Helper: Agregasi statistik numerik
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

    # Cetak laporan komparatif ke terminal
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

    # Visualisasi plot spasial
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    
    map_img = None
    if os.path.exists(map_file_path):
        map_img = np.flipud(plt.imread(map_file_path))
        h, w = map_img.shape
        extent_bounds = [ORIGIN_X, ORIGIN_X + w*RESOLUTION, ORIGIN_Y, ORIGIN_Y + h*RESOLUTION]

    # Helper: cari lintasan paling mendekati nilai rata-rata jarak
    def get_representative(data_list):
        if not data_list: return None
        avg_len = np.mean([d['length'] for d in data_list])
        return min(data_list, key=lambda d: abs(d['length'] - avg_len))

    datasets = [
        {'title': 'Trajektori Lingkungan Simulasi', 'data': sim_results, 'rep': get_representative(sim_results), 'ax': axes[0]},
        {'title': 'Trajektori Lingkungan Riil', 'data': real_results, 'rep': get_representative(real_results), 'ax': axes[1]}
    ]

    for ds in datasets:
        ax = ds['ax']
        
        if map_img is not None:
            ax.imshow(map_img, cmap='gray', origin='lower', extent=extent_bounds)
        else:
            ax.grid(True)
            
        # Plot rintangan dengan orientasinya
        CENTER_X = 4.23
        CENTER_Y = 1.13
        BOX_SIZE = 0.7
        
        x_bottom_left = CENTER_X - (BOX_SIZE / 2)
        y_bottom_left = CENTER_Y - (BOX_SIZE / 2)
        
        rect = plt.Rectangle((x_bottom_left, y_bottom_left), BOX_SIZE, BOX_SIZE, color='black', alpha=0.8, zorder=2)
        
        t = transforms.Affine2D().rotate_deg_around(CENTER_X, CENTER_Y, -45) + ax.transData
        rect.set_transform(t)
        ax.add_patch(rect)

        # Plot global path
        if df_global is not None:
            gx, gy = global_coords[:, 0], global_coords[:, 1]
            ax.plot(gx, gy, color='lime', linewidth=2, linestyle='--', zorder=3, label='Global Path (MLPSO)')
            ax.scatter(gx[0], gy[0], color='black', marker='o', s=150, zorder=8, label='Start (Global)')
            ax.scatter(gx[-1], gy[-1], color='black', marker='*', s=300, edgecolors='black', linewidths=1.0, zorder=8, label='Goal (Global)')

        # Plot seluruh trial sebagai bayangan abu-abu
        for i, run in enumerate(ds['data']):
            lbl_traj = 'Riwayat Trial' if i == 0 else None
            lbl_end = 'Endpoint Trajektori' if i == 0 else None
            
            ax.plot(run['x'], run['y'], color='gray', alpha=0.2, linewidth=1.5, zorder=4, label=lbl_traj)
            ax.scatter(run['x'][-1], run['y'][-1], color='red', marker='x', s=60, zorder=7, label=lbl_end)

        # Plot lintasan representatif (rata-rata)
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
    plt.show()

if __name__ == "__main__":
    analyze_sim_vs_real()