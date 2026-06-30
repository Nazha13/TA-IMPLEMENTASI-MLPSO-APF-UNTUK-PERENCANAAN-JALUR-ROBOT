import pandas as pd
import glob
import numpy as np
import matplotlib.pyplot as plt
import os
import re
from scipy.ndimage import distance_transform_edt

def analyze_local_planners():
    # Setup konfigurasi skenario dan parameter lingkungan
    TARGET_SCENARIO = "Scenario 1" 
    
    base_folder = f'./local_planner_test/{TARGET_SCENARIO}'
    map_file_path = './src/asr_navigation/maps/grit_1_edited_2.pgm'  
    
    RESOLUTION = 0.05    
    ORIGIN_X = -13.313419 
    ORIGIN_Y = -14.406021 

    ROBOT_RADIUS = 0.3
    OBS_RADIUS = 0.293191

    # Inisialisasi koordinat rintangan dinamis (unmapped) berdasarkan skenario
    if TARGET_SCENARIO == "Scenario 1":
        obstacles = [(3.782688, 1.816288), (6.413248, 2.487809)]
    elif TARGET_SCENARIO == "Scenario 2":
        obstacles = [(4.121788, 1.354628), (5.820588, 2.851258)]
    else:
        obstacles = []

    algorithms = ['APF', 'TEB', 'DWA']
    batches = ['Batch 1', 'Batch 2']
    
    results = {}

    # Penyiapan Peta EDT (Euclidean Distance Transform) Global
    print("Menyiapkan Global EDT Map...")
    distance_map_m = None
    map_img = None
    
    if os.path.exists(map_file_path):
        map_img = plt.imread(map_file_path)
        # Konversi ke 2D (grayscale) jika format asli gambar adalah RGB (3 channel)
        if map_img.ndim == 3: 
            map_img = map_img.mean(axis=2)
            
        map_img = np.flipud(map_img)
        rows, cols = map_img.shape
        
        # Grid biner: nilai piksel < 0.5 (gelap) dianggap sebagai tembok/rintangan
        obstacle_grid = (map_img < 0.5) 
        
        # Integrasikan rintangan unmapped ke dalam grid spasial
        if obstacles:
            y_grid_idx, x_grid_idx = np.mgrid[0:rows, 0:cols]
            x_real = ORIGIN_X + (x_grid_idx * RESOLUTION)
            y_real = ORIGIN_Y + (y_grid_idx * RESOLUTION)
            
            for obs_x, obs_y in obstacles:
                dist_to_obs = np.hypot(x_real - obs_x, y_real - obs_y)
                obstacle_grid[dist_to_obs <= OBS_RADIUS] = True

        # Kalkulasi matriks jarak (EDT)
        edt_input = ~obstacle_grid 
        distance_map_m = distance_transform_edt(edt_input) * RESOLUTION

    # Pemrosesan metrik dan ekstraksi data trajektori
    print(f"Memproses data untuk {TARGET_SCENARIO}...\n")
    
    for algo in algorithms:
        algo_metrics = []
        all_trajectories = []
        
        for batch in batches:
            batch_path = os.path.join(base_folder, algo, batch)
            if not os.path.exists(batch_path):
                continue
                
            metric_file = os.path.join(batch_path, 'hasil_pengujian_metrik.csv')
            df_metrics = pd.DataFrame()
            if os.path.exists(metric_file):
                df_metrics = pd.read_csv(metric_file)
                df_metrics['Min Clearance (m)'] = pd.to_numeric(df_metrics['Min Clearance (m)'], errors='coerce')
                
            traj_files = glob.glob(f'{batch_path}/trajectories/*.csv')
            for t_file in traj_files:
                df_traj = pd.read_csv(t_file)
                
                if 'X' in df_traj.columns and 'Y' in df_traj.columns:
                    x_coords = df_traj['X'].to_numpy()
                    y_coords = df_traj['Y'].to_numpy()
                    
                    dx = np.diff(x_coords)
                    dy = np.diff(y_coords)
                    path_length = np.sum(np.hypot(dx, dy))
                    
                    # Rekalkulasi min clearance menggunakan data spasial rute
                    trial_min_clearance = float('inf')
                    if distance_map_m is not None:
                        traj_cols = np.round((x_coords - ORIGIN_X) / RESOLUTION).astype(int)
                        traj_rows = np.round((y_coords - ORIGIN_Y) / RESOLUTION).astype(int)
                        
                        traj_cols = np.clip(traj_cols, 0, distance_map_m.shape[1] - 1)
                        traj_rows = np.clip(traj_rows, 0, distance_map_m.shape[0] - 1)
                        
                        clearances = distance_map_m[traj_rows, traj_cols] - ROBOT_RADIUS
                        trial_min_clearance = np.min(clearances)
                    
                    match = re.search(r'trial(\d+)\.csv', t_file)
                    status_val = "Unknown"
                    trial_num = "Unknown"
                    
                    if match and not df_metrics.empty:
                        trial_num = int(match.group(1))
                        
                        # Override clearance di dataframe dengan hasil pengukuran ulang
                        if trial_min_clearance != float('inf'):
                            df_metrics.loc[df_metrics['Trial'] == trial_num, 'Min Clearance (m)'] = trial_min_clearance

                        status_row = df_metrics[df_metrics['Trial'] == trial_num]
                        if not status_row.empty:
                            status_val = status_row['Status'].iloc[0]
                    
                    all_trajectories.append({
                        'file': t_file,
                        'length': path_length,
                        'x': x_coords,
                        'y': y_coords,
                        'status': status_val,
                        'trial': trial_num,
                        'batch': batch
                    })
            
            if not df_metrics.empty:
                algo_metrics.append(df_metrics)
                    
        if not algo_metrics or not all_trajectories:
            print(f"Data tidak ditemukan untuk algoritma {algo}")
            continue
            
        combined_metrics = pd.concat(algo_metrics, ignore_index=True)
        
        # Agregasi data statistik
        stats = {
            'dur_avg': combined_metrics['Duration (s)'].mean(),
            'dur_min': combined_metrics['Duration (s)'].min(),
            'dur_max': combined_metrics['Duration (s)'].max(),
            'dur_std': combined_metrics['Duration (s)'].std(),
            
            'clear_avg': combined_metrics['Min Clearance (m)'].mean(),
            'clear_min': combined_metrics['Min Clearance (m)'].min(),
            'clear_max': combined_metrics['Min Clearance (m)'].max(),
            'clear_std': combined_metrics['Min Clearance (m)'].std(),
            
            'len_avg': combined_metrics['Path Length (m)'].mean(),
            'len_min': combined_metrics['Path Length (m)'].min(),
            'len_max': combined_metrics['Path Length (m)'].max(),
            'len_std': combined_metrics['Path Length (m)'].std(),
            
            'cpu_avg': combined_metrics['Avg CPU (%)'].mean(),
            'cpu_min': combined_metrics['Avg CPU (%)'].min(),
            'cpu_max': combined_metrics['Avg CPU (%)'].max(),
            'cpu_std': combined_metrics['Avg CPU (%)'].std()
        }
        
        status_counts = combined_metrics['Status'].value_counts().to_dict()
        success_direct = status_counts.get('Success_Direct', 0)
        success_replanned = status_counts.get('Success_Replanned', 0)
        failed_timeout = status_counts.get('Failed_Timeout', 0) 
        total_runs = len(combined_metrics)
        
        # Pemilihan trajektori representatif untuk di-plot
        direct_trajs = [t for t in all_trajectories if t['status'] == 'Success_Direct']
        replan_trajs = [t for t in all_trajectories if t['status'] == 'Success_Replanned']
        failed_trajs = [t for t in all_trajectories if t['status'] == 'Failed_Timeout']
        
        trajs_to_plot = []
        
        if direct_trajs:
            mean_len = np.mean([t['length'] for t in direct_trajs])
            rep = min(direct_trajs, key=lambda t: abs(t['length'] - mean_len))
            rep['plot_type'] = 'Success_Direct'
            trajs_to_plot.append(rep)
            
        if replan_trajs:
            mean_len = np.mean([t['length'] for t in replan_trajs])
            rep = min(replan_trajs, key=lambda t: abs(t['length'] - mean_len))
            rep['plot_type'] = 'Success_Replanned'
            trajs_to_plot.append(rep)
            
        if not direct_trajs and not replan_trajs and failed_trajs:
            mean_len = np.mean([t['length'] for t in failed_trajs])
            rep = min(failed_trajs, key=lambda t: abs(t['length'] - mean_len))
            rep['plot_type'] = 'Failed_Timeout'
            trajs_to_plot.append(rep)
        
        results[algo] = {
            'runs': total_runs,
            'stats': stats,
            'raw_data': {  
                'duration': combined_metrics['Duration (s)'].dropna().tolist(),
                'clearance': combined_metrics['Min Clearance (m)'].dropna().tolist(),
                'length': combined_metrics['Path Length (m)'].dropna().tolist(),
                'cpu': combined_metrics['Avg CPU (%)'].dropna().tolist()
            },
            'status': {
                'direct': success_direct,
                'replanned': success_replanned,
                'failed': failed_timeout
            },
            'all_trajectories': all_trajectories, 
            'trajs_to_plot': trajs_to_plot
        }

    # Cetak laporan benchmark ke terminal
    print("\n--- Ringkasan Benchmark Local Planner ---")
    for algo, data in results.items():
        st = data['stats']
        print(f"\nAlgoritma: {algo} ({data['runs']} Total Runs)")
        
        print("  1. Waktu Komputasi (detik)")
        print(f"     Min/Max/Mean : {st['dur_min']:.2f} / {st['dur_max']:.2f} / {st['dur_avg']:.2f}")
        print(f"     Std Deviasi  : {st['dur_std']:.2f}")
        
        print("  2. Panjang Lintasan (meter)")
        print(f"     Min/Max/Mean : {st['len_min']:.2f} / {st['len_max']:.2f} / {st['len_avg']:.2f}")
        print(f"     Std Deviasi  : {st['len_std']:.2f}")
        
        print("  3. Min Clearance (meter)")
        print(f"     Min/Max/Mean : {st['clear_min']:.3f} / {st['clear_max']:.3f} / {st['clear_avg']:.3f}")
        print(f"     Std Deviasi  : {st['clear_std']:.3f}")

        print("  4. Konsumsi CPU (%)")
        print(f"     Min/Max/Mean : {st['cpu_min']:.1f} / {st['cpu_max']:.1f} / {st['cpu_avg']:.1f}")
        print(f"     Std Deviasi  : {st['cpu_std']:.1f}")

        print("\n  Status Navigasi:")
        print(f"     Direct Success    : {data['status']['direct']}")
        print(f"     Replanned Success : {data['status']['replanned']}")
        print(f"     Failed (Timeout)  : {data['status']['failed']}")
        print("-" * 45)

    # Visualisasi perbandingan trajektori pada peta
    num_algos = len(results)
    if num_algos == 0: return

    fig, axes = plt.subplots(1, num_algos, figsize=(6 * num_algos, 8))
    if num_algos == 1: axes = [axes]

    # Ambil referensi titik Start/Goal dari algoritma pertama untuk konsistensi antar subplot
    ref_algo = list(results.keys())[0]
    ref_traj = results[ref_algo]['all_trajectories'][0]
    ref_start, ref_goal = (ref_traj['x'][0], ref_traj['y'][0]), (ref_traj['x'][-1], ref_traj['y'][-1])

    for idx, (algo, data) in enumerate(results.items()):
        ax = axes[idx]
        
        # Render map menggunakan map_img yang sudah di-load di awal skrip
        if map_img is not None:
            max_x = ORIGIN_X + (map_img.shape[1] * RESOLUTION)
            max_y = ORIGIN_Y + (map_img.shape[0] * RESOLUTION)
            ax.imshow(map_img, cmap='gray', origin='lower', extent=[ORIGIN_X, max_x, ORIGIN_Y, max_y])
        else:
            ax.grid(True)
        
        # Render rintangan statis dari variabel obstacles yang sudah dibuat
        for ox, oy in obstacles:
            ax.add_patch(plt.Circle((ox, oy), radius=OBS_RADIUS, color='black', alpha=0.7, zorder=4))
        
        # Render garis bayangan (overlay) untuk semua riwayat percobaan
        for i, traj in enumerate(data['all_trajectories']):
            is_failed = 'Failed' in str(traj['status'])
            ax.plot(traj['x'], traj['y'], color='red' if is_failed else 'gray', 
                    linewidth=1.2 if is_failed else 0.8, alpha=0.3 if is_failed else 0.05, zorder=2 if is_failed else 1)

        # Render garis tegas untuk rute yang merepresentasikan kelompoknya
        for traj in data['trajs_to_plot']:
            color = 'blue' if 'Success' in traj['plot_type'] else 'red'
            ls = '-' if 'Direct' in traj['plot_type'] else ':'
            ax.plot(traj['x'], traj['y'], color=color, linestyle=ls, linewidth=2.5, label=f"Rep: {traj['plot_type']}")
        
        # Render indikator Start dan Goal
        ax.scatter(ref_start[0], ref_start[1], color='black', s=150, edgecolors='black', label='Start', zorder=10)
        ax.scatter(8.5, 5.5, color='black', s=200, marker='*', edgecolors='black', label='Goal', zorder=10)

        # Marker penanda untuk kasus kegagalan (Timeout/Stuck)
        ax.plot([], [], 'rx', label='Failed (Timeout/Stuck)', markersize=10, markeredgewidth=2)
        for traj in data['all_trajectories']:
            if 'Failed' in str(traj['status']):
                ax.text(traj['x'][-1], traj['y'][-1], 'X', color='red', fontsize=15, fontweight='bold', ha='center', va='center', zorder=10)

        ax.set_title(f'{algo} Planner', fontsize=14, fontweight='bold')
        ax.legend(loc='lower right', fontsize=9)
        ax.axis('equal')
        if map_img is not None:
            ax.set_xlim(ORIGIN_X, ORIGIN_X + (map_img.shape[1] * RESOLUTION))
            ax.set_ylim(ORIGIN_Y, ORIGIN_Y + (map_img.shape[0] * RESOLUTION))

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    analyze_local_planners()