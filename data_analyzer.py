import pandas as pd
import glob
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.ndimage import distance_transform_edt
from scipy.stats import mannwhitneyu, shapiro

def analyze_experiment_1():
    # Setup konfigurasi path dan peta
    map_file_path = './src/asr_navigation/maps/grit_1_edited_2.pgm'  
    folders = {
        'STD_PSO': './pso_test/STD_PSO',
        'MLPSO': './pso_test/MLPSO'
    }
    
    RESOLUTION = 0.05    
    ORIGIN_X = -13.313419     
    ORIGIN_Y = -14.406021  
    ROBOT_RADIUS = 0.3

    # Warna plot global
    COLOR_WORST = 'red'
    COLOR_AVG = 'blue'
    COLOR_BEST = 'lime'

    # Generate peta EDT untuk evaluasi clearance
    edt_meters = None
    map_img = None
    if os.path.exists(map_file_path):
        map_img = plt.imread(map_file_path)
        map_img = np.flipud(map_img) 
        threshold = 0.5 if map_img.max() <= 1.0 else 127
        free_space = map_img > threshold
        edt_pixels = distance_transform_edt(free_space)
        edt_meters = (edt_pixels * RESOLUTION) - ROBOT_RADIUS
    else:
        print("Peringatan: Map tidak ditemukan. Clearance fisik tidak akan dihitung.")

    results = {}

    # Ekstrak data dan hitung metrik dari tiap percobaan
    for algo_name, folder_path in folders.items():
        print(f"Membaca data {algo_name} di {folder_path}...")
        file_list = glob.glob(f'{folder_path}/run_*_summary_path.csv')
        
        if not file_list:
            print(f"  -> Tidak ada file untuk {algo_name}")
            continue

        all_runs = []

        for file in file_list:
            run_id = os.path.basename(file).split('_')[1]
            df_summary = pd.read_csv(file, nrows=1)
            time_ms = df_summary['Total_Time_ms'].iloc[0]
            
            df_path = pd.read_csv(file, skiprows=2)
            x_coords = df_path['X'].to_numpy()
            y_coords = df_path['Y'].to_numpy()
            
            # Hitung panjang lintasan (Euclidean distance antar titik)
            dx = np.diff(x_coords)
            dy = np.diff(y_coords)
            path_length = np.sum(np.hypot(dx, dy))
            
            # Hitung jarak terdekat ke rintangan menggunakan EDT map
            min_clearance = 0.0
            if edt_meters is not None and len(x_coords) > 0:
                height, width = edt_meters.shape
                px = np.clip(((x_coords - ORIGIN_X) / RESOLUTION).astype(int), 0, width-1)
                py = np.clip(((y_coords - ORIGIN_Y) / RESOLUTION).astype(int), 0, height-1)
                min_clearance = np.min(edt_meters[py, px])

            # Ambil metrik iterasi konvergensi
            conv_files = glob.glob(f'{folder_path}/run_{run_id}_convergence*.csv')
            conv_file = sorted(conv_files)[-1] if conv_files else None 
            
            conv_iter = 0
            conv_iters_arr = []
            conv_scores_arr = []
            
            if conv_file:
                df_conv = pd.read_csv(conv_file)
                iter_col = df_conv.columns[0]
                
                # Standarisasi nama kolom berdasar algoritma
                if algo_name == 'STD_PSO' and 'Raw_Cost' in df_conv.columns:
                    fit_col = 'Raw_Cost'
                elif 'gBest_Score' in df_conv.columns:
                    fit_col = 'gBest_Score'
                else:
                    fit_col = df_conv.columns[2]
                
                # Simpan array ini agar tidak perlu read_csv lagi saat plotting
                conv_iters_arr = df_conv[iter_col].to_numpy()
                conv_scores_arr = df_conv[fit_col].to_numpy()
                
                min_val = np.min(conv_scores_arr)
                steady_idx = np.where(conv_scores_arr == min_val)[0][0]
                conv_iter = conv_iters_arr[steady_idx]
            
            all_runs.append({
                'id': run_id, 
                'time': time_ms, 
                'length': path_length, 
                'clearance': min_clearance, 
                'x': x_coords, 
                'y': y_coords, 
                'conv_iters': conv_iters_arr,
                'conv_scores': conv_scores_arr,
                'conv_iter': conv_iter
            })

        # Kompilasi statistik keseluruhan
        stats = {}
        if all_runs:
            for key in ['length', 'time', 'clearance', 'conv_iter']:
                vals = [r[key] for r in all_runs]
                stats[key] = {
                    'min': np.min(vals), 
                    'max': np.max(vals),
                    'mean': np.mean(vals), 
                    'std': np.std(vals)
                }

            # Tentukan run terbaik dan terburuk murni berdasarkan spasial (clearance + panjang)
            best_run = max(all_runs, key=lambda x: (x['clearance'], -x['length']))
            worst_run = min(all_runs, key=lambda x: (x['clearance'], -x['length']))

            # Cari representasi "run rata-rata" menggunakan jarak centroid/medoid
            for r in all_runs:
                dist = np.sqrt(
                    ((r['length'] - stats['length']['mean']) / (stats['length']['std'] + 1e-6))**2 +
                    ((r['time'] - stats['time']['mean']) / (stats['time']['std'] + 1e-6))**2 +
                    ((r['clearance'] - stats['clearance']['mean']) / (stats['clearance']['std'] + 1e-6))**2
                )
                r['centroid_dist'] = dist
            avg_run = min(all_runs, key=lambda x: x['centroid_dist'])
        else:
            best_run = worst_run = avg_run = {}

        results[algo_name] = {
            'All_Runs': all_runs,
            'Stats': stats,
            'Best': best_run, 
            'Worst': worst_run, 
            'Average': avg_run
        }

    # Print laporan summary ke terminal
    print("\n--- Tabel Ringkasan Karakteristik Lintasan ---")
    for algo_name, data in results.items():
        if not data['All_Runs']: continue
        s = data['Stats']
        
        print(f"\nAlgoritma: {algo_name}")
        print(f"{'Parameter Trajektori':<30} | {'Min':<10} | {'Max':<10} | {'Mean':<10} | {'Std Dev':<10}")
        print("-" * 80)
        print(f"{'Panjang Lintasan (meter)':<30} | {s['length']['min']:<10.2f} | {s['length']['max']:<10.2f} | {s['length']['mean']:<10.2f} | {s['length']['std']:<10.2f}")
        print(f"{'Waktu Komputasi (ms)':<30} | {s['time']['min']:<10.2f} | {s['time']['max']:<10.2f} | {s['time']['mean']:<10.2f} | {s['time']['std']:<10.2f}")
        print(f"{'Clearance Terhadap Halangan (m)':<30} | {s['clearance']['min']:<10.3f} | {s['clearance']['max']:<10.3f} | {s['clearance']['mean']:<10.3f} | {s['clearance']['std']:<10.3f}")
        print(f"{'Iterasi Konvergensi':<30} | {s['conv_iter']['min']:<10.0f} | {s['conv_iter']['max']:<10.0f} | {s['conv_iter']['mean']:<10.0f} | {s['conv_iter']['std']:<10.0f}")
        print("-" * 80)

    # Plot 1: Visualisasi jalur spasial
    fig_paths, (ax_std, ax_ml) = plt.subplots(1, 2, figsize=(18, 8))
    path_axes = {'STD_PSO': ax_std, 'MLPSO': ax_ml}

    if map_img is not None:
        height, width = map_img.shape
        max_x = ORIGIN_X + (width * RESOLUTION)
        max_y = ORIGIN_Y + (height * RESOLUTION)
        extent_bounds = [ORIGIN_X, max_x, ORIGIN_Y, max_y]

    for algo_name, cases in results.items():
        ax = path_axes.get(algo_name)
        if ax is None or not cases['All_Runs']: continue
            
        if map_img is not None:
            ax.imshow(map_img, cmap='gray', origin='lower', extent=extent_bounds)
            ax.set_xlim(ORIGIN_X, max_x)
            ax.set_ylim(ORIGIN_Y, max_y)
        else:
            ax.grid(True)

        # Background: overlay semua jalur untuk melihat variance
        for i, run_data in enumerate(cases['All_Runs']):
            lbl = 'Overlay Semua Percobaan' if i == 0 else None
            ax.plot(run_data['x'], run_data['y'], color='gray', alpha=0.15, linewidth=1.0, label=lbl)

        c_best = cases['Best']
        c_worst = cases['Worst']
        c_avg = cases['Average']

        # Highlight best, average, dan worst runs
        ax.plot(c_worst['x'], c_worst['y'], color=COLOR_WORST, linestyle='--', linewidth=2.0, label='Lintasan Terburuk')
        ax.plot(c_avg['x'], c_avg['y'], color=COLOR_AVG, linestyle='-', linewidth=3.0, label='Lintasan Rata-rata')
        ax.plot(c_best['x'], c_best['y'], color=COLOR_BEST, linestyle='--', linewidth=2.0, label='Lintasan Terbaik')
        
        # Tambahkan marker untuk posisi start dan goal
        if len(c_best['x']) > 0:
            ax.plot(c_best['x'][0], c_best['y'][0], marker='o', color='black', markersize=10, linestyle='none', label='Start Pose')
            ax.plot(c_best['x'][-1], c_best['y'][-1], marker='*', color='black', markersize=15, linestyle='none', label='Goal Pose')
        
        ax.set_title(f'{algo_name}: Analisis Lintasan Spasial')
        ax.set_xlabel('X Koordinat (meter)')
        ax.set_ylabel('Y Koordinat (meter)')
        ax.legend(loc='best', framealpha=0.9)
        ax.axis('equal')

    # Plot 2: Grafik profil konvergensi
    fig_conv, (ax_c_std, ax_c_ml) = plt.subplots(1, 2, figsize=(16, 6))
    conv_axes = {'STD_PSO': ax_c_std, 'MLPSO': ax_c_ml}

    for algo_name, cases in results.items():
        ax_c = conv_axes.get(algo_name)
        if ax_c is None or not cases['All_Runs']: continue

        all_runs_data = cases.get('All_Runs', [])
        
        def plot_conv(run_dict, color, style, width, alpha, label=None):
            # Ambil data langsung dari variabel array untuk menghindari I/O disk ulang
            if len(run_dict.get('conv_iters', [])) > 0:
                ax_c.plot(run_dict['conv_iters'], run_dict['conv_scores'], 
                          color=color, linestyle=style, linewidth=width, alpha=alpha, label=label)

        # Background: overlay semua konvergensi
        for i, run in enumerate(all_runs_data):
            lbl = 'Overlay Semua Percobaan' if i == 0 else None
            plot_conv(run, color='gray', style='-', width=1.0, alpha=0.15, label=lbl)

        # Cari ekstrem konvergensi murni berdasar jumlah iterasi
        c_best_conv = min(all_runs_data, key=lambda x: x['conv_iter'])
        c_worst_conv = max(all_runs_data, key=lambda x: x['conv_iter'])
        mean_conv_val = cases['Stats']['conv_iter']['mean']
        c_avg_conv = min(all_runs_data, key=lambda x: abs(x['conv_iter'] - mean_conv_val))

        plot_conv(c_worst_conv, color=COLOR_WORST, style='--', width=2.0, alpha=1.0, 
                  label=f'Terlambat (Iter {c_worst_conv["conv_iter"]})')
        plot_conv(c_avg_conv, color=COLOR_AVG, style='-', width=3.0, alpha=1.0, 
                  label=f'Rata-rata (Iter {c_avg_conv["conv_iter"]})')
        plot_conv(c_best_conv, color=COLOR_BEST, style='--', width=2.0, alpha=1.0, 
                  label=f'Tercepat (Iter {c_best_conv["conv_iter"]})')
        
        ax_c.set_title(f'{algo_name}: Profil Konvergensi')
        ax_c.set_xlabel('Iterasi')
        ax_c.set_ylabel('Cost / Fitness Score')
        ax_c.set_yscale('log')
        ax_c.grid(True, which="both", ls="--", alpha=0.5)
        ax_c.legend(loc='best', framealpha=0.9)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    analyze_experiment_1()