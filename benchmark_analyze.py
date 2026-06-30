import pandas as pd
import glob
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.ndimage import distance_transform_edt

# Membaca data metrik dan path dari file CSV di dalam folder
def process_folder(folder_path, edt_meters, ORIGIN_X, ORIGIN_Y, RESOLUTION):
    file_list = glob.glob(f'{folder_path}/run_*_summary_path.csv')
    if not file_list:
        return None
        
    times, dists, turns, edges, fitness, clearances = [], [], [], [], [], []
    all_paths = {}

    for file in file_list:
        # Ekstrak metrik summary
        df_summary = pd.read_csv(file, nrows=1)
        times.append(df_summary['Total_Time_ms'].iloc[0])
        dists.append(df_summary['Dist_Cost'].iloc[0])
        turns.append(df_summary['Turn_Cost'].iloc[0])
        edges.append(df_summary['Edge_Cost'].iloc[0])
        fitness.append(df_summary['Final_Fitness'].iloc[0])
        
        # Ekstrak data lintasan
        df_path = pd.read_csv(file, skiprows=2)
        all_paths[file] = df_path
        
        # Hitung jarak terdekat ke rintangan berdasarkan peta EDT
        if edt_meters is not None:
            height, width = edt_meters.shape
            px = np.clip(((df_path['X'] - ORIGIN_X) / RESOLUTION).astype(int), 0, width-1)
            py = np.clip(((df_path['Y'] - ORIGIN_Y) / RESOLUTION).astype(int), 0, height-1)
            min_clear = np.min(edt_meters[py, px])
            clearances.append(min_clear)
        else:
            clearances.append(0.0)

    return {
        'times': np.array(times),
        'dists': np.array(dists),
        'turns': np.array(turns) * (180.0 / np.pi), # Rad to Deg
        'edges': np.array(edges),
        'fitness': np.array(fitness),
        'clearances': np.array(clearances),
        'paths': all_paths,
        'files': file_list
    }

def analyze_benchmark():
    # Setup path dan parameter map
    astar_folder = './benchmark_test/hard/ASTAR'
    mlpso_folder = './benchmark_test/hard/ASTAR'
    map_file_path = './src/asr_navigation/maps/grit_1_edited_2.pgm'  
    
    RESOLUTION = 0.05    
    ORIGIN_X = -13.313419     
    ORIGIN_Y = -14.406021  
    ROBOT_RADIUS = 0.3  
    TOLERANCE = 0.5

    # Generate peta jarak obstacle (EDT) untuk evaluasi clearance
    edt_meters, map_img = None, None
    if os.path.exists(map_file_path):
        map_img = plt.imread(map_file_path)
        map_img = np.flipud(map_img)
        threshold = 0.5 if map_img.max() <= 1.0 else 127
        free_space = map_img > threshold
        edt_pixels = distance_transform_edt(free_space)
        edt_meters = (edt_pixels * RESOLUTION) - ROBOT_RADIUS

    print("Mengekstrak data MLPSO...")
    mlpso_data = process_folder(mlpso_folder, edt_meters, ORIGIN_X, ORIGIN_Y, RESOLUTION)
    if mlpso_data is None:
        print(f"Data tidak ditemukan di folder: {mlpso_folder}")
        return

    print("Mengekstrak data A* sebagai ground truth...")
    astar_data = process_folder(astar_folder, edt_meters, ORIGIN_X, ORIGIN_Y, RESOLUTION)
    base_fitness = np.mean(astar_data['fitness']) if astar_data is not None else 0.0

    # Kalkulasi rasio penemuan jalur optimal
    total_runs = len(mlpso_data['files'])
    optimal_runs = 0
    if base_fitness > 0:
        optimal_mask = (mlpso_data['fitness'] <= (base_fitness + TOLERANCE)) & (mlpso_data['edges'] == 0)
        optimal_runs = np.sum(optimal_mask)

    # Cari run terbaik dan terburuk berdasarkan waktu komputasi dan fitness
    sorted_indices = np.lexsort((mlpso_data['times'], mlpso_data['fitness']))
    best_idx = sorted_indices[0]
    worst_idx = sorted_indices[-1]

    best_file = os.path.basename(mlpso_data['files'][best_idx])
    worst_file = os.path.basename(mlpso_data['files'][worst_idx])

    # Cetak laporan hasil benchmark
    print("\n--- Laporan Hasil Benchmark MLPSO ---")
    print(f"Rasio Jalur Optimal : {optimal_runs} / {total_runs} trial")
    print("-" * 37)
    
    print(f"\n1. Run Terbaik ({best_file})")
    print(f"   Waktu Komputasi : {mlpso_data['times'][best_idx]:.2f} ms")
    print(f"   Fitness         : {mlpso_data['fitness'][best_idx]:.5f}")
    print(f"   Panjang Jalur   : {mlpso_data['dists'][best_idx]:.2f} m")
    print(f"   Clearance Min   : {mlpso_data['clearances'][best_idx]:.3f} m")
    print(f"   Total Belokan   : {mlpso_data['turns'][best_idx]:.1f}°")

    print(f"\n2. Run Terburuk ({worst_file})")
    print(f"   Waktu Komputasi : {mlpso_data['times'][worst_idx]:.2f} ms")
    print(f"   Fitness         : {mlpso_data['fitness'][worst_idx]:.5f}")
    print(f"   Panjang Jalur   : {mlpso_data['dists'][worst_idx]:.2f} m")
    print(f"   Clearance Min   : {mlpso_data['clearances'][worst_idx]:.3f} m")
    print(f"   Total Belokan   : {mlpso_data['turns'][worst_idx]:.1f}°")

    print(f"\n3. Rata-rata dari {total_runs} Trial (Mean ± SD)")
    print(f"   Waktu Komputasi : {np.mean(mlpso_data['times']):.2f} ± {np.std(mlpso_data['times'], ddof=1):.2f} ms")
    print(f"   Fitness         : {np.mean(mlpso_data['fitness']):.5f} ± {np.std(mlpso_data['fitness'], ddof=1):.5f}")
    print(f"   Panjang Jalur   : {np.mean(mlpso_data['dists']):.2f} ± {np.std(mlpso_data['dists'], ddof=1):.2f} m")
    print(f"   Clearance Min   : {np.mean(mlpso_data['clearances']):.3f} ± {np.std(mlpso_data['clearances'], ddof=1):.3f} m")
    print(f"   Total Belokan   : {np.mean(mlpso_data['turns']):.1f} ± {np.std(mlpso_data['turns'], ddof=1):.1f}°")
    print("-" * 37)

    # Plot visualisasi 2D
    plt.figure(figsize=(10, 10))
    if map_img is not None:
        height, width = map_img.shape
        max_x = ORIGIN_X + (width * RESOLUTION)
        max_y = ORIGIN_Y + (height * RESOLUTION)
        extent_bounds = [ORIGIN_X, max_x, ORIGIN_Y, max_y]
        plt.imshow(map_img, cmap='gray', origin='lower', extent=extent_bounds)
    else:
        plt.grid(True)

    # Plot jalur A* (baseline)
    if astar_data is not None:
        first_astar = True
        for file, path_df in astar_data['paths'].items():
            plt.plot(path_df['X'].to_numpy(), path_df['Y'].to_numpy(), color='lime', linewidth=3.0, alpha=0.3, 
                     label='A* Baseline' if first_astar else None)
            first_astar = False

    # Plot jalur MLPSO
    first_suboptimal = True
    first_optimal = True

    for idx, file in enumerate(mlpso_data['files']):
        path_df = mlpso_data['paths'][file]
        is_optimal = (mlpso_data['fitness'][idx] <= (base_fitness + TOLERANCE)) and (mlpso_data['edges'][idx] == 0)
        
        if is_optimal:
            plt.plot(path_df['X'].to_numpy(), path_df['Y'].to_numpy(), 
                     color='blue', linewidth=2.0, linestyle='--', alpha=0.8, 
                     label='Optimal MLPSO' if first_optimal else None)
            first_optimal = False
        else:
            plt.plot(path_df['X'].to_numpy(), path_df['Y'].to_numpy(), 
                     color='red', linewidth=2.0, linestyle='--', alpha=0.6, 
                     label='Suboptimal MLPSO' if first_suboptimal else None)
            first_suboptimal = False

    # Plot titik Start dan Goal
    best_path_df = mlpso_data['paths'][mlpso_data['files'][best_idx]]
    start_x, start_y = best_path_df['X'].iloc[0], best_path_df['Y'].iloc[0]
    goal_x, goal_y = best_path_df['X'].iloc[-1], best_path_df['Y'].iloc[-1]

    plt.plot(start_x, start_y, 'ko', markersize=10, label='Start Pose')
    plt.plot(goal_x, goal_y, 'k*', markersize=15, label='Goal Pose')

    plt.title('Evaluasi Lintasan Spasial MLPSO')
    plt.xlabel('X Coordinates (meters)')
    plt.ylabel('Y Coordinates (meters)')
    plt.legend(loc='best')
    plt.axis('equal') 
    
    if map_img is not None:
        plt.xlim(ORIGIN_X, max_x)
        plt.ylim(ORIGIN_Y, max_y)
        
    plt.show()

if __name__ == "__main__":
    analyze_benchmark()