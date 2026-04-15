import pandas as pd
import os
import glob
import yaml
import matplotlib.pyplot as plt
import numpy as np

# ==========================================
# USER CONFIGURATION: Set your file paths here
# ==========================================
# Reference file (e.g., Water Displacement measurements)
REF_FILE = r"E:\multi_source_info\lidar\排水法测体积.xlsx"

# Generated files to compare (list of CSV/XLSX files)
GEN_FILES = [
    # r"E:\multi_source_info\lidar\results_0309_0p002_by_mask\banyan_Detong_1_63_347.csv",
    # r"E:\multi_source_info\lidar\results_0309_0p002_by_lidar\banyan_Detong_1_63_347.csv",
    # r"E:\multi_source_info\lidar\results\results_0325_yinshan_0p002_by_mask\1_98_position_1_160kV.csv"
    r"E:\multi_source_info\lidar\results\results_0325_yinshan_0p002_by_lidar\1_98_position_1_160kV.csv"

    # Add more files here to compare together
]
# ==========================================

def compare_volumes():
    # 1. Validate paths
    if not os.path.exists(REF_FILE):
        print(f"Error: Reference file not found at {REF_FILE}")
        return

    valid_gen_files = []
    for f in GEN_FILES:
        if os.path.exists(f):
            valid_gen_files.append(f)
        else:
            print(f"Warning: Generated file not found: {f}")
    
    if not valid_gen_files:
        print("Error: No valid generated files found to compare.")
        return

    # 2. Read Reference data
    try:
        df_ref = pd.read_excel(REF_FILE, sheet_name="0325")
        # Assuming last column is volume (ml) and first is ID
        ref_ids = df_ref.iloc[:, 0].values
        ref_vols = df_ref.iloc[:, -1].values
        ref_map = {str(int(k)): v for k, v in zip(ref_ids, ref_vols) if pd.notnull(k)}
    except Exception as e:
        print(f"Error reading reference file: {e}")
        return

    # 3. Process each generated file
    all_results = [] # List of dicts {label, ids, gen_vols, ref_vols, rel_errs}

    for gen_file in valid_gen_files:
        label = os.path.basename(os.path.dirname(gen_file)) + " / " + os.path.basename(gen_file)
        try:
            # Check if CSV or TSV (xlsx-named TSV)
            if gen_file.endswith('.csv'):
                df_gen = pd.read_csv(gen_file, skiprows=2)
            else:
                df_gen = pd.read_csv(gen_file, sep='\t', skiprows=2)
                
            gen_ids = df_gen['ID'].values
            gen_vols = df_gen['Volume(cm3)'].values
            
            plot_ids = []
            plot_ref_vols = []
            plot_gen_vols = []
            plot_rel_errs = []

            for gid_raw, gvol in zip(gen_ids, gen_vols):
                gid_str = str(gid_raw)
                if gid_str.startswith('ore_'):
                    gid_num = int(gid_str.replace('ore_', '')) + 1
                else:
                    try:
                        gid_num = int(float(gid_str))
                    except:
                        gid_num = gid_str

                search_key = str(gid_num)
                if search_key in ref_map:
                    rvol = ref_map[search_key]
                    err = abs(gvol - rvol)
                    rel_err = (err / rvol) * 100 if rvol != 0 else 0
                    
                    plot_ids.append(str(gid_num))
                    plot_ref_vols.append(rvol)
                    plot_gen_vols.append(gvol)
                    plot_rel_errs.append(rel_err)
            
            if plot_ids:
                all_results.append({
                    'label': label,
                    'ids': plot_ids,
                    'ref_vols': plot_ref_vols,
                    'gen_vols': plot_gen_vols,
                    'rel_errs': plot_rel_errs
                })
        except Exception as e:
            print(f"Error processing {gen_file}: {e}")

    # 4. Plotting
    if not all_results:
        print("No matching data found for plotting.")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    cmap = plt.get_cmap('tab10')

    # Subplot 1: Volume Comparison
    # Plot reference only once (from the first result set)
    ref_ids = all_results[0]['ids']
    ref_vols = all_results[0]['ref_vols']
    ax1.plot(ref_ids, ref_vols, marker='o', linestyle='-', label='Reference (ml)', color='black', linewidth=2, alpha=0.6)
    
    for i, res in enumerate(all_results):
        ax1.plot(res['ids'], res['gen_vols'], marker='s', linestyle='--', 
                 label=f"LiDAR: {res['label']}", color=cmap(i))
    
    ax1.set_ylabel('Volume')
    ax1.set_title('Volume Comparison: LiDAR vs Reference')
    ax1.legend()
    ax1.grid(True, linestyle='--', alpha=0.7)

    # Subplot 2: Relative Error
    for i, res in enumerate(all_results):
        ax2.plot(res['ids'], res['rel_errs'], marker='^', label=f"Error: {res['label']}", color=cmap(i))
    
    ax2.axhline(y=10, color='red', linestyle=':', label='10% Error Threshold')
    ax2.set_xlabel('Ore ID')
    ax2.set_ylabel('Relative Error (%)')
    ax2.set_title('Relative Error Analysis')
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend()

    plt.tight_layout()
    
    try:
        plt.show()
    except:
        pass

if __name__ == "__main__":
    compare_volumes()
