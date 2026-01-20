#!/usr/bin/env python3
"""
Joint Position Comparison Plotter
比較 Unity 原始目標與 OpenArm 實際位置的對應關係

使用方式：
    python3 plot_joint_comparison.py joint_data_20260120_143000.pkl
    
    或指定手臂：
    python3 plot_joint_comparison.py joint_data_20260120_143000.pkl --arm left
    python3 plot_joint_comparison.py joint_data_20260120_143000.pkl --arm right
    python3 plot_joint_comparison.py joint_data_20260120_143000.pkl --arm both
"""

import matplotlib.pyplot as plt
import matplotlib
import pickle
import os
import sys
import argparse
from datetime import datetime
from typing import Optional

# 使用非 GUI backend（適用於無顯示器環境）
matplotlib.use('Agg')

# 設定中文字體（如果可用）
try:
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica']
    plt.rcParams['axes.unicode_minus'] = False
except:
    pass


def plot_joint_comparison(
    data_file: str,
    arm: str = 'both',
    output_dir: Optional[str] = None,
    show_error: bool = True
) -> list:
    """
    繪製關節位置比較圖
    
    Args:
        data_file: 數據檔案路徑 (.pkl)
        arm: 繪製的手臂 ('left', 'right', 'both')
        output_dir: 輸出目錄，預設為 utils/pictures
        show_error: 是否顯示追蹤誤差
    
    Returns:
        list: 產生的圖檔路徑列表
    """
    # 載入數據
    with open(data_file, 'rb') as f:
        data = pickle.load(f)
    
    timestamps = data['timestamp']
    
    if len(timestamps) == 0:
        print("錯誤：數據檔案是空的")
        return []
    
    # 設定輸出目錄
    if output_dir is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, 'pictures')
    
    os.makedirs(output_dir, exist_ok=True)
    
    # 產生時間戳
    file_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    output_files = []
    arms_to_plot = []
    
    if arm in ['left', 'both']:
        arms_to_plot.append('left')
    if arm in ['right', 'both']:
        arms_to_plot.append('right')
    
    for arm_name in arms_to_plot:
        unity_key = f'{arm_name}_unity_target'
        actual_key = f'{arm_name}_actual'
        
        # 為每個關節繪製一張圖
        for joint_idx in range(7):
            joint_num = joint_idx + 1
            
            fig, ax = plt.subplots(figsize=(12, 5))
            
            unity_data = data[unity_key][joint_idx]
            actual_data = data[actual_key][joint_idx]
            
            # 繪製 Unity 目標和 OpenArm 實際位置
            ax.plot(timestamps, unity_data, 'b-', 
                   label='Unity Target', linewidth=1.5, alpha=0.8)
            ax.plot(timestamps, actual_data, 'r-', 
                   label='OpenArm Actual', linewidth=1.5, alpha=0.8)
            
            # 如果需要顯示誤差
            if show_error:
                error = [u - a for u, a in zip(unity_data, actual_data)]
                ax2 = ax.twinx()
                ax2.fill_between(timestamps, error, alpha=0.2, color='green', label='Error')
                ax2.set_ylabel('Error (rad)', color='green')
                ax2.tick_params(axis='y', labelcolor='green')
                ax2.axhline(y=0, color='green', linestyle='--', alpha=0.3)
            
            # 設定圖表
            arm_label = 'Left' if arm_name == 'left' else 'Right'
            ax.set_title(f'{arm_label} Arm - Joint {joint_num}: Unity Target vs OpenArm Actual')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Position (rad)')
            ax.legend(loc='upper left')
            ax.grid(True, alpha=0.3)
            
            # 儲存圖片
            filename = f"{arm_name}_joint{joint_num}_{file_timestamp}.jpg"
            filepath = os.path.join(output_dir, filename)
            
            plt.tight_layout()
            plt.savefig(filepath, dpi=150, format='jpg', 
                       bbox_inches='tight', facecolor='white')
            plt.close()
            
            output_files.append(filepath)
            print(f"已儲存: {filename}")
    
    print(f"\n共產生 {len(output_files)} 張圖，儲存於: {output_dir}")
    return output_files


def plot_all_joints_combined(
    data_file: str,
    arm: str = 'left',
    output_dir: Optional[str] = None
) -> str:
    """
    繪製所有關節的組合圖（7 個子圖）
    
    Args:
        data_file: 數據檔案路徑
        arm: 手臂 ('left' 或 'right')
        output_dir: 輸出目錄
    
    Returns:
        str: 產生的圖檔路徑
    """
    with open(data_file, 'rb') as f:
        data = pickle.load(f)
    
    timestamps = data['timestamp']
    
    if output_dir is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, 'pictures')
    
    os.makedirs(output_dir, exist_ok=True)
    
    file_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    unity_key = f'{arm}_unity_target'
    actual_key = f'{arm}_actual'
    
    fig, axes = plt.subplots(7, 1, figsize=(14, 18), sharex=True)
    
    arm_label = 'Left' if arm == 'left' else 'Right'
    fig.suptitle(f'{arm_label} Arm: Unity Target vs OpenArm Actual Position', 
                 fontsize=14, fontweight='bold')
    
    for i in range(7):
        ax = axes[i]
        unity_data = data[unity_key][i]
        actual_data = data[actual_key][i]
        
        ax.plot(timestamps, unity_data, 'b-', 
               label='Unity Target', linewidth=1.2, alpha=0.8)
        ax.plot(timestamps, actual_data, 'r-', 
               label='OpenArm Actual', linewidth=1.2, alpha=0.8)
        
        ax.set_ylabel(f'Joint {i+1}\n(rad)', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        
        # 計算並顯示平均誤差
        error = [abs(u - a) for u, a in zip(unity_data, actual_data)]
        avg_error = sum(error) / len(error) if error else 0
        ax.text(0.02, 0.95, f'Avg Error: {avg_error:.4f} rad', 
               transform=ax.transAxes, fontsize=8,
               verticalalignment='top', 
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    axes[-1].set_xlabel('Time (s)', fontsize=12)
    
    plt.tight_layout()
    
    filename = f"{arm}_all_joints_{file_timestamp}.jpg"
    filepath = os.path.join(output_dir, filename)
    plt.savefig(filepath, dpi=150, format='jpg', 
               bbox_inches='tight', facecolor='white')
    plt.close()
    
    print(f"已儲存組合圖: {filename}")
    return filepath


def main():
    parser = argparse.ArgumentParser(
        description='繪製 Unity 目標與 OpenArm 實際位置的比較圖'
    )
    parser.add_argument(
        'data_file',
        help='數據檔案路徑 (.pkl)'
    )
    parser.add_argument(
        '--arm', '-a',
        choices=['left', 'right', 'both'],
        default='both',
        help='繪製的手臂 (預設: both)'
    )
    parser.add_argument(
        '--output', '-o',
        help='輸出目錄'
    )
    parser.add_argument(
        '--combined', '-c',
        action='store_true',
        help='額外產生組合圖（7 個關節在同一張圖）'
    )
    parser.add_argument(
        '--no-error',
        action='store_true',
        help='不顯示追蹤誤差'
    )
    
    args = parser.parse_args()
    
    if not os.path.exists(args.data_file):
        print(f"錯誤：找不到檔案 {args.data_file}")
        sys.exit(1)
    
    # 繪製個別關節圖
    plot_joint_comparison(
        args.data_file,
        arm=args.arm,
        output_dir=args.output,
        show_error=not args.no_error
    )
    
    # 繪製組合圖
    if args.combined:
        if args.arm in ['left', 'both']:
            plot_all_joints_combined(args.data_file, 'left', args.output)
        if args.arm in ['right', 'both']:
            plot_all_joints_combined(args.data_file, 'right', args.output)


if __name__ == '__main__':
    main()
