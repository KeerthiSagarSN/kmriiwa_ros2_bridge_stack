#!/usr/bin/env python3
import numpy as np
import sys
import os

# Fix numpy/matplotlib compatibility issue
import numpy.core.multiarray
import numpy.core._multiarray_umath

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend first
import matplotlib.pyplot as plt

def plot_navigation_data(npz_file):
    """Load and plot navigation data from NPZ file"""
    
    # Load data
    data = np.load(npz_file)
    time = data['time']
    distance = data['distance']
    inflation_radius = data['inflation_radius']
    
    print(f"Loaded {len(time)} data points from {npz_file}")
    
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Plot distance to goal
    ax1.plot(time, distance, 'b-', linewidth=2, label='Distance to Goal')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Distance (m)', color='b')
    ax1.tick_params(axis='y', labelcolor='b')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_title(f'Navigation Data - {os.path.basename(npz_file)}')
    
    # Plot inflation radius
    ax2.plot(time, inflation_radius, 'r-', linewidth=2, label='Inflation Radius')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Radius (m)', color='r')
    ax2.tick_params(axis='y', labelcolor='r')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plot as image
    output_image = npz_file.replace('.npz', '.png')
    plt.savefig(output_image, dpi=300, bbox_inches='tight')
    print(f"Plot saved to {output_image}")
    
    # Try to show plot (may not work in all environments)
    try:
        matplotlib.use('TkAgg')  # Try interactive backend
        plt.show()
    except:
        print("Interactive display not available, but image saved successfully")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_navigation_data.py <path_to_npz_file>")
        print("\nOr to plot the latest file:")
        data_dir = os.path.expanduser('~/navigation_data')
        if os.path.exists(data_dir):
            files = sorted([f for f in os.listdir(data_dir) if f.endswith('.npz')])
            if files:
                latest_file = os.path.join(data_dir, files[-1])
                print(f"Latest file: {latest_file}")
                plot_navigation_data(latest_file)
            else:
                print(f"No NPZ files found in {data_dir}")
        sys.exit(1)
    
    npz_file = sys.argv[1]
    plot_navigation_data(npz_file)