#!/usr/bin/env python3

import sys
import os
import cv2
import numpy as np
import yaml

def convert_png_to_pgm_yaml(png_path, output_dir=None, resolution=0.05, origin=None):
    """
    Convert a PNG map to PGM and YAML files for ROS2 Nav2.
    
    Args:
        png_path: Path to the PNG map file
        output_dir: Directory to save output files (defaults to same as input)
        resolution: Map resolution in meters/pixel
        origin: Origin coordinates as [x, y, theta] (defaults to [0, 0, 0])
    """
    # Load the PNG image
    img = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Error: Could not load image {png_path}")
        return False
    
    # For navigation maps, free space is typically represented by white (255)
    # and obstacles by black (0). If the image uses the opposite convention, invert it.
    # Uncomment the line below if needed:
    #img = 255 - img
    
    # Ensure proper thresholding for nav2
    # Values below 127 will be considered occupied (0), above as free (255)
    #_, img_binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    _, img_binary = cv2.threshold(img, 210, 255, cv2.THRESH_BINARY)
    
    # Set up file paths
    if output_dir is None:
        output_dir = os.path.dirname(png_path)
    
    base_name = os.path.splitext(os.path.basename(png_path))[0]
    pgm_path = os.path.join(output_dir, f"{base_name}.pgm")
    yaml_path = os.path.join(output_dir, f"{base_name}.yaml")
    
    # Write PGM file
    cv2.imwrite(pgm_path, img_binary)
    
    # Create YAML file
    if origin is None:
        origin = [0.0, 0.0, 0.0]
    
    yaml_data = {
        'image': f"{base_name}.pgm",
        'resolution': resolution,
        'origin': origin,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'mode': 'trinary'
    }
    
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
    
    print(f"Generated map files:")
    print(f"  PGM: {pgm_path}")
    print(f"  YAML: {yaml_path}")
    print(f"\nYou can use these files with Nav2 by loading them with:")
    print(f"  ros2 launch nav2_bringup tb3_simulation_launch.py map:={yaml_path}")
    
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 convert_map.py <png_file> [resolution] [output_dir]")
        sys.exit(1)
    
    png_file = sys.argv[1]
    
    #resolution = 0.05  # Default resolution (meters/pixel)
    resolution = None  # Default resolution (meters/pixel)
    if len(sys.argv) > 2:
        resolution = float(sys.argv[2])
    
    output_dir = None
    if len(sys.argv) > 3:
        output_dir = sys.argv[3]
        os.makedirs(output_dir, exist_ok=True)
    
    convert_png_to_pgm_yaml(png_file, output_dir, resolution)