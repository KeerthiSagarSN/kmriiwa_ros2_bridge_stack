import numpy as np
from PIL import Image
import yaml

# Map dimensions (in pixels) matching your previous map
width = 738
height = 542

# Create empty map (255 = free space in PGM format)
empty_map = np.ones((height, width), dtype=np.uint8) * 255

# Save as PGM
image = Image.fromarray(empty_map)
image.save("bay_cs.pgm")

# Create YAML file with your specifications
yaml_content = {
    "image": "bay_cs.pgm",
    "mode": "trinary",
    "resolution": 0.05,  # 5cm per pixel, as specified
    "origin": [0.0, 0.0, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.25
}

with open("bay_cs.yaml", "w") as yaml_file:
    yaml.dump(yaml_content, yaml_file, default_flow_style=False)

print("Empty map created successfully with dimensions 738x542 @ 0.05 m/cell!")