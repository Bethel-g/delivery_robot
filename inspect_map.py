import yaml
import cv2
import numpy as np

with open('/home/betheln/ros2_ws/src/Robotics/delivery_robot2/delivery_robot/maps/office_map.yaml', 'r') as f:
    config = yaml.safe_load(f)

img = cv2.imread('/home/betheln/ros2_ws/src/Robotics/delivery_robot2/delivery_robot/maps/office_map.pgm', cv2.IMREAD_GRAYSCALE)
res = config['resolution']
orig = config['origin']

def map_to_grid(x, y):
    gx = int((x - orig[0]) / res)
    gy = int((y - orig[1]) / res)
    return gx, gy

gx, gy = map_to_grid(7.5, 2.0)
h, w = img.shape
gy = h - gy - 1 # Flip Y for image coords if needed? Or maybe just print a window.
print(f"Goal grid coords: {gx}, {gy} (img shape {w}x{h})")

# Print a 20x20 grid around the goal
window = img[gy-10:gy+10, gx-10:gx+10]
for row in window:
    print("".join(['#' if p < 200 else '.' for p in row]))

print("--- Room 3 (2.5, 5.3) ---")
gx, gy = map_to_grid(2.5, 5.3)
gy = h - gy - 1
window = img[gy-10:gy+10, gx-10:gx+10]
for row in window:
    print("".join(['#' if p < 200 else '.' for p in row]))

