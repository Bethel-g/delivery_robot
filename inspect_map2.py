import yaml
import cv2

with open('/home/betheln/ros2_ws/src/Robotics/delivery_robot2/delivery_robot/maps/office_map.yaml', 'r') as f:
    config = yaml.safe_load(f)

img = cv2.imread('/home/betheln/ros2_ws/src/Robotics/delivery_robot2/delivery_robot/maps/office_map.pgm', cv2.IMREAD_GRAYSCALE)
res = config['resolution']
orig = config['origin']
h, w = img.shape

def print_map(x, y):
    gx = int((x - orig[0]) / res)
    gy = int((y - orig[1]) / res)
    gy = h - gy - 1
    print(f"--- Map around {x}, {y} ---")
    window = img[gy-15:gy+15, gx-15:gx+15]
    for row in window:
        print("".join(['#' if p < 200 else '.' for p in row]))

print_map(7.5, 4.0)
print_map(7.5, 3.2)
