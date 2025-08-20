import holoocean
import numpy as np
from pynput import keyboard
import cv2

pressed_keys = list()
force = 25

def on_press(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.append(key.char)
        pressed_keys = list(set(pressed_keys))

def on_release(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.remove(key.char)

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

def parse_keys(keys, val):
    command = np.zeros(8)
    if 'i' in keys:
        command[0:4] += val
    if 'k' in keys:
        command[0:4] -= val
    if 'j' in keys:
        command[[4,7]] += val
        command[[5,6]] -= val
    if 'l' in keys:
        command[[4,7]] -= val
        command[[5,6]] += val

    if 'w' in keys:
        command[4:8] += val
    if 's' in keys:
        command[4:8] -= val
    if 'a' in keys:
        command[[4,6]] += val
        command[[5,7]] -= val
    if 'd' in keys:
        command[[4,6]] -= val
        command[[5,7]] += val

    return command

with holoocean.make("Dam-Hovering") as env:
    while True:
        if 'q' in pressed_keys:
            break
        command = parse_keys(pressed_keys, force)

        #send to holoocean
        env.act("auv0", command)
        state = env.tick()
        #print(state["SemanticSegmentationCamera"].shape)
        if "RGBDCamera" in state:
           pixels = state["RGBDCamera"]
           #print(pixels.max())
           depth_data = pixels[:, :, 4]
           normalized_depth = cv2.normalize(depth_data, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
           grayscale_image = cv2.cvtColor(normalized_depth, cv2.COLOR_GRAY2BGR) 
           cv2.namedWindow("Camera Output")
           cv2.imshow("Camera Output", grayscale_image)
           cv2.waitKey(0)
           cv2.destroyAllWindows()