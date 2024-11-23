from djitellopy import Tello
import cv2, math, time

tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()

tello.takeoff()

# tello.go_xyz_speed(300, 200, 0, 50)
tello.curve_xyz_speed(100, 100, 0, 200, 0, 0, 50)

tello.land()
