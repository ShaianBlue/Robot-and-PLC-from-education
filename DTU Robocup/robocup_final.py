#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */

###########################################################################################
# This code is for final demonstration for Robocup!
# Add your challenge with state! 
# If you want to find with ctrl F with the below keyword!
# funcimage - image related function line [imageAnalysis(save), imagesaving(save)]
# funcdistance - moving forward with 1m or defined centimeter  [driveOneMeter(), driveuserMeter()]
# funcdegree - turning left(-) or right(+) pi or defined degree [driveTurnPi(), driveTurnDegree()]
# funcline - line following function. [ driveToLine() - triggered with distance sensor, driveMyLine() - without trigger]
# funcex - pseudo code for challenges. [ drive2Object(target_x, frame_width,distance,velocity=None),drive2Axes(), drive2Golfball2()]
# funcobject- object detection (ball) and move to the object [ball_info,pid_control,drive2Object]
# ball info is from shaian's code.

# if you want to start, touch right distance sensor after running python3 robocupmain.py 
# if you want to start from your challenge, python3 robocupmain.py -u 10 <-it's the number of state 

# Shaian and Tugba can check the #funcobject part for their challenges!

import sys
import os
add_path=os.path.expanduser("~/svn/robobot/mqtt_python/") 
sys.path.append(add_path)
#import threading
import time as t
#import select
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
# robot function
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service
from simu import imu
import math
# Self-made functions
#from Robot import robot
import findBall as fb
import ArUco as aruco
###############################################################################
# funcimage
def imageAnalysis(save):
  if cam.useCam:
    ok, img, imgTime = cam.getImage()
    if not ok: # size(img) == 0):
      if cam.imageFailCnt < 5:
        print("% Failed to get image.")
    else:
      h, w, ch = img.shape
      if not service.args.silent:
        # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
        pass
      edge.paint(img)
      if not gpio.onPi:
        cv.imshow('frame for analysis', img)
      if save:
        fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
        cv.imwrite(fn, img)
        if not service.args.silent:
          print(f"% Saved image {fn}")
      pass
    pass
  pass

def imagesaving(save): ##MEMO: image save function for image analysis
  # MEMO:
  if cam.useCam:
    ok, img, imgTime = cam.getImage()
    if not ok: # size(img) == 0):
      if cam.imageFailCnt < 5:
        print("% Failed to get image.")
    else:
      h, w, ch = img.shape
      if not service.args.silent:
        # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
        pass
      if not gpio.onPi:
        cv.imshow('frame for analysis', img)
      if save:
        fn = f"/home/local/svn/robobot/mqtt_python/final_image/image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg" # MEMO: check the image folder exist!
        cv.imwrite(fn, img)
        if not service.args.silent:
          print(f"% Saved image {fn}")
      pass
    pass
  pass

############################################################

stateTime = datetime.now()
def stateTimePassed():
  return (datetime.now() - stateTime).total_seconds()

############################################################

# funcdistance
def driveOneMeter():
  state = 0
  pose.tripBreset()
  print("% Driving 1m -------------------------")
  service.send(service.topicCmd + "T0/leds","16 0 100 0") # green
  while not ((service.stop) or gpio.stop()):
    if state == 0: # wait for start signal
      service.send("robobot/cmd/ti/rc","0.2 0.0") # (forward m/s, turn-rate rad/sec)
      state = 1
    elif state == 1:
      if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001:
        state = 99
    else:
      print(f"# drive 1m drove {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break;
    print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
    t.sleep(0.05)
  pass
  service.send(service.topicCmd + "T0/leds","16 0 0 0") # end
  print("% Driving 1m ------------------------- end")


def driveuserMeter(centimeter,velocity=None): # this function is for moving forward "input(centimeter)" cm. you can also give velocity. default velocity is 0.2m/s
  state = 0
  pose.tripBreset()
  print("% Driving user defined m -------------------------")
  service.send(service.topicCmd + "T0/leds","16 0 100 0") # green
  if velocity is None:  # defalult velocity
    velocity = 0.2
  while not ((service.stop) or gpio.stop()):
    if state == 0: # wait for start signal
      service.send("robobot/cmd/ti/rc",f"{velocity} 0.0") # (forward m/s, turn-rate rad/sec)
      state = 1
    elif state == 1:
      if pose.tripB > centimeter*0.01 or pose.tripBtimePassed() > 20: # we set a weight 1/1.42 to calibrate meter
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001:
        state = 99
    else:
      print(f"# drive {centimeter}cm drove {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break;
    print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
    t.sleep(0.05)
  pass
  service.send(service.topicCmd + "T0/leds","16 0 0 0") # end
  print("% Driving ------------------------- end")

############################################################################################################
#funcobject - You can change this prototyped code into improved ones. just check the structure and replace the name in state part

def ball_info(im_RGB,colors=[True, True, True, True, True]):
  #cv.imwrite("debug_image.jpg", im_RGB)
  ball = fb.findBall(im_RGB, colors)
  if ball is None:
    return None, None, None
  extraBall = [int(color) for color in colors]
  foundBall = fb.remove_extra_ball(ball, extraBall)
  data = fb.get_ball_info(im_RGB, foundBall[0])
  return data

def aruco_info(im_RGB, ID, rotate):
    InternMat = np.array([[2549.54 / 4, 0, 1639.73 / 4],
                          [0, 2552.31 / 4, 1197.41 / 4],
                          [0, 0, 1]])
    distortion = np.array([0.18, -0.31, 0, 0])
    arucoID, corner = aruco.detect_aruco(im_RGB)
    if arucoID is None:
      return None, None, None
    arucoVal = {}
    for idx, aID in enumerate(arucoID):
      if aID[0] in ID:
        arucoVal[f"{aID[0]}"] = corner[idx]
    if len(arucoVal.keys()) == 0:
      return None, None, None
    else:
      print(list(arucoVal.values()))
      arucoIdx = np.argmax([abs((corner[0][0][0] - corner[0][1][0]) / (corner[0][0][1] - corner[0][1][1])) for corner in list(arucoVal.values())])
      rotation, translation = aruco.orientation(arucoVal[str(list(arucoVal.keys())[arucoIdx])], distortion, InternMat)
      if rotate:
        _, angle, distance = aruco.get_camera_position(rotation, translation)
      else:
        angle, distance = aruco.get_position(translation, 0.02)
      return angle, distance, list(arucoVal.keys())[arucoIdx]

# PID controller variables
prev_error = 0.0  # Previous error for D term
integral = 0.0    # Integral term accumulator
Kp = 0.5          # Proportional gain
Ki = 0.01         # Integral gain
Kd = 0.1          # Derivative gain

def pid_control(error, prev_error, integral, dt):
    # Proportional term
    P = Kp * error
    # Integral term (with anti-windup)
    integral += error * dt
    I = Ki * integral
    # Derivative term
    D = Kd * (error - prev_error) / dt
    # PID output
    output = P + I + D
    return output, integral
    
def drive2ObjectPID(distance=None, velocity=None, object=None): #If you want to utilize this code, you have to adjust PID gain and keep it safe
    state_i = 0
    frame_width = 3280  # default width
    target_center = frame_width / 2  # target is in the center of the frame
    size = 100  # default size
    pose.tripBreset()
    print("% Adjusting position to center -------------------------")
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # Green LED
    
    if velocity is None:
        velocity = 0.1  # Default lateral movement speed
    if distance is None:
        distance = 72000  # Default size threshold for stopping
    if object is None: #Default object is golfball
      object=0 # object 0: golfball 1: flatball(sorting challenge) 2. hole 3.aruco for sorting 4. aruco for goal 

    prev_error = 0.0  # Initialize previous error
    integral = 0.0    # Initialize integral term
    tolerance=10 #have to be adjusted
    while not ((service.stop) or gpio.stop()):
        if cam.useCam:
            ok, img, imgTime = cam.getImage()
            if not ok:
                if cam.imageFailCnt < 5:
                    print("% Failed to get image.")
            else:
                if object==0: #golfball
                  frame_width, x_center, size = ball_info(img, colors=[True, False, False, False, False])
                elif object ==1: # flatball 
                  frame_width, x_center, size = ball_info(img, colors=[False, True, False, True, False])
                elif object==2: #hole
                  frame_width, x_center, size = ball_info(img, colors=[False, False, False, False, True])
                elif object==3: #sort aruco
                  #frame_width, x_center, size = aruco_sorting_info(img, colors=[True, False, True])
                  pass
                elif object==4: #goal aruco
                  #frame_width, x_center, size = aruco_goal_info(img, colors=[True, False, True]
                  pass
            if frame_width is not None and x_center is not None and size is not None :
              error = x_center - target_center  # Error: difference from the center
              target_center = frame_width / 2
              if pose.tripBtimePassed()>5:
                service.send("robobot/cmd/ti/rc", f"0 0")
              if size > distance:
                  service.send("robobot/cmd/ti/rc", "0.0 0.0")  # Stop the robot if the object is large enough
                  print("Position Adjustment -------------------- end")
                  break
              # Apply PID control to adjust movement
              if state_i == 0:  # Start adjustment
                  dt = 0.02  # Time step (change as needed)
                  pid_output, integral = pid_control(error, prev_error, integral, dt)
                  
                  if pid_output > 0:
                      # Object is to the right → move left
                      if (-velocity + pid_output)>-0.7:
                        service.send("robobot/cmd/ti/rc", f"0.15 {-velocity + pid_output}")
                  else:
                      # Object is to the left → move right
                      if (velocity + pid_output)<0.7:
                        service.send("robobot/cmd/ti/rc", f"0.15 {velocity + pid_output}")
                  
                  prev_error = error  # Update previous error for the next iteration
                  state_i = 1
            
              elif state_i == 1:
                  if abs(error) < tolerance or pose.tripBtimePassed() > 20:
                      service.send("robobot/cmd/ti/rc", f"{velocity} 0.0")  # Object is centered! Going straight
                      
            
            else:
                service.send("robobot/cmd/ti/rc", "0.0 0.0")  # Stop movement

            print(f"# Move to Object in time: {pose.tripBtimePassed():.3f} sec")
            t.sleep(0.02)

    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # Turn off LED
    print("% Position Adjustment ------------------------- end")

def drive2Object(distance=None, velocity=None, object=None): #driv2Object(distance=6000,velocity=0.15,object=1)
    service.send(service.topicCmd + "T0/servo","1 -800 200")
    startTime = datetime.now()
    frame_width = 820 # default width
    target_center = frame_width / 2  # target is in the center of the frame
    size = 100  # default size
    pose.tripBreset()
    print("% Adjusting position to center -------------------------")
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # Green LED
    edge.lineControl(0,0)
    if velocity is None:
        velocity = 0.1  # Default lateral movement speed
    if distance is None:
        distance = 5000  # Default size threshold for stopping
    if object is None: #Default object is golfball
      object=0 # object 0: golfball 1: flatball(sorting challenge) 2. hole 3.aruco for sorting 4. aruco for goal 

    prev_error = 0.0  # Initialize previous error
    integral = 0.0    # Initialize integral term
    tolerance=10 #have to be adjusted
    size_old = None
    ball_fail = 0
    K_gain=0.05 # Adjust this gain to change turnlate depend on the error
    while not ((service.stop) or gpio.stop()):
        if cam.useCam:
            ok, img, imgTime = cam.getImage()
            if not ok:
                if cam.imageFailCnt < 5:
                    print("% Failed to get image.")
            else:
                #imagesaving(True)
                if object==0: #golfball
                  frame_width, x_center, size = ball_info(img, colors=[True, False, False, False, False])
                elif object ==1: # hole 
                  frame_width, x_center, size = ball_info(img, colors=[False, False, False, False, True])
                elif object==2: #red ball
                  frame_width, x_center, size = ball_info(img, colors=[False, True, False, False, False])
                elif object==3: #blue ball
                  frame_width, x_center, size = ball_info(img, colors=[False, False, False, True, False])
                if frame_width is not None and x_center is not None and size is not None:
                  if size_old is None:
                    size_old = size
                    gain = size
                  elif size_old > size and size_old > 3 / 4 * distance:
                    size = size_old + gain
                  target_center = frame_width / 2
                  error = x_center - target_center  # Error: difference from the center
                  gain = abs(size - size_old)
                  size_old = size
                  ball_fail = 0
                  if size > distance:
                      service.send("robobot/cmd/ti/rc", "0.0 0.0")  # Stop the robot if the object is large enough
                      print("Position Adjustment -------------------- end")
                      service.send(service.topicCmd + "T0/servo","1 210 200") #remember servo range is -800(up)~800(down) and 0 is maximum velocity
                      t.sleep(1)
                      MieDriveUserMeter(0.02, 0.1)
                      MieDriveUserMeter(0.04, -0.1)
                      MieDriveUserMeter(0.02, 0.1)
                      break
                  else:
                    beginTime2 = datetime.now()
                    if (beginTime2 - startTime).total_seconds() > 40:
                      print("Took too long")
                      break
                    while(((datetime.now() - beginTime2).total_seconds()) <= 0.2):
                      if error > tolerance:
                          # Object is on the rightside → turn right
                          service.send("robobot/cmd/ti/rc", f"{velocity} -0.2")
                          #if you want to do feedback control, you can change it 
                          #service.send("robobot/cmd/ti/rc", f"{velocity} {-0.3*error*(1+K_gain)}") 
                      elif error<-tolerance:
                          # Object is on the leftside → turn left
                          service.send("robobot/cmd/ti/rc", f"{velocity} 0.2")
                          #service.send("robobot/cmd/ti/rc", f"{velocity} {0.3*error*(1+K_gain)}") 
                      else : #error is in the tolerance. centered enough.
                          service.send("robobot/cmd/ti/rc", f"0.2 0.0")  # Object is centered! Going straight
                    service.send("robobot/cmd/ti/rc", f"0 0")
                    t.sleep(0.1)
                else:
                  service.send("robobot/cmd/ti/rc", f"-0.05 0.0")
                  ball_fail += 1
                  if ball_fail == 5:
                    print("Ball isn't visible")
                    return

    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # Turn off LED
    print("% Position Adjustment ------------------------- end")

def drive2Aruco(velocity=None, marker=None, rotate=True):
  pose.tripBreset()
  print("% Adjusting position to center -------------------------")
  service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # Green LED
  if velocity is None:
    velocity = 0.15  # Default lateral movement speed
  if marker is None:
    marker=0 # object 0: ID 12 or 13 1: ID 14 or 15 2: ID 25

  if not ((service.stop) or gpio.stop()):
    if cam.useCam:
      ok, img, _ = cam.getImage()
      if not ok:
        if cam.imageFailCnt < 5:
          print("% Failed to get image.")
      else:
        markerVal = [12, 13] if marker == 0 else [14, 15] if marker == 1 else [25] if marker == 2 else [0]
        angle, distance, chosenID = aruco_info(img, markerVal, rotate)  
        if angle is not None and distance is not None:
          if rotate:
            driveTurnDegree(-angle, 0.01)
          else:
            driveTurnDegree(angle, 0.01)
            if marker == 2:
              MieDriveUserMeter(distance, velocity)
            else:
              MieDriveUserMeter(distance - 0.4, velocity)
        else:
          driveTurnDegree(-60, 0.01)
          MieDriveUserMeter(0.1, -0.2)
          chosenID = drive2Aruco(marker=marker, rotate=rotate)

    service.send("robobot/cmd/ti/rc", f"0 0")
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # Turn off LED
    print("% Position Adjustment ------------------------- end")
    return chosenID

def distance2Aruco(velocity=None, marker=None):
  pose.tripBreset()
  print("% Adjusting position to center -------------------------")
  service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # Green LED
  if velocity is None:
    velocity = 0.15  # Default lateral movement speed
  if marker is None:
    marker=0 # object 0: ID 12 or 13 1: ID 14 or 15 2: ID 25

  if not ((service.stop) or gpio.stop()):
    if cam.useCam:
      ok, img, _ = cam.getImage()
      if not ok:
        if cam.imageFailCnt < 5:
          print("% Failed to get image.")
      else:
        markerVal = [12, 13] if marker == 0 else [14, 15] if marker == 1 else [25] if marker == 2 else [0]
        angle, distance, _ = aruco_info(img, markerVal, True)
        if angle is not None and distance is not None:
          driveTurnDegree(-angle, 0.01)
          return distance
        else:
          driveTurnDegree(-60, 0.01)
          distance = distance2Aruco(marker=marker)
          return distance

    service.send("robobot/cmd/ti/rc", f"0 0")
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # Turn off LED
    print("% Position Adjustment ------------------------- end")
    return distance

def drive2ball(velocity = 0.15, ball = None):
  pose.tripBreset()
  print("% Adjusting position to center -------------------------")
  service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # Green LED
  if ball is None:
    ball = 0 # 0 for golf, 1 for red, 2 for blue

  InternMat = np.array([[2549.54 / 4, 0, 1639.73 / 4],
                        [0, 2552.31 / 4, 1197.41 / 4],
                        [0, 0, 1]])
  distortion = np.array([0.18, -0.31, 0, 0])
  tilt = 11 * np.pi / 180
  ExternMat = np.dot(np.dot(np.array([[1, 0, 0, 0],
                                      [0, np.cos(-np.pi / 2), -np.sin(-np.pi / 2), 0],
                                      [0, np.sin(-np.pi / 2), np.cos(-np.pi / 2), 0],
                                      [0, 0, 0, 1]]),
                            np.array([[np.cos(np.pi / 2), 0, np.sin(np.pi / 2), 0],
                                      [0, 1, 0, 0],
                                      [-np.sin(np.pi / 2), 0, np.cos(np.pi / 2), 0],
                                      [0, 0, 0, 1]])),
                     np.array([[1, 0, 0, -0.0025],
                               [0, np.cos(tilt), -np.sin(tilt), 0],
                               [0, np.sin(tilt), np.cos(tilt), -0.19],
                               [0, 0, 0, 1]]))
  
  while not ((service.stop) or gpio.stop()):
    if cam.useCam:
      ok, img, _ = cam.getImage()
      if not ok:
        if cam.imageFailCnt < 5:
          print("% Failed to get image.")
      else:
        chosenColor = [True, False, False, False, False] if ball == 0 else [False, True, False, False, False] if ball == 1 else [False, False, False, True, False] if ball == 2 else [True, True, True, True, True]
        extraBall = [int(color) for color in chosenColor]
        ballObject = fb.findBall(img, chosenColor)
        if ballObject is None:
          MieDriveUserMeter(0.05, -velocity)
          drive2ball(velocity, ball)
        else:
          foundBall = fb.remove_extra_ball(ballObject, extraBall)
          ball3D = fb.place3D(InternMat, ExternMat, distortion, foundBall)
          angle, distance = fb.next_action(ball3D, 0.25, 0.02)
          driveTurnDegree(angle, 0.01)
          if distance > 0.5:
            MieDriveUserMeter(0.5, velocity)
          else:
            MieDriveUserMeter(distance, velocity)
            break

##################################################################################################
# funcline

def driveToLine():
  state = 0
  pose.tripBreset()
  dist_to_line = 0
  print("% Driving to line ---------------------- right ir start ---")
  service.send(service.topicCmd + "T0/leds","16 0 100 0") # green
  while not ((service.stop) or gpio.stop()):
    if state == 0: # forward towards line
      if ir.ir[0] < 0.2: # it works as a trigger to start line following with the distance sensor rightside of robot! 
        service.send("robobot/cmd/ti/rc","0.4 0.0") # (forward m/s, turn-rate rad/sec)
        service.send("robobot/cmd/T0/lognow","3") # (start Teensy log)
        state = 1
    elif state == 1:
      if pose.tripB > 1.0 or pose.tripBtimePassed() > 10:
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      if edge.lineValidCnt > 4:
        # start follow line
        edge.lineControl(0.4, 0)
        dist_to_line = pose.tripB
        pose.tripBreset()
        print(" to state 10")
        state = 10
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001:
        print(" to state 99")
        state = 99
    elif state == 10:
      if edge.lineValidCnt < 2:
        edge.lineControl(0, 0)
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        print(" to state 2")
        state = 2
    else:
      print(f"# drive to line {dist_to_line:.3f}m, then along line {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break
    # print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds, line valid cnt = {edge.lineValidCnt}")
    t.sleep(0.01)
  pass
  service.send(service.topicCmd + "T0/leds","16 0 0 0") # end
  print("% Driving to line ------------------------- end")



#################################################################################################################
# funcdegree
def driveTurnPi():
  state = 0
  pose.tripBreset()
  print("% Driving a Pi turn -------------------------")
  service.send(service.topicCmd + "T0/leds","16 0 100 0") # green
  while not ((service.stop) or gpio.stop()):
    if state == 0: # wait for start signal
      service.send("robobot/cmd/ti/rc","0.2 0.5") # (forward m/s, turn-rate rad/sec)
      state = 1
    elif state == 1:
      if pose.tripBh > 3.14 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        state = 99
    else:
      print(f"# drive turned {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break;
    print(f"# turn {state}, now {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds")
    t.sleep(0.05)
  pass
  service.send(service.topicCmd + "T0/leds","16 0 0 0") # end
  print("% Driving a Pi turn ------------------------- end")

def driveTurnDegree(deg,velocity=None): ##MEMO: This function is for turn right(positive) / left(negative) in degree.
  state_i = 0  
  if velocity is None:
    velocity = 0.2  # Default lateral movement speed
  pose.tripBreset()
  print("% Driving a Pi turn -------------------------")
  service.send(service.topicCmd + "T0/leds","16 0 100 0") # green
  while not ((service.stop) or gpio.stop()): 
    if state_i == 0: # wait for start signal
      deg2rad=deg*3.14/180 # MEMO: the degree have a weight, 0.75, for calibration.
      if deg2rad>0:
        service.send("robobot/cmd/ti/rc",f"{velocity} -0.5") # (forward m/s, turn-rate rad/sec)
      else:
        service.send("robobot/cmd/ti/rc",f"{velocity} 0.5") # (forward m/s, turn-rate rad/sec)
      state_i = 1
    elif state_i == 1:
      
      if abs(pose.tripBh) > abs(deg2rad) :
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state_i = 2
      pass
    elif state_i == 2:
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        state_i = 99
    else:
      print(f"# drive turned {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break;
    print(f"# turn {state_i}, now {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds")
    t.sleep(0.05)
  pass
  service.send(service.topicCmd + "T0/leds","16 0 0 0") # end
  print("% Driving a Pi turn ------------------------- end")
##################################################################################################
# Mie challenge
def MieDriveUserMeter(meter, velocity):

  pose.tripBreset()
  #Begin moving at velocity 
  position = pose.tripB

  service.send("robobot/cmd/ti/rc",f"{velocity} 0.0") # (forward m/s, turn-rate rad/sec)
  print(f"meter: {meter}, position: {position}, velocity: {velocity}")
  if velocity > 0:
    while position < meter and (not(gpio.stop())): #keep moving untill 
      t.sleep(0.05)
      position = pose.tripB
  else:
    meter = -meter
    while position > meter and  (not(gpio.stop())): #keep moving untill 
      t.sleep(0.05)
      position = pose.tripB

  service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)

def MieMeasureTimeToPassAxe():
  stateMeasureTime = 0

  while(1):
    distance = ir.ir[1]
    print(f"distance is: {distance}")
    if stateMeasureTime == 0: 
      if(distance <= 0.2): #If the distance is less than 0.1 meter
        print("distance less than 0.1 m")
        #Axe is closed
        stateMeasureTime = 1

    elif stateMeasureTime == 1: 
      if distance > 0.2: #If flag has been set and the distance is greater than 0.1 meter
        #Axe has just opened 
        time_at_axe_opening = datetime.now() #begin timer
        while(distance > 0.2): #wait for the axe to close. 
          t.sleep(0.01)
          distance = ir.ir[1]
        #Axe has just closed  
        return(datetime.now() - time_at_axe_opening).total_seconds()

def MieDriveUserMeterWithDeacceleration(meter, velocity):
  pose.tripBreset()

  #Begin moving at velocity 
  position = pose.tripB
  service.send("robobot/cmd/ti/rc",f"{velocity} 0.0") # (forward m/s, turn-rate rad/sec)
  while position < meter: #keep moving untill 
    t.sleep(0.05)
    position = pose.tripB


  #deaccelerate
  service.send("robobot/cmd/ti/rc","0.5 0.0") # (forward m/s, turn-rate rad/sec)
  pose.tripBreset()
  position = pose.tripB
  while position < 0.3: #deaccelerate untill
    t.sleep(0.05)
    position = pose.tripB

  #stop completely after deaccelerating
  service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)

def MiePassAxe():
  pass_axe_state = 0
  while(1):
    distance = ir.ir[1]
    #print(f"distance is: {distance}")

    if pass_axe_state == 0: #Wait for axe to be open
      if distance > 0.2: #Axe is open 
        print(f"Axe is open")
        pass_axe_state = 1 
    
    elif pass_axe_state == 1: #wait for axe to be closed
      if distance <= 0.2: #Axe has just closed. 
        print("Axe has closed")
        pass_axe_state = 2 
    
    elif pass_axe_state == 2: 
      if distance > 0.2: #Axe has just become open
        #Begin driving straight 
        #MieDriveUserMeterWithDeacceleration(0.4, 1.0)
        edge.lineControl(1.0, 0.0) 
        t.sleep(0.5)
        break 
      
    t.sleep(0.1)
#######################################################
# Jianing function
# === Utility Functions ===

def get_pitch_from_acc(acc):
    ax = acc[0]
    az = acc[2]

    pitch_rad = math.atan2(ax, az)
    pitch_deg = pitch_rad * 180.0 / math.pi
    return pitch_deg

def robotPitch():
    ax, ay, az = imu.acc[0], imu.acc[1], imu.acc[2]
    return get_pitch_from_acc([ax, ay, az])

from collections import deque
WINDOW_SIZE = 3
pitch_buffer = deque(maxlen=WINDOW_SIZE)

def get_filtered_pitch(): #abs(pitch)<5 : ground # pitch>5 : slope #pitch<-13 is stairs
    current_pitch = robotPitch()  
    pitch_buffer.append(current_pitch)
    return sum(pitch_buffer) / len(pitch_buffer)


def loop():
  userdistance=0
  userdegree=0
  uservelocity=0
  ##### FOR INITIALIZATION #############
  from ulog import flog
  bbias=0
  images = 0
  ledon = True
  tripTime = datetime.now()
  oldstate = -1
  oldmystate=-1
  nowdeg=0
  service.send(service.topicCmd + "T0/leds","16 30 30 0") # LED 16: yellow - waiting
  state=0
  flog.write(state)
  ##### FOR Arg PROCESSING meter, turn  #############
  if service.args.meter:
    state = 101 # run 1m
  elif service.args.pi:
    state = 102 # turn 180 degree
  elif service.args.usestate is not None and service.args.usestate > 0:
    state = service.args.usestate
  elif service.args.usermeter is not None and service.args.usermeter > 0:
    userdistance = service.args.usermeter
    state = 250 #defined state for custom meter
  elif service.args.userdegree is not None and service.args.userdegree> 0:
    userdegree= service.args.userdegree
    state = 260 #defined state for custom degree
  if service.args.uservelocity is not None and service.args.uservelocity> 0:
    uservelocity= service.args.uservelocity*0.1 # if you want to set velocity into 0.2, you have to type 2. 
    
  print(f"% Using state {state}")
  # elif not service.args.now:
  #   print("% Ready, press start button")
  # main state machine
  edge.lineControl(0, 0) # make sure line control is off

  ##### process block ! ! ####################################################

  while not ((service.stop)):
    print(f"state: {state}")
    if state==0: #start signal
      if ir.ir[0]<0.25 or gpio.start():
        state=1
    elif state == 1: # wait for start signal
      start = True # gpio.start() or service.args.now
      if start:
        flog.write(state)
        print("% Starting")
        service.send(service.topicCmd + "T0/leds","16 0 0 30") # blue: running
        service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        service.send(service.topicCmd + "T0/servo","1 -800 200") #[servo number / position / velocity]remember servo range is -800(up)~800(down) and 0 is maximum velocity
        # follow line (at 0.25cm/s)
        edge.lineControl(0.3, 0.0) # m/s and position on line -2.0..2.0 #MEMO: I will test the speed and test the line sensor data per velocity
        print(f"time bf reset: {pose.tripBtimePassed()}")
        pose.tripBreset()
        state = 2
    elif state==2:
      print(edge.average)
      print(f"time: {pose.tripBtimePassed()}")
      if edge.average>=310 and pose.tripBtimePassed() > 7:
        edge.lineControl(0,0)
        driveTurnDegree(30,0.05)
        edge.lineControl(0.4,0)
        print("avoid crossingline")
        state=4
      elif pose.tripBtimePassed() > 15:
        edge.lineControl(0.4,0)
        state=4
    elif state == 4: # Jianing- slope Detection . Allocated state 10~19
        print(f"time: {pose.tripBtimePassed()}")
        slope=get_filtered_pitch()
        if slope>10 and pose.tripBtimePassed() > 14: 
          edge.lineControl(0.5,0)
          pose.tripBreset()
          state=5
    elif state==5:
      print(f"time: {pose.tripBtimePassed()}")
      if pose.tripB >0.8:
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        edge.lineControl(0.0, 0.0)
        t.sleep(0.1)
        edge.lineControl(0.13, 0.0)
        state=6
    elif state ==6: 
      print(f"edge: edge.average")
      if edge.average >400 : 
        print(f"edge: edge.average")
        edge.lineControl(0,0) # stop following line
        pose.tripBreset()
        service.send(service.topicCmd + "ti/rc","-0.1 0") #go back
        state=7
    elif state==7:
      if pose.tripBtimePassed() > 0.5:
        edge.lineControl(0,0)
        service.send(service.topicCmd + "ti/rc","0.1 0.9") # turn left to follow the line
        pose.tripBreset()
        state=8
    elif state==8:
      if abs(pose.tripBh) > abs(3.14*0.3) :
        pose.tripBreset()
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        edge.lineControl(0.1, 0.0)
        state=10
    elif state==10: #Tugba challenge - start to drive to the ball on the seeasaw state allocation : 10~
      service.send(service.topicCmd + "T0/servo","1 -800 200") #[servo number / position / velocity]remember servo range is -800(up)~800(down) and 0 is maximum velocity
      t.sleep(4)
      edge.lineControl(0,0)
      drive2Object(distance = 6850, object = 0) # adjust velocity and distance ! #
      edge.lineControl(0,0)
      service.send(service.topicCmd + "ti/rc","0.1 0.0")
      t.sleep(0.01)
      pose.tripBreset()
      service.send(service.topicCmd + "ti/rc","0.0 0.0")
      edge.lineControl(0.1,0)
      print(f"time: {pose.tripBtimePassed()}")
      service.send(service.topicCmd + "T0/servo","1 1200 200")
      state=1001
    elif state==1001:
      slope=robotPitch()
      if abs(slope) > 10:
        state = 1002
    elif state==1002:
      service.send(service.topicCmd + "T0/servo","1 1200 200")
      pose.tripBreset()
      edge.lineControl(0.15,0)
      print(edge.average)
      state=1003
    elif state==1003:
      slope=robotPitch()
      if slope>-7 and slope<7:
        print("I arrived on the ground!")
        pose.tripBreset()
        state=1004
    elif state==1004:
      if pose.tripB>0.8:
        edge.lineControl(0.25,0)
        state=1005
    elif state==1005:
      print(f"1005 line:{edge.average} and {pose.tripB}")
      if edge.average<100 and pose.tripB>0.9:
        print(f"1005 line after:{edge.average} and {pose.tripB}")
        edge.lineControl(0,0)
        bbias=pose.tripB-1.35
        nowdeg=pose.tripBh*180/3.14
        MieDriveUserMeter(bbias, -0.2)
        driveTurnDegree(nowdeg, velocity=0.001)
        driveTurnDegree(-90, velocity=0.001)
        pose.tripBreset()
        edge.lineControl(0.1,0)
        state = 1006
    elif state == 1006:
      if pose.tripB > 0.3:
        edge.lineControl(0,0)
        driveTurnDegree(90, velocity=0.001)
        service.send(service.topicCmd + "ti/rc","0.2 0.0") #0.15
        state = 1007
    elif state == 1007:
      if ir.ir[1] > 1:
        service.send(service.topicCmd + "ti/rc","0.2 0.0")
      elif ir.ir[1] < 1 and ir.ir[1] > 0.7:
        service.send(service.topicCmd + "ti/rc","0.15 0.0")
      elif ir.ir[1] < 0.7:
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        driveTurnDegree(95, velocity=0.001)
        service.send(service.topicCmd + "ti/rc","0.1 0.0") 
        #MieDriveUserMeter(bbias, -0.2)
        #print(f"tilted : {nowdeg}")
        #driveTurnDegree(nowdeg,velocity=0.001)
        #print(edge.average)
        #t.sleep(0.05)
        #bias=pose.tripBh
        #print(f"bias: {bias} and degree: {bias*180/3.14}")
        #driveTurnDegree(-20,0.05) #turn left
        #t.sleep(0.05)
        #MieDriveUserMeter(1.1, 0.3)
        #pose.tripBreset()
        #driveTurnDegree(70,0.05)
        #MieDriveUserMeter(0.7, 0.3)
        #edge.lineControl(0.15,0)
        #pose.tripBreset()
        state=1008
    elif state==1008:
      if edge.lineValidCnt > 2:
        pose.tripBreset()
        edge.lineControl(0.2,0)
        state=5050 #1009
    elif state==5050:
      print(edge.average)
      if edge.average>600:
        print(edge.average)
        pose.tripBreset()
        edge.lineControl(0,0)
        state=5051
    elif state==5051:
      distance = distance2Aruco(marker=2)
      if distance < 1.1:
        MieDriveUserMeter(abs(distance - 1.1), -0.1)
      else:
        MieDriveUserMeter(abs(distance - 1.1), 0.1)
      driveTurnDegree(90, 0.01)
      MieDriveUserMeter(0.2, 0.1)
      pose.tripBreset()
      edge.lineControl(0.15,0)
      state=1010

    elif state==1010: # go up the slope 
      if edge.lineValidCnt > 2:
        edge.lineControl(0.2,0) #0.15
        pose.tripBreset()
        state=20
    elif state==20: #From 20~ it will start to go up the slope to put the ball into the hole.
      slope=robotPitch()
      if pose.tripBtimePassed() > 5 and slope>10 :
        service.send(service.topicCmd + "T0/servo","1 1200 200")
        edge.lineControl(0.25,0)
        pose.tripBreset()
        state=21
    elif state==21:
      slope=get_filtered_pitch()
      print(slope)
      if abs(slope) < 8 and pose.tripBtimePassed() > 2: #was 5
        print(f'time: {pose.tripBtimePassed()}')
        pose.tripBreset()
        edge.lineControl(0.1,0)
        state=22
    elif state==22:
      print(f'no crossing line sensor: {edge.average}')
      if edge.average > 400:
      #if edge.crossingLineCnt > 1:
        print('start go back')
        edge.lineControl(0.0,0)
        service.send("robobot/cmd/ti/rc","-0.2 0.5")
        pose.tripBreset()
        state= 23
    elif state==1999:
      print(f'no crossing line sensor: {edge.average}')
      if edge.crossingLineCnt > 1 :
        print('start go back')

        edge.lineControl(0.0,0)
        nowdeg=pose.tripBh*180/3.14
        state=2000

    elif state==2000:
      print(f'next: {nowdeg}')
      MieDriveUserMeter(0.1, -0.2)
      driveTurnDegree(nowdeg,velocity=0.01)
      driveTurnDegree(-60,velocity=0.01)
      MieDriveUserMeter(0.8, 0.2)
      break
      driveTurnDegree(-90,velocity=0.02)
      MieDriveUserMeter(0.16, 0.1)
      driveTurnDegree(-30,velocity=0.02)
      driveTurnDegree(-40,velocity=0.02)
      driveTurnDegree(30,velocity=0.02)
      driveTurnDegree(-30,velocity=0.02)
      driveTurnDegree(200,velocity=0.02)
      MieDriveUserMeter(0.30, 0.1)
      state=30 #have to be discarded if not 
      
    elif state==23:
      if pose.tripBtimePassed() > 1.4 and abs(pose.tripBh) > 3.14*0.55:
          MieDriveUserMeter(0.23, 0.1)
          driveTurnDegree(-35,velocity=0.02)
          driveTurnDegree(-40,velocity=0.02)
          driveTurnDegree(30,velocity=0.02)
          driveTurnDegree(-30,velocity=0.02)
          driveTurnDegree(30,velocity=0.02)
          MieDriveUserMeter(0.10, -0.1)
          driveTurnDegree(180,velocity=0.02)
          MieDriveUserMeter(0.20, 0.1)
          service.send(service.topicCmd + "T0/servo","1 -800 200")
          state=30
    elif state == 24: # for 2nd golf ball! if not, go directly 30
      MieDriveUserMeter(0.1, -0.1)
      service.send(service.topicCmd + "T0/servo","1 -800 200")
      driveTurnDegree(180, 0.01)
      drive2Object(distance=6500, object=0)
      state = 25
    elif state == 25:
      driveTurnDegree(-10, 0.01)
      MieDriveUserMeter(0.5, -0.1)
      driveTurnDegree(-90, 0.01)
      service.send(service.topicCmd + "ti/rc","0.1 0")
      state = 26
    elif state == 26:
      if edge.lineValidCnt > 2:
        service.send(service.topicCmd + "ti/rc","0 0")
        edge.lineControl(0.1, 0)
        state = 27
    elif state == 27:
      if edge.average> 420:
        edge.lineControl(0, 0)
        service.send("robobot/cmd/ti/rc","-0.2 0.5")
        pose.tripBreset()
        state = 28
    elif state == 28:
      if pose.tripBtimePassed() > 1 and abs(pose.tripBh) > 3.14*0.6:
          service.send("robobot/cmd/ti/rc","0 0")
          MieDriveUserMeter(0.16, 0.1)
          driveTurnDegree(-30,velocity=0.02)
          driveTurnDegree(-40,velocity=0.02)
          driveTurnDegree(30,velocity=0.02)
          driveTurnDegree(-30,velocity=0.02)
          service.send(service.topicCmd + "T0/servo","1 -800 200")
          MieDriveUserMeter(0.10, -0.1)
          driveTurnDegree(200,velocity=0.02)
          MieDriveUserMeter(0.20, 0.1)
          state = 30
    elif state==30 : #If you want to add second golf ball challenge, use state number from 24 to 39 and connect with state 23. 
      service.send(service.topicCmd + "T0/servo","1 -550 200") #uplift the gripper and start to go down the stairs
      service.send("robobot/cmd/ti/rc","0 0")
      edge.lineControl(0.1,0)
      pose.tripBreset()
      staircnt=0
      stairtime=datetime.now() 
      slope=robotPitch()
      if slope<-15:
          service.send(service.topicCmd + "T0/servo","1 -800 200")
          state=31
      if slope<-45:
          service.send(service.topicCmd + "T0/servo","1 -800 200")
    elif state==31:
      slope=robotPitch()
      print(slope)
      if slope>-5 and slope<5:
        edge.lineControl(0,0)
        t.sleep(0.05)
        MieDriveUserMeter(0.05, -0.2)
        edge.lineControl(0.1,0)
        print(f"stair: {staircnt}")
        print(f"distance: {ir.ir[1]}")
        if staircnt>=5 or ((datetime.now() - stairtime).total_seconds())>20:
          pose.tripBreset()
          state=41
        staircnt=staircnt+1
        state=32
    elif state==32:
      slope=robotPitch()
      print(f"distance: {ir.ir[1]}")
      if slope<-10 and staircnt<5:
          service.send(service.topicCmd + "T0/servo","1 -800 200")
          state=31
      if slope<-60 :
          service.send(service.topicCmd + "T0/servo","1 -800 200")

      if staircnt>=5:
        pose.tripBreset()
        state=41
      if ((datetime.now() - stairtime).total_seconds())>20 : # just go next step.
        pose.tripBreset()
        state=41
    elif state==41:
      print(f"edge: {edge.average}")
      if pose.tripB > 0.7 : # it have to start lift up gripper again
          state=42
    elif state==42: # After stairs
      edge.lineControl(0.1,0)
      print(edge.average)
      if edge.average>400:
        edge.lineControl(0,0)
        driveTurnDegree(-90, 0.01)
        edge.lineControl(0.1,0)
        pose.tripBreset()
        state=43
    elif state==43: # After 90 degree from stairs
      if pose.tripBtimePassed() > 1.5:
        edge.lineControl(0,0)
        pose.tripBreset()
        service.send(service.topicCmd + "ti/rc","0.4 0.0")
        state=44
    elif state==44: # Stopping from running away
      if pose.tripB > 0.95:
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        service.send(service.topicCmd + "T0/servo","1 -200 200")
        edge.lineControl(0.15,0)
        pose.tripBreset()
        state=45
    elif state==45: # Find crossing line at basket
      print(edge.average)
      edge.lineControl(0.15,0)
      if edge.average>300:
        edge.lineControl(0,0)
        driveTurnDegree(80, 0.01)
        edge.lineControl(0.1,0)
        pose.tripBreset()
        state=46
    elif state==46: # Push the basketball and move back
      print(pose.tripBtimePassed())
      if pose.tripBtimePassed() > 0.5:
        edge.lineControl(0,0)
        service.send(service.topicCmd + "T0/servo","1 -200 200")
        t.sleep(0.05)
        MieDriveUserMeter(0.15, 0.1)
        MieDriveUserMeter(0.2, -0.1)
        driveTurnDegree(-110, 0.01)
        edge.lineControl(0.15,0)
        service.send(service.topicCmd + "T0/servo","1 -800 200")
        state=56
    elif state == 56: # moving into the axes
      service.send(service.topicCmd + "ti/rc","0 0")
      edge.lineControl(0, 0.0)
      pose.tripBreset()
      edge.lineControl(0.1, 0.0) # m/s and position on line -2.0..2.0 #MEMO: I will test the speed and test the line sensor data per velocity
      state = 57
    elif state == 57: 
      if edge.crossingLineCnt >4:#increase to make the linecontrol more sensitive. 
        edge.lineControl(0,0) # stop following line
        pose.tripBreset()
        service.send(service.topicCmd + "ti/rc","-0.1 0")
        state=58
    elif state == 58:
      if pose.tripBtimePassed() > 0.5:
        service.send(service.topicCmd + "ti/rc","0.1 -0.9") # turn right to follow the line
        pose.tripBreset()
        state=59
    elif state == 59:
      print(pose.tripBh)
      if abs(pose.tripBh) > abs(3.14*0.3) : #turn 
        pose.tripBreset()
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        edge.lineControl(0.05, 0.0)
        state = 60
    elif state==60: #moving to the axeschallenge Mia : state 60~70
      edge.lineControl(0.05, 0.0)
      beginTime = datetime.now()
      smallest_distance = ir.ir[1]
      MieState = 1 
      if MieState == 1: #approach axe
        while(smallest_distance > 0.2): #approach the axe but stop 0.2 m before it. 
          edge.lineControl(0.1, 0.0) # (forward m/s, turn-rate rad/sec)
          t.sleep(0.01)
            
          current_distance = ir.ir[1]
          if current_distance < smallest_distance:
            smallest_distance = current_distance
            print(f"smallest distance1 {smallest_distance}")
      
        #stop the robot 
        print("Distance less than 0.2m")
        edge.lineControl(0.0, 0.0) # (forward m/s, turn-rate rad/sec)
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)

        #wait 5 sec to estimate the distance from the axe
        while(((datetime.now() - beginTime).total_seconds()) <= 5): 
          current_distance = ir.ir[1]
          if current_distance < smallest_distance:
            smallest_distance = current_distance
            print(f"smallest distance2 {smallest_distance}")
        
        if(smallest_distance > 0.15): #drive a small distance at low velocity to get closer to axe. 
          drive_distance = smallest_distance - 0.15
          print(f"drive distance: {drive_distance}")
          MieDriveUserMeter(drive_distance, 0.07)
        
        MieState = 2
      
      if MieState == 2: #compute velocity
        # distance_to_pass_axe = 0.46 #meter
        # time_to_pass_axe = MieMeasureTimeToPassAxe() #seconds
        # velocity_to_pass_axe = distance_to_pass_axe / time_to_pass_axe #m/s
        # print(f"Velocity to pass axe is: {velocity_to_pass_axe} m/s")
        MieState = 3

      if MieState == 3: #pass gate 
        MiePassAxe()
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        t.sleep(0.5)
        edge.lineControl(0.2, 0.0) 
        service.send("robobot/cmd/ti/rc","0.2 0.0") # (forward m/s, turn-rate rad/sec)
        print("Down here")
        MieState = 4 
      
      if MieState == 4: #exit axe challenge
        distance = ir.ir[1]
        print(f"hej distance is: {distance}")

        while(distance > 0.4):
          distance = ir.ir[1]
          print(f"hej distance is: {distance}")
          t.sleep(0.2)

        edge.lineControl(0,0) # stop following line
        service.send(service.topicCmd + "ti/rc","0.0 0.0") # turn left
        MieState = 5 
      
      if MieState == 5: 
        print("Axechallenge is Finished")
        state=70
      
    elif state == 70:
      service.send(service.topicCmd + "T0/servo","1 -800 200")
      detectBall = 2 # 2 for red ball, 3 for blue ball
      driveTurnDegree(130, 0.01)
      drive2Object(distance=8000, object=detectBall) # Find the red ball
      if detectBall == 2:
        state = 71
      elif detectBall == 3:
        state = 73
    elif state == 71:
      #if ir.ir[1] < 1:
      #  driveTurnDegree(90, 0.01)
      arucoMark = drive2Aruco(marker=0, rotate=False) # Find the aruco marker (ID: 12/13)
      service.send(service.topicCmd + "T0/servo","1 -800 200") # Let the gripper go up
      state = 72
    elif state == 72:
      arucoMark = drive2Aruco(marker=0, rotate=True)
      MieDriveUserMeter(0.3, -0.1)
      if arucoMark == "13":
        driveTurnDegree(-90, 0.01)
      elif arucoMark == "12":
        driveTurnDegree(-90, 0.01)
        MieDriveUserMeter(0.5, -0.1)
        driveTurnDegree(110, 0.01)
      service.send(service.topicCmd + "ti/rc","0.1 0.0") 
      state = 75
    elif state == 73:
      arucoMark = drive2Aruco(marker=1, rotate=False) # Find the aruco marker (ID: 14/15)
      service.send(service.topicCmd + "T0/servo","1 -800 200") # Let the gripper go up
      state = 74
    elif state == 74:
      drive2Aruco(marker=1, rotate=True)
      driveTurnDegree(45, 0.01)
      MieDriveUserMeter(1.5, 0.1)
      driveTurnDegree(50, 0.01)
      print("ball sorting challenge is ended!") 
      state = 75
    elif state==75:
      if edge.lineValidCnt > 4:
        edge.lineControl(0.2,0)
        state=76
    elif state == 76:
      if edge.average>600:
        edge.lineControl(0.0, 0)
        service.send("robobot/cmd/ti/rc","0.0 0.0")
        pose.tripBreset()
        drive2Aruco(marker=2, rotate=False) # Find the aruco marker (ID: 25)
        state = 78
    #elif state == 77:
    #  if edge.crossingLineCnt > 10:
    #    edge.lineControl(0,0) # Stop moving
    #    service.send("robobot/cmd/ti/rc","0.0 0.0")
    #    pose.tripBreset()
    #    drive2Aruco(marker=2, rotate=False) # Find the aruco marker (ID: 25)
    #    state = 78
    elif state==78:
      #drive2Object(distance=None, velocity=None,object=4) #detecting goal
      print("all challenge is ended!") 
      state=100

    elif state == 101:
      driveOneMeter()
      state = 100
    elif state == 102:
      driveTurnPi()
      state = 100
    elif state == 103:
      edge.lineControl(0.2,0)
      
    elif state==105: #move toward golf ball!
      drive2Object(object=0)
      break
    elif state ==250: #MEMO: for arugument -um user meter 
      driveuserMeter(userdistance,uservelocity)
      state=100
      break
    elif state ==260: #MEMO : for argument -ud user degree
      driveTurnDegree(-90, -0.2)
      break
    elif state == 300:
      service.send(service.topicCmd + "T0/servo","1 50 0")
      break
    elif state==199:
      pass
    elif state==9999 or gpio.stop():
      edge.lineControl(0,0)
      service.send(service.topicCmd + "T0/servo","1 1200 0")
      service.send("robobot/cmd/ti/rc","0 0")
      print("Mission terminated!")
      break
    elif state==8080:
      service.send(service.topicCmd + "T0/servo","1 -800 0")
    else: # abort
      print(f"% Mission finished/aborted; state={state}")
      break
      
    # allow openCV to handle imshow (if in use)
    # images are almost useless while turning, but
    # used here to illustrate some image processing (painting)
    if (cam.useCam):
      imageAnalysis(False) # MEMO: we have to change it into TRUE if we want to use and save image of edge detection
      imagesaving(False) # MEMO : This is for saving images at every sequence!
      key = cv.waitKey(100) # ms
      if key > 0: # e.g. Esc (key=27) pressed with focus on image
        break
    #
    # note state change and reset state timer
    if state != oldstate:
      flog.write(state)
      flog.writeRemark(f"% State change from {oldstate} to {state}")
      print(f"% State change from {oldstate} to {state}")
      oldstate = state
      stateTime = datetime.now()
    # do not loop too fast
    t.sleep(0.01)
    pass # end of while loop
  # end of mission, turn LEDs off and stop
  service.send(service.topicCmd + "T0/leds","16 0 0 0") 
  gpio.set_value(20, 0)
  edge.lineControl(0,0) # stop following line
  service.send(service.topicCmd + "ti/rc","0 0")
  t.sleep(0.05)
  pass

############################################################

if __name__ == "__main__":
    if service.process_running("mqtt-client"):
      print("% mqtt-client is already running - terminating")
      print("%   if it is partially crashed in the background, then try:")
      print("%     pkill mqtt-client")
      print("%   or, if that fails use the most brutal kill")
      print("%     pkill -9 mqtt-client")
    else:
      # set title of process, so that it is not just called Python
      setproctitle("mqtt-client")
      print("% Starting")
      # where is the MQTT data server:
      service.setup('localhost') # localhost
      #service.setup('10.197.217.81') # Juniper
      #service.setup('10.197.217.80') # Newton
      #service.setup('bode.local') # Bode
      if service.connected:
        pose.setup()
        loop()
      service.terminate()
    print("% Main Terminated")
