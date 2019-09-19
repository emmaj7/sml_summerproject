#!/usr/bin/env python

# Written by Mikael Glamheden
# Last editet: 2019-08-28

# Controller for following straight line paths and for turning.

import math

def normalize_angle(angle):
    if angle > math.pi:
        angle -= 2*math.pi
    elif angle < -math.pi:
        angle += 2*math.pi
    return angle

def saturate(v, max):
    if v > max:
        v = max
    if v < -max:
        v = -max
    return v

def line_follower_reverse(x, y, yaw, x0, y0, xg, yg):
    p = 0.4 # look ahead gain
    k1 = 1 # velocity gain
    k2 = 4.0 # angular velocity gain

    yaw_ref = math.atan2(yg-y0,xg-x0)
    yaw_ref = normalize_angle(yaw_ref)

    # Velocity control
    dg = math.cos(yaw_ref)*(xg-x) + math.sin(yaw_ref)*(yg-y)
    v = k1*dg
    v =  - saturate(v,0.3)
    # v = v * 0.9 # if too fast

    angle = normalize_angle(yaw_ref-math.pi)
    # Angle control
    dp = - (angle - yaw)
    # dp = math.sin(yaw_ref)*(x+p*math.cos(yaw)-x0) - math.cos(yaw_ref)*(y+p*math.sin(yaw)-y0)
    w = k2*dp

    return v, w

def line_follower(x, y, yaw, x0, y0, xg, yg):
    p = 0.4 # look ahead gain
    k1 = 0.9 # velocity gain
    k2 = 8.0 # angular velocity gain

    yaw_ref = math.atan2(yg-y0,xg-x0)
    yaw_ref = normalize_angle(yaw_ref)

    # Velocity control
    dg = math.cos(yaw_ref)*(xg-x) + math.sin(yaw_ref)*(yg-y)
    v = k1*dg
    v = saturate(v,0.3)
    # v = v*0.9 # if too fast
    # Angle control
    # dp = yaw_ref-yaw;
    dp = math.sin(yaw_ref)*(x+p*math.cos(yaw)-x0) - math.cos(yaw_ref)*(y+p*math.sin(yaw)-y0)
    w = k2*dp

    return v, w

def orientation_controller(x, y, yaw, yaw_ref, direction):
    k1 = 3.0 # angular velocity gain
    k2 = 1.0 # velocity gain


    if direction == 'L' and (yaw_ref-yaw) < 0:
        w = -k1*(yaw_ref-yaw)
    elif direction == 'R' and (yaw_ref-yaw) > 0:
        w = -k1*(yaw_ref-yaw)
    else:
        w = k1*(yaw_ref-yaw) # angular velocity controller
    w = saturate(w,35*math.pi/180)

    v = 0.3 # override velocity controller.
    #v = v*0.9 # if too fast
    return v, w
