#!/usr/bin/env python

# Written by Mikael Glamheden
# 2019-06-04

# Controller for following straight line paths and orienting car.

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
    k1 = 1.0 # velocity gain
    k2 = 2.0 # angular velocity gain

    yaw_ref = math.atan2(yg-y0,xg-x0)
    yaw_ref = normalize_angle(yaw_ref)

    # Velocity control
    dg = math.cos(yaw_ref)*(xg-x) + math.sin(yaw_ref)*(yg-y)
    v = k1*dg
    v =  - saturate(v,0.5)

    # Angle control
    dp = - (- math.pi + yaw_ref - yaw)
    # dp = math.sin(yaw_ref)*(x+p*math.cos(yaw)-x0) - math.cos(yaw_ref)*(y+p*math.sin(yaw)-y0)
    w = k2*dp

    return v, w
def line_follower(x, y, yaw, x0, y0, xg, yg):
    p = 0.5 # look ahead gain
    k1 = 2.0 # velocity gain
    k2 = 2.0 # angular velocity gain

    yaw_ref = math.atan2(yg-y0,xg-x0)
    yaw_ref = normalize_angle(yaw_ref)

    # Velocity control
    dg = math.cos(yaw_ref)*(xg-x) + math.sin(yaw_ref)*(yg-y)
    v = k1*dg
    v = saturate(v, 0.3)

    # Angle control
    # dp = yaw_ref-yaw;
    dp = math.sin(yaw_ref)*(x+p*math.cos(yaw)-x0) - math.cos(yaw_ref)*(y+p*math.sin(yaw)-y0)
    w = k2*dp

    return v, w

def orientation_controller(x, y, yaw, x0, y0, xg, yg):
    k1 = 10 # angular velocity gain
    k2 = 1 # velocity gain

    yaw_ref = math.atan2(yg-y0,xg-x0)
    yaw_ref = normalize_angle(yaw_ref)

    w = k1*(yaw_ref-yaw) # angular velocity controller
    w = saturate(w,math.pi/6)
    # velocity controller
    d0 = math.cos(yaw_ref)*(x0-x) + math.sin(yaw_ref)*(y0-y)
    v = k2*d0
    # if v < 0:
    #     v = v - 0.05
    # if v >= 0:
    #     v  = v + 0.05
    v = 0.3 # override velocity controller.
    return v, w
