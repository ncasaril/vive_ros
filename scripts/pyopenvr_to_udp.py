#!/bin/env python

import sys
import time
import openvr

import sys, argparse
from socket import *

# TODO: Rewrite using subprocess for python 3 compat.
def find_broadcast_ip():
    # Find the broadcast address for eth0 or wlan0
    #ips = commands.getoutput("/sbin/ifconfig|grep -A1 -e 'eth0' -e 'wlan0' -e 'enp3s0'").split("\n")
    try:
        myip = ips[1].split()[1][5:]
        broadip = inet_ntoa( inet_aton(myip)[:3] + b'\xff' )
    except socket.error:
        try:
            myip = ips[4].split()[1][5:]
            broadip = inet_ntoa( inet_aton(myip)[:3] + b'\xff' )
        except error:
            print("Can't get broadcast address")
            exit(2)

    print('LAN broadcast', broadip)
    return broadip


parser = argparse.ArgumentParser()
parser.add_argument('-p',metavar='port',type=int,dest='port',help='udp port to listen to',default=8002)
parser.add_argument('-b',type=str,dest='ip',help='broadcast ip')

args = parser.parse_args()

udpport=8002

if (args.port): udpport = args.port
if (args.ip): broadip = args.ip
else: broadip=find_broadcast_ip()

# Setup socket
s=socket(AF_INET, SOCK_DGRAM)
s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)


openvr.init(openvr.VRApplication_Scene)

# Create data structure to hold locations of tracked devices such as headsets, controllers, and lighthouses
poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
poses = poses_t()

okToRun = True

while okToRun:
    openvr.VRCompositor().waitGetPoses(poses, len(poses), None, 0)
    for j in range(openvr.k_unMaxTrackedDeviceCount):
        #hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]
        hmd_pose = poses[j]
        if hmd_pose.eTrackingResult != openvr.TrackingResult_Running_OK: continue
        if hmd_pose.bDeviceIsConnected == False: continue
        dev_type = openvr.IVRSystem().getTrackedDeviceClass(j)
        P=hmd_pose.mDeviceToAbsoluteTracking
        sd = str(P)
        # Translate into json for sending over UDP
        d=bytes('{"vive_id":%d'%j + ',"type_id":%d'%dev_type +',"time":"%f"'%time.time()+',"pose":' + sd + '}', 'utf-8')
        s.sendto(d,(broadip,udpport))
        
    e = openvr.VREvent_t()
    while (openvr.VRSystem().pollNextEvent(e) == True):
        send = False
        # Translate into json for sending over UDP
        if e.eventType == openvr.VREvent_ButtonPress:   send = True
        if e.eventType == openvr.VREvent_ButtonUnPress: send = True
        if e.eventType == openvr.VREvent_Quit:
            okToRun = False
            send = True
            
        if send==False: continue
        
        dev_type = openvr.IVRSystem().getTrackedDeviceClass(e.trackedDeviceIndex)
        d=bytes('{"vive_id":%d'%e.trackedDeviceIndex + ',"type_id":%d'%dev_type +',"time":"%f"'%time.time()
                + ',"event":"' + openvr.getEventTypeNameFromEnum(e.eventType) +'"}', 'utf-8')
        s.sendto(d,(broadip,udpport))
        
    time.sleep(0.02)

openvr.shutdown()
