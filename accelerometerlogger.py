#!/usr/bin/env python

################################################################################
# COPYRIGHT(c) 2018 STMicroelectronics                                         #
#                                                                              #
# Redistribution and use in source and binary forms, with or without           #
# modification, are permitted provided that the following conditions are met:  #
#   1. Redistributions of source code must retain the above copyright notice,  #
#      this list of conditions and the following disclaimer.                   #
#   2. Redistributions in binary form must reproduce the above copyright       #
#      notice, this list of conditions and the following disclaimer in the     #
#      documentation and/or other materials provided with the distribution.    #
#   3. Neither the name of STMicroelectronics nor the names of its             #
#      contributors may be used to endorse or promote products derived from    #
#      this software without specific prior written permission.                #
#                                                                              #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE    #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE   #
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE    #
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR          #
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF         #
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS     #
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN      #
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)      #
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE   #
# POSSIBILITY OF SUCH DAMAGE.                                                  #
################################################################################

################################################################################
# Author:  Davide Aliprandi, STMicroelectronics                                #
################################################################################

from __future__ import print_function
import sys
import os
import time
from abc import abstractmethod
import math
import smtplib
import uuid

import twilio
from twilio.rest import Client

from blue_st_sdk.manager import Manager
from blue_st_sdk.manager import ManagerListener
from blue_st_sdk.node import NodeListener
from blue_st_sdk.feature import FeatureListener
from blue_st_sdk.features.audio.adpcm.feature_audio_adpcm import FeatureAudioADPCM
from blue_st_sdk.features.audio.adpcm.feature_audio_adpcm_sync import FeatureAudioADPCMSync

# Presentation message.
INTRO = """##################
# SensorTile data logger #
##################"""

# Bluetooth Scanning time in seconds (optional).
SCANNING_TIME_s = 5

#
# Printing intro.
#
def print_intro():
    print('\n' + INTRO + '\n')


# INTERFACES
account_sid = 'AC6935cc0c6fab67bd18dc9c23edb6f705'
auth_token = 'afc76eae4e07874cf2d3a2195265f965'

#
# Implementation of the interface used by the Manager class to notify that a new
# node has been discovered or that the scanning starts/stops.
#
class MyManagerListener(ManagerListener):

    #
    # This method is called whenever a discovery process starts or stops.
    #
    # @param manager Manager instance that starts/stops the process.
    # @param enabled True if a new discovery starts, False otherwise.
    #
    def on_discovery_change(self, manager, enabled):
        print('Discovery %s.' % ('started' if enabled else 'stopped'))
        if not enabled:
            print()

    #
    # This method is called whenever a new node is discovered.
    #
    # @param manager Manager instance that discovers the node.
    # @param node    New node discovered.
    #
    def on_node_discovered(self, manager, node):
        print('New device discovered: %s.' % (node.get_name()))


#
# Implementation of the interface used by the Node class to notify that a node
# has updated its status.
#
class MyNodeListener(NodeListener):

    #
    # To be called whenever a node connects to a host.
    #
    # @param node Node that has connected to a host.
    #
    def on_connect(self, node):
        print('Device %s connected.' % (node.get_name()))

    #
    # To be called whenever a node disconnects from a host.
    #
    # @param node       Node that has disconnected from a host.
    # @param unexpected True if the disconnection is unexpected, False otherwise
    #                   (called by the user).
    #
    def on_disconnect(self, node, unexpected=False):
        print('Device %s disconnected%s.' % \
            (node.get_name(), ' unexpectedly' if unexpected else ''))
        if unexpected:
            # Exiting.
            print('\nExiting...\n')
            sys.exit(0)


def connect_ble(MAC):
    # Creating Bluetooth Manager.
    manager = Manager.instance()
    manager_listener = MyManagerListener()
    manager.add_listener(manager_listener)

    while True:
        # Synchronous discovery of Bluetooth devices.
        print('Scanning Bluetooth devices...\n')
        manager.discover(SCANNING_TIME_s)
            
        # Getting discovered devices.
        discovered_devices = manager.get_nodes() 
        if not discovered_devices:
            print('No Bluetooth devices found. Exiting...\n')
            sys.exit(0)

        # Checking discovered devices.
        device = None
        for discovered in discovered_devices:
            if discovered.get_tag() == MAC: #Compares discovered devices MAC with the MAC provided
                device = discovered #If found, then that is the device
                break
        if not device:
            print('Device not found...\n')
            sys.exit(0)

        # Connecting to the devices.
        node_listener = MyNodeListener()
        device.add_listener(node_listener)
        print('Connecting to %s...' % (device.get_name()))
        if not device.connect():
            print('Connection failed.\n')
            sys.exit(0)
        print('Connection done.')
        return device #Returns the device object

def listfeatures(device): #Returns all all the features of the device
    # Getting features.
    features = device.get_features() 
    
    print('\nFeatures:') #Print the device features
    i = 0
    for feature in features: 
        print('%d) %s' % (i, feature.get_name()))
        i+=1
    return features

def readsensor(device, sensor): #Returns a single data of any sensor
    device.enable_notifications(sensor)
    dado='0'
    if device.wait_for_notifications(3):
        dado = str(sensor)
    device.disable_notifications(sensor)
    return dado
    
def logdata(data, logger, logsize, path, starttime):

    separate = data.split(' ') #Default accelerometer data comes in this format "Accelerometer(31283): ( X: 59 mg    Y: 995 mg    Z: 39 mg )", so i want to get only the x y z for the log
    x = round(float(separate[3])*9.8/1000,1)
    y = round(float(separate[9])*9.8/1000,1)#Transforms mg to m/s^2
    z = round(float(separate[15])*9.8/1000,1)
    executiontime = round((time.time() - starttime)*1000) 
    xyzt = str(x)+" "+str(y)+" "+str(z)+" "+str(executiontime)
    
    if abs(x) > 10:
        sendFallMessage()
    
    if len(logger) < logsize : #If log has not reached the logsize, just add elements
        logger.append(xyzt)
    else: #If log has reached the logsize, delete the first element, and make all elements go back 1 index
        i=0
        while i < logsize-1:
            logger[i] = logger[i+1]
            i=i+1
        logger[logsize-1] = xyzt
    log = open(path, "w")
    for line in logger:
        log.write(line);
        log.write('\n')
    log.close()
    print('X: %.1f m/s2 Y: %.1f m/s2 Z: %.1f m/s2 Time: %.1f ms' % (x,y,z,executiontime))
    return logger
            
def sendFallMessage():
    client = Client(account_sid, auth_token)
    client.messages \
        .create( \
            body="We have detected a fall. Please check on your loved one.",
            from_='+15075788201',
            to='+12037154454')
            
    print('FALL DETECTED')

    
def main(argv):
    # Printing intro.
    print_intro()
    random_uuid = uuid.uuid4()
    try:
        SENSORTILE_MAC = 'c0:83:24:33:52:58' #Enter your device MAC adress here
        device = connect_ble(SENSORTILE_MAC)
        
        features = listfeatures(device)
        #Features SENSORTILE:
        #0 Temperature
        #1 Humidity
        #2 Pressure
        #3 Magnetometer
        #4 Gyroscope
        #5 Accelerometer
        accelerometer = features[5] #Change here what feature you want
        
        logger = [] #Array to store log data
        logger2 = []
        logsize = 1000000 #Change here to increase or decrease logsize
        starttime = time.time() #Gets the timestamp reference
        path_acc = f"/home/pi/Documents/accerlerometer_logger_{random_uuid}.txt" #Path of the log file for accelerometer
        path_gyr = "/home/pi/Documents/gyroscope_logger.txt"
        
        while True:
            data = readsensor(device, accelerometer) #Calls the readsensor devices, returns the sensor data
            logger= logdata(data, logger, logsize, path_acc, starttime) #Logs the sensordata in a txt file

            # data2 = readsensor(device, gyroscope)
            # logger2 = logdata(data2, logger2, logsize, path_gyr, starttime)
        
        

    except KeyboardInterrupt:
        try:
            # Exiting.
            print('\nExiting...\n')
            sys.exit(0)
        except SystemExit:
            os._exit(0)


if __name__ == "__main__":

    main(sys.argv[1:])
