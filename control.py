from __future__ import print_function
import myo
from os import system, name
from serial import *
import time
serial_port = Serial(("Com7"), baudrate = 57600, timeout=0.05)
time.sleep(3)

j2 = bytearray([2,3])
j3 = bytearray([4,5])
j1 = bytearray([6,7])
grip = bytearray([8,9])
pos = bytearray([12,11])
bytePack = [j2,j3,j1,grip,pos]
modes = ["Move J2", "Move J3","J1", "Gripper", "Position"]
count = 0
class Listener(myo.DeviceListener):
    count = 0
    def on_connected(self, event):
        print("Hello, there")
        event.device.vibrate(myo.VibrationType.short)
        event.device.request_battery_level()
    def on_battery_level(self, event):
        print("Your battery level is:", event.battery_level)
    def on_pose(self, event):
        global count #Make the variables global
        global Serial
        system('cls') #Clear the screen
        if event.pose == myo.Pose.fingers_spread or event.pose == myo.Pose.fist:
            count -= 1
        elif event.pose == myo.Pose.double_tap:
            count += 1
        elif event.pose == myo.Pose.wave_in:
            packet = bytearray([bytePack[count][0]])
            print(packet)
            serial_port.write(packet)
        elif event.pose ==myo.Pose.wave_out:
            packet = bytearray([bytePack[count][1]])
            print(packet)
            serial_port.write(packet)
        elif event.pose ==myo.Pose.rest:
            packet = bytearray([1])
            serial_port.write(packet)

            
        if count >= len(modes):
            count = 0
        elif count < 0:
            count = len(modes)-1
        
        for x in range(count):
            print(modes[x])  ###Printing the part before selection
        print(">>>"+modes[count]+"<<<") ###Printing the selected part
        for x in range(len(modes)-1-count):
            print(modes[x+count+1]) ###


if __name__ == '__main__':
    
    myo.init()
    hub = myo.Hub()
    listener = Listener()
    while hub.run(listener.on_event, 500):  
        pass
