import can
import time

address = '/dev/tty.usbmodem14101'
bus = can.interface.Bus(bustype='serial', channel=address, bitrate=1000000)

count = 0
while True:
    count += 1
    print("loop: ", count)
    bus.recv(timeout = 0.01)
    print(messsage)
