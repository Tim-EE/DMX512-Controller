# This file is used to command each channel on the DMX universe to blink one
# after another. This way, you can tell which light is on which channel.

from __future__ import print_function
import serial
import random
import sys
import argparse
import datetime
from time import sleep

HW_OPTIONS = {'DMX_USB_PRO','CUSTOM'}
HW_SELECTION = 'DMX_USB_PRO'

DMX_ARRAY_LENGTH = 30
dmxArray = [0] * DMX_ARRAY_LENGTH

if DMX_ARRAY_LENGTH < 25:
    print("DMX_ARRAY_LENGTH cannot be less than 25, setting to 25.")
    DMX_ARRAY_LENGTH = 25
elif DMX_ARRAY_LENGTH > 512:
    print("DMX_ARRAY_LENGTH cannot be greater than 512, setting to 512.")
    DMX_ARRAY_LENGTH = 512

# print("DMX_ARRAY_LENGTH: {:d}".format(DMX_ARRAY_LENGTH))

def MakeSerialData(channelIndex,channelValue):
    serialData = None

    if HW_SELECTION == 'DMX_USB_PRO':
        dmxArray[channelIndex-1] = channelValue
        serialDataStr = ''.join('{:02x} '.format(x) for x in dmxArray)
        lowByteLength = ''.join('{:02x} '.format(DMX_ARRAY_LENGTH % 256))
        highByteLength = ''.join('{:02x} '.format(int(DMX_ARRAY_LENGTH / 256) % 256))
        serialDataStr = '7E 06 ' + lowByteLength + highByteLength + serialDataStr + 'E7'
        print(serialDataStr)
        serialData = bytearray.fromhex(serialDataStr)
        # quit()

    elif HW_SELECTION == 'CUSTOM':
        serialDataStr = ''.join('{:02x} '.format(x) for x in [0x5B, channelIndex,channelValue, 0x5D])
        # serialData = bytearray.fromhex('5B {:x} {:x} 5D'.format(channelIndex,channelValue))
        serialData = bytearray.fromhex(serialDataStr)
        print(serialDataStr)
        # quit()

    return serialData

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("-p","--port", type=int,help="The serial COM port to use")
    parser.add_argument("-max","--max-channel-count", type=int,help="The serial COM port to use")
    # parser.add_argument("-hw","--max-channel-count", type=int,help="The serial COM port to use")
    # if sys.argv[1:]:
        # args = parser.parse_args()
    args = parser.parse_args()

    maxChannel = DMX_ARRAY_LENGTH
    if args.max_channel_count:
        maxChannel = args.max_channel_count

    # if True:
    with serial.Serial() as ser:
        # ser.baudrate = 9600
        ser.baudrate = 115200
        ser.port = 'COM{:d}'.format(args.port) # look for VCOM FT245BM device driver
        ser.timeout = 1
        ser.open()
        # Get Widget Serial Number Request (Label = 10, no data)
        # check_bytes = bytes.fromhex('5B {:x} {:x} 5D'.format(index,255))
        # check_bytes = bytearray.fromhex('5B %x %x 5D'.format(index,255))
        # print("Toggling index {:d}".format(index))

        direction = -1
        dispValue = 255
        incrementValue = 5
        incrementChannelAfterNTimes = 5
        currentChannelIndex = 1
        currentChannelIterationCount = 0

        while True:
            # check_bytes = bytes.fromhex('5B {:x} {:x} 5D'.format(currentChannelIndex,dispValue))
            serialBytes = MakeSerialData(currentChannelIndex,dispValue)
            if serialBytes != None:
                ser.write(serialBytes)

            # set new value
            if (dispValue - incrementValue < 0):
                direction = 1
                currentChannelIterationCount +=1

            elif (dispValue + incrementValue > 2**8-1):
                direction = -1


            dispValue += incrementValue * direction
            # print("dispValue: " + str(dispValue))
            # print(str([hex(x) for x in serialBytes]))

            # if HW_SELECTION != 'DMX_USB_PRO':
            sleep(0.01)

            if (currentChannelIterationCount == incrementChannelAfterNTimes):
                currentChannelIterationCount = 0
                currentChannelIndex += 1
                if (currentChannelIndex >= maxChannel):
                    currentChannelIndex = 1

                print("currentChannelIndex: " + str(currentChannelIndex))

            # quit()
