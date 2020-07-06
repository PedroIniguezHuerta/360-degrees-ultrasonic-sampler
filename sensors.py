# 360 degrees ultrasonic sampler
# By Pedro Iniguez Huerta
#
# This program uses all pins of port 1 for MSP430 to read up to 4 ultrasonic
# sensors and send the samples to Raspberry PI through I2C protocol.
# The MSP430 is the slave while the Raspberry PI 3 is the master.
# MSP430 supports the following commands from I2C bus:
#   - GET_SENSORS (50) ==> Return the number of seconds currently supported.
#                          this is configurable at compilation time (SENSORS=4)
#   - READ_SENSORS (50) ==> Return the samples of each ultrasonic seonsor.
#                           Notes:
#                            - To avoid hangs MSP430 starts a timer to abort
#                              waiting for ultrasonic response if MAX_TIMEOUT 
#                              expired
#                            - MSP430 added header '<' and tail '>' to the 
#                              sampling data prior sending ultrasonic samples 
#                              back to Raspberry PI 3.
#
#  This program implements a python version of bachagas Kalman Filter to reduce impact of false positives samples:
#  https://github.com/bachagas/Kalman/blob/master/Kalman.h
#
#   Below is an pinout diagram about how connect MSP430 with Reaspberry PI 3
#
#                                /|\  /|\
#               MSP430G2xx3     100k  100k     Raspberry PI 3
#                   slave         |    |        master
#             -----------------   |    |  -----------------
#HR/SRF05-1 -|P1.1 P1.7/UCB0SDA|<-|---+->|P3 SDA           |-
#HR/SRF05-2 -|P1.2             |  |      |                 |
#HR/SRF05-3 -|P1.3             |  |      |                 |-
#HR/SRF05-4 -|P1.4 P1.6/UCB0SCL|<-+----->|P5 SCL           |
#START SAMP -|P1.5             |         |                 |
# LED       -|P1.0             |         |                 |


import smbus
import time
from usKalmanFilter import USKalman as usk

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

GET_SENSORS = 50
READ_SENSORS = 51
SAMPLING_TIME = 0.05
IDLE_TIME = 0.01
MAX_SENSORS = 4               # Max sensors supported by MSP430
SENSORS = 0
address = 0x48                # MSP430 slave address

displaySensors = True
counter = 1
us_sensors = []

def readSensors():
    global SENSORS, counter
    global us_sensors
    
    ######################################################################
    # Inform MSP430 willing to read ultrasonic sensors
    ######################################################################
    counter += 1
    retries = 0
    retry = True
    while retry:
        try:
            value = GET_SENSORS  # GET_SENSORS command
            bus.write_byte(address, value)
            response=bus.read_i2c_block_data(address, 0, 3) # <sensors>
            if SENSORS == 0:
                print "GET_SENSORS==>",response
            SENSORS = response[1]
            retry = False
        except:
            retries += 1
            if retries > 2:
                retry = False
                print "Raspberry pi not responding1. Aborting execution"
                return 1

    ######################################################################
    # Wait 50 ms prior reading samples
    ######################################################################
    time.sleep(SAMPLING_TIME) #wait 50 ms to read the ultrasonic sensors

    ######################################################################
    # Read Samples
    ######################################################################
    retries = 0
    retry = True
    while retry:
        try:
            value = READ_SENSORS  # READ_SENSORS command
            rsp_len = 2 + SENSORS
            bus.write_byte(address, value)
            response=bus.read_i2c_block_data(address, 0, rsp_len ) # <s1s2>

            for i in range(0,len(response)):
                try:
                    usfilter  = us_sensors[i]
                    #print usfilter.getEstimatedError()
                    #sample = usfilter.getFilteredValue(response[i])
                    #print response[i]
                    response[i] = sample
                except:
                   pass
                if response[i] == 255:
                    response[i] = "-"

            print response[0]*2,",",response[1]*2,",",response[2]*2,",",response[3]*2, counter
            retry = False
        except:
            retries += 1
            if retries > 2:
                retry = False
                print "Raspberry pi not responding2. Aborting execution"
                return 1
    return 0


#############################################################
# main entry point
#############################################################
for i in range(0, MAX_SENSORS):
    usfilter = usk(0.125,32,1023,0)
#    print usfilter.getEstimatedError()
#    sample = usfilter.getFilteredValue(25)
#    print sample
    us_sensors.append(usfilter)

while True:
    if readSensors() == 1:
        break;  #abort execution
    
    time.sleep(IDLE_TIME)

