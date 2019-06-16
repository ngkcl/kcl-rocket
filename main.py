from picamera import PiCamera
from time import sleep
import time
from smbus import SMBus
import RPi.GPIO as GPIO
import sys

# Camera Module Constants
VIDEO_RESOLUTION = (1920, 1080)
PICTURE_RESOLUTION = (2592, 1944)
VIDEO_DIR = "/home/pi/video_out.h264"
PICTURE_DIR = "/home/pi/camera_still.jpg"

cam_mod = None

# I2C ADDRESS/BITS

MPL3115A2_ADDRESS = (0x60)

# MPL3115A2 REGISTERS

MPL3115A2_REIGSTER_STATUS = (0x00)
MPL3115A2_REGISTER_STATUS_TDR = 0x02
MPL3115A2_REIGSTER_STATUS_PDR = 0x04
MPL3115A2_REGISTER_STATUS_PTDR = 0x08

MPL3115A2_REGISTER_PRESSURE_MSB = (0x01)
MPL3115A2_REGISTER_PRESSURE_CSB = (0x02)
MPL3115A2_REGISTER_PRESSURE_LSB = (0x03)

MPL3115A2_REGISTER_TEMP_MSB = (0x04)
MPL3115A2_REIGSTER_TEMP_LSB = (0x05)

MPL3115A2_REGISTER_DR_STATUS = (0x06)

MPL3115A2_OUT_P_DELTA_MSB = (0x07)
MPL3115A2_OUT_P_DELTA_CSB = (0x08)
MPL3115A2_OUT_P_DELTA_LSB = (0x09)

MPL3115A2_OUT_T_DELTA_MSB = (0x0A)
MPL3115A2_OUT_T_DELTA_LSB = (0x0B)

MPL3115A2_BAR_IN_MSB = (0x14)

MPL3115A2_WHOAMI = (0x0C)

# MPL3115A2 BITS

MPL3115A2_PT_DATA_CFG = 0x13
MPL3115A2_PT_DATA_CFG_TDEFE = 0x01
MPL3115A2_PT_DATA_CFG_PDEFE = 0x02
MPL3115A2_PT_DATA_CFG_DREM = 0x04

MPL3115A2_CTRL_REG1 = (0x26)
MPL3115A2_CTRL_REG1_SBYB = 0x01
MPL3115A2_CTRL_REG1_OST = 0x02
MPL3115A2_CTRL_REG1_RST = 0x04
MPL3115A2_CTRL_REG1_OS1 = 0x00
MPL3115A2_CTRL_REG1_OS2 = 0x08
MPL3115A2_CTRL_REG1_OS4 = 0x10
MPL3115A2_CTRL_REG1_OS8 = 0x18
MPL3115A2_CTRL_REG1_OS16 = 0x20
MPL3115A2_CTRL_REG1_OS32 = 0x28
MPL3115A2_CTRL_REG1_OS64 = 0x30
MPL3115A2_CTRL_REG1_OS128 = 0x38
MPL3115A2_CTRL_REG1_RAW = 0x40
MPL3115A2_CTRL_REG1_ALT = 0x80
MPL3115A2_CTRL_REG1_BAR = 0x00
MPL3115A2_CTRL_REG2 = (0x27)
MPL3115A2_CTRL_REG3 = (0x28)
MPL3115A2_CTRL_REG4 = (0x29)
MPL3115A2_CTRL_REG5 = (0x2A)

MPL3115A2_REGISTER_STARTCONVERSION = (0x12)

bus = SMBus(1)

# Rocket variables
DISPATCH_ALTITUDE = 0
DISPATCH_PRESSURE = 0
ALTITUDE_THRESHOLD = 10
PRESSURE_THRESHOLD = 0
DISPATCH_TIME_AFTER_LAUNCH = 2

current_altitude = 0
current_pressure = 0

altitude_delta = 0

# Launched flag - first stage
has_launched = False

# Dispatched flag - second stage
has_dispatched = False

# GPIO config
RELAY_PIN = 24

def init_mpl():
    global bus

    whoami = bus.read_byte_data(MPL3115A2_ADDRESS, MPL3115A2_WHOAMI)
    if whoami ! = 0xc4:
        print("[X] ERROR: Device not active")
        # exit

    bus.write_byte_data(
            MPL3115A2_ADDRESS,
            MPL3115A2_CTRL_REG1,
            MPL3115A2_CTRL_REG1_SBYB |
            MPL3115A2_CTRL_REG1_OS128 |
            MPL3115A2_CTRL_REG1_ALT)
    
    bus.write_byte_data(
            MPL3115A2_ADDRESS,
            MPL3115A2_PT_DATA_CFG,
            MPL3115A2_PT_DATA_CFG_TDEFE |
            MPL3115A2_PT_DATA_CFG_PDEFE |
            MPL3115A2_PT_DATA_CFG_DREM)

def poll_mpl():
    sta = 0
    while not (sta & MPL3115A2_REIGSTER_STATUS_PDR):
        sta = bus.read_byte_data(MPL3115A2_ADDRESS, MPL3115A2_REIGSTER_STATUS)

def get_altitude():
    bus.write_byte_data(
            MPL3115A2_ADDRESS,
            MPL3115A2_CTRL_REG1,
            MPL3115A2_CTRL_REG1_SBYB |
            MPL3115A2_CTRL_REG1_OS128 |
            MPL3115A2_CTRL_REG1_ALT)

    poll_mpl()

    msb, csb, lsb = bus.read_i2c_block_data(MPL3115A2_ADDRESS, MPL3115A2_REGISTER_PRESSURE_MSB, 3)
    print(msb, csb, lsb)
    
    alt = ((msb<<24) | (csb<<16) | (lsb<<8)) / 65536

    # correct sign
    if alt > (1<<15):
        alt -= 1<<16

    return alt

def get_pressure():
    bus.write_byte_data(
            MPL3115A2_ADDRESS,
            MPL3115A2_CTRL_REG1,
            MPL3115A2_CTRL_REG1_SBYB |
            MPL3115A2_CTRL_REG1_OS128 | 
            MPL3115A2_CTRL_REG1_BAR)

    poll_mpl()

    msb, csb, lsb = bus.read_i2c_block_data(MPL3115A2_ADDRESS, MPL3115A2_REGISTER_PRESSURE_MSB, 3)
    print(msb, csb, lsb)

    return ((msb<<16) | (csb<<8) | lsb) / 64

def calibrate_mpl():
    pa = int(pressure()/2)
    bus.write_i2c_block_data(MPL3115A2_ADDRESS, MPL3115A2_BAR_IN_MSB, [pa>>8 & 0xff, pa & 0xff])


def init_cam_mod():
    global cam_mod

    try:
        cam_mod = PiCamera() 
    except Exception as e:
        print("[X] ERROR: {}".format(e))
        sys.exit(0)

def test_camera(is_video = True, recording_time = 3):
    if is_video:
        cam_mod.start_preview()
        cam_mod.start_recording(VIDEO_DIR)
        sleep(recording_time)
        cam_mod.stop_recording()
        cam_mod.stop_preview()
    else:
        cam_mod.start_preview()
        sleep(5)
        cam_mod.capture(PICTURE_DIR)
        cam_mod.stop_preview()

def record_video():
    cam_mod.start_preview()
    cam_mod.start_recording(VIDEO_DIR)

def stop_video():
    cam_mod.stop_recording()
    cam_mod.stop_preview()

# relay trigger functions
trigger_relay_up = lambda: GPIO.output(RELAY_PIN, 1)
trigger_relay_down = lambda: GPIO.output(RELAY_PIN, 0)

def main():
    global current_altitude, current_pressure, has_launched
    init_mpl()
    init_cam_mod()
    
    # Setup GPIO pins for relay
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RELAY_PIN, GPIO.OUT, initial = 0)

    # run tests (remove in prod):
    test_camera()

    time.sleep(7)
    
    # TODO: blink LED?

    # Start recording:
    record_video()

    while True:
        time.sleep(0.03)
        new_altitutde = get_altitude()
        time.sleep(0.03)
        new_pressure = get_pressure()

        if (new_altitude - current_altitude) > ALTITUDE_THRESHOLD:
            has_launched = True
            
            # wait allocated time before dispatching second stage
            time.sleep(DISPATCH_TIME_AFTER_LAUNCH)
            
            # activate relay
            trigger_relay_up()

            # wait 2 seconds and close relay
            time.sleep(2)
            trigger_relay_down()
           
            # wait a bit more for some good footage
            time.sleep(10)
            break

    # Stop recording
    stop_video()

if __name__ == "__main__":
    main()
