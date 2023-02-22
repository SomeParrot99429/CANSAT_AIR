import busio
import digitalio
import sdcardio
import storage

# new import
import time

import adafruit_ov2640

spi = busio.SPI(clock=board.GP2, MOSI=board.GP3, MISO=board.GP4)

# sd card set up
cs = board.GP5
sdcard = sdcardio.SDCard(spi, cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")

with digitalio.DigitalInOut(board.GP10) as reset:
    reset.switch_to_output(False)
    time.sleep(0.001)
    bus = busio.I2C(board.GP9, board.GP8)

cam = adafruit_ov2640.OV2640(
    bus,
    data_pins=[
        board.GP12,
        board.GP13,
        board.GP14,
        board.GP15,
        board.GP16,
        board.GP17,
        board.GP18,
        board.GP19,
    ],
    clock=board.GP11,
    vsync=board.GP7,
    href=board.GP21,
    mclk=board.GP20,
    shutdown=None,
    reset=board.GP10,
)

cam.flip_x = True
cam.flip_y = True

def take_image():
    try:
        cam.sifze = adafruit_ov2640.OV2640_SIZE_VGA
        cam.colorspace = adafruit_ov2640.OV2640_COLOR_JPEG
        b = bytearray(cam.capture_buffer_size)
        # add time stuff
        jpeg = cam.capture(b)
        count = count +1
        with open(f"/sd/img{count:04d}.jpeg", "wb") as file:
            file.write(jpeg)

        count += 1

    except:
        print("failed")

count = 0
take_image()