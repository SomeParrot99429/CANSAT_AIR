import time
import board
import busio
import digitalio
import sdcardio
import storage

import adafruit_rfm9x
import adafruit_bmp280
import adafruit_gps
import adafruit_sht4x

# gps setup
i2c = busio.I2C(board.GP27, board.GP26)  # has gps, temperature and pressure sensor on one pin
gps = adafruit_gps.GPS_GtopI2C(i2c)
gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b"PMTK220,1000")
last_print = time.monotonic()

# radio set up
spi = busio.SPI(clock=board.GP2, MOSI=board.GP3, MISO=board.GP4)
cs = digitalio.DigitalInOut(board.GP6)
reset = digitalio.DigitalInOut(board.GP7)
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 433.0)

# sd card set up
cs = board.GP5
sdcard = sdcardio.SDCard(spi, cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")

# temperature and pressure setup
bmp280_sensor = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)

# temperature and humidity setup
sht = adafruit_sht4x.SHT4x(i2c)

# board light
light = digitalio.DigitalInOut(board.GP25)
light.switch_to_output(value=True)
light_on = True

# other variables
take_image = False
send_image = False


def new_file():
    title = f"last_run"

    file = open(f"/sd/{title}.txt", "w")
    file.write("")
    file.close()


def write(data):
    data = f"{str(time.monotonic())}|{data}"
    title = f"last_run"
    file = open(f"/sd/{title}.txt", "a")
    file.write(data)
    file.close()


def read_temperature():
    return bmp280_sensor.temperature


def read_pressure():
    return bmp280_sensor.pressure


def read_altitude():
    return bmp280_sensor.altitude


def get_gps_location():
    gps.update()
    try:
        if gps.has_fix:
            if gps.latitude is not None:
                latitude = float(gps.latitude)
            else:
                latitude = 0

            if gps.longitude is not None:
                longitude = float(gps.longitude)
            else:
                longitude = 0

            if gps.altitude_m is not None:
                alt = float(gps.altitude_m)
            else:
                alt = 0

            if gps.speed_knots is not None:
                speed = float(gps.speed_knots)
            else:
                speed = 0

            if gps.track_angle_deg is not None:
                track_angle = float(gps.track_angle_deg)
            else:
                track_angle = 0

            if gps.horizontal_dilution is not None:
                horizontal_dilution = float(gps.horizontal_dilution)
            else:
                horizontal_dilution = 0

        else:
            latitude, longitude, alt, speed, track_angle, horizontal_dilution = 0, 0, 0, 0, 0, 0

        return latitude, longitude, alt, speed, track_angle, horizontal_dilution
    except:
        return 0, 0, 0, 0, 0, 0


def send(to_send):
    rfm9x.send(to_send)


def try_read(timeout):
    return rfm9x.receive(timeout=timeout)


new_file()
while True:
    light_on = not light_on
    light.value = light_on
    loop_start_time = time.time()
    try:
        sht_temperature = sht.temperature
    except:
        sht_temperature = 0
    time.sleep(0.1)
    try:
        humidity = sht.relative_humidity
    except:
        humidity = 0

    try:
        bmp_temperature = read_temperature()
    except:
        bmp_temperature = 0

    if sht_temperature == 0 or bmp_temperature == 0:
        temperature = 0
    else:
        temperature = (bmp_temperature + sht_temperature) / 2
    time.sleep(0.1)
    try:
        pressure = read_pressure()
    except:
        pressure = 0
    time.sleep(0.1)
    try:
        altitude = read_altitude()
    except:
        altitude = 0

    try:
        lat, long, alt, speed, track_angle, horizontal_dilution = get_gps_location()
    except:
        lat, long, alt, speed, track_angle, horizontal_dilution = 0, 0, 0, 0, 0, 0

    try:
        message = "data|{0:.3f}|{1:.3f}|{2:.3f}|{3:.6f}|{4:.6f}|{5:.3f}|{6:.3f}|{7:.3f}|{8:.3f}|{9:.3f}|\n".format(
            altitude, temperature, pressure, lat, long, humidity, alt, speed, track_angle, horizontal_dilution)
        send(message)
        write(message)
        print(message)

    except:
       print("this is not ment to appear....")

    try:
        time.sleep((0.7 - ((time.time() - loop_start_time) % 0.7)))
    except:
        print("it don't want to sleep right now")


