import serial
from time import sleep

# SERIAL_PORT = '/dev/ttyUSB0'              # Raspberry
SERIAL_PORT = '/dev/tty.usbserial-140'      # mac di Giacomo


class ArduinoSensors:
    # ultrasonic sensors
    _right_sensor: float
    _left_sensor: float
    # compass sensors
    _orientation: float

    def __init__(self):
        self._right_sensor = 0
        self._left_sensor = 0
        self._orientation = 0
        self.update()

    def _set_values(self, right_sensor, left_sensor, orientation):
        self._right_sensor = right_sensor
        self._left_sensor = left_sensor
        self._orientation = orientation

    def update(self):
        # line read
        my_serial = serial.Serial(SERIAL_PORT, 9600)
        byte = ""
        while byte != b"\n":
            byte = my_serial.read(1)
        line = my_serial.readline()
        line = line.decode('utf-8').strip()
        # splitting
        values_list = line[1:].split("#")
        # setters
        self._set_values(float(values_list[0]), float(values_list[1]), float(values_list[2]))
        my_serial.close()

    def get_right_sensor(self):
        return self._right_sensor

    def get_left_sensor(self):
        return self._left_sensor

    def get_orientation(self):
        return self._orientation


if __name__ == "__main__":
    sensors = ArduinoSensors()
    while True:
        sensors.update()
        print("right: ",sensors.get_right_sensor())
        print("left: ",sensors.get_left_sensor())
        print("orientation: ",sensors.get_orientation())
        sleep(1)
