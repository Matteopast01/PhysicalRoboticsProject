import serial

SERIAL_PORT = '/dev/ttyUSB0'  # on Raspberry pi USB port top right
# SERIAL_PORT = '/dev/tty.usbserial-140' # on Giacomo Mac


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
        # read from sensors
        ser = serial.Serial(SERIAL_PORT, 9600)
        # line read
        line = ser.readline().decode('utf-8').strip()
        # splitting
        values_list = line[1:].split("#")
        # setters
        self._set_values(float(values_list[0]), float(values_list[1]), float(values_list[2]))

    def get_right_sensor(self):
        return self._right_sensor

    def get_left_sensor(self):
        return self._left_sensor

    def get_orientation(self):
        return self._orientation


if __name__ == "__main__":
    sensors = ArduinoSensors()
    print(sensors.get_right_sensor())
    print(sensors.get_left_sensor())
    print(sensors.get_orientation())
