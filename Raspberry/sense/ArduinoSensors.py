from serial import Serial
from threading import Thread
from time import sleep

SERIAL_PORT = '/dev/ttyUSB0'  # must be changed with a different configuration
# SERIAL_PORT = '/dev/tty.usbserial-140'


class ArduinoSensors:
    # ultrasonic sensors
    _right_sensor: float
    _left_sensor: float
    _serial: Serial
    # compass sensors
    _orientation: float

    def __init__(self):
        self._right_sensor = 0
        self._left_sensor = 0
        self._orientation = 0
        self._serial = Serial(SERIAL_PORT, 9600)
        Thread(target=self._update).start()  # Avvia l'esecuzione asincrona in background

    def _set_values(self, right_sensor, left_sensor, orientation):
        self._right_sensor = right_sensor
        self._left_sensor = left_sensor
        self._orientation = orientation

    def _update(self):
        # line read
        byte = ""
        while byte != b"\n" :
            byte = self._serial.read(1)
        while True:
            line = self._serial.readline()
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
        print("right: ",sensors.get_right_sensor())
        print("left: ",sensors.get_left_sensor())
        print("orientation: ",sensors.get_orientation())
        sleep(1)
