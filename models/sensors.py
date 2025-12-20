
import numpy as np

"""
This file contains the sensor models for the aircraft.
"""

class SensorModel:
    def __init__(self, sensor_type, bias=0.0, noise_std=0.0):
        self.sensor_type = sensor_type
        self.bias = bias
        self.noise_std = noise_std

    def get_sensor_data(self, x_truth):
        return x_truth + self.bias + np.random.normal(0, self.noise_std)

class InertialMeasurementUnit(SensorModel):
    def __init__(self):
        super().__init__("Inertial Measurement Unit")

    def get_sensor_data(self):
        return self.sensor_data

class GPS(SensorModel):
    def __init__(self):
        super().__init__("GPS")

    def get_sensor_data(self):
        return self.sensor_data

class AirspeedSensor(SensorModel):
    def __init__(self):
        super().__init__("Airspeed Sensor")

    def get_sensor_data(self):
        return self.sensor_data

class BarometricPressureSensor(SensorModel):
    def __init__(self):
        super().__init__("Barometric Pressure Sensor")

    def get_sensor_data(self):
        return self.sensor_data

class Magnetometer(SensorModel):
    def __init__(self):
        super().__init__("Magnetometer")

    def get_sensor_data(self):
        return self.sensor_data # TODO: Add magnetometer data

class Gyroscope(SensorModel):
    def __init__(self, bias, noise_std):
        super().__init__("Gyroscope", bias=bias, noise_std=noise_std)

class Accelerometer:
    def __init__(self, bias=0.0, noise_std=0.0):
        super().__init__("Accelerometer", bias=bias, noise_std=noise_std)

    # def get_sensor_data(self, x):
    #     return x[9:12] + np.random.normal(0, self.gyro_noise_std, 3)

    def get_sensor_data(self):
        return self.sensor_data # TODO: Add accelerometer data
