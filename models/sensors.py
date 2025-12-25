
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

class Gyroscope(SensorModel):
    def __init__(self, bias, noise_std):
        super().__init__("Gyroscope", bias=bias, noise_std=noise_std)

class Accelerometer(SensorModel):
    def __init__(self, bias=0.0, noise_std=0.0):
        super().__init__("Accelerometer", bias=bias, noise_std=noise_std)

class InertialMeasurementUnit:
    def __init__(self, gyro_bias=0.0, gyro_noise_std=0.0, accel_bias=0.0, accel_noise_std=0.0):
        self._gyros = [
            Gyroscope(gyro_bias, gyro_noise_std),
            Gyroscope(gyro_bias, gyro_noise_std),
            Gyroscope(gyro_bias, gyro_noise_std)
            ]
        self._accels = [
            Accelerometer(accel_bias, accel_noise_std),
            Accelerometer(accel_bias, accel_noise_std),
            Accelerometer(accel_bias, accel_noise_std)
            ]

    def get_ang_vel_meas(self, omega_truth):
        return np.array([self._gyros[i].get_sensor_data(omega_truth[i]) for i in range(3)])

    def get_sens_accel_meas(self, sens_accel_truth):
        return np.array([self._accels[i].get_sensor_data(sens_accel_truth[i]) for i in range(3)])

class GPS():
    def __init__(self, noise_std_n, noise_std_e, noise_std_d, tau_gps, T_samp):
        self._noise_std_n = noise_std_n
        self._noise_std_e = noise_std_e
        self._noise_std_d = noise_std_d
        self._tau_gps = tau_gps
        self._T_samp = T_samp
        self._gps_error_n = 0.0
        self._gps_error_e = 0.0
        self._gps_error_d = 0.0

    def update_gps_error(self):
        tau_gps = self._tau_gps
        Ts = self._T_samp
        k_decay = np.exp(-tau_gps * Ts)

        self._gps_error_n = k_decay * self._gps_error_n + np.random.normal(0, self._noise_std_n)
        self._gps_error_e = k_decay * self._gps_error_e + np.random.normal(0, self._noise_std_e)
        self._gps_error_d = k_decay * self._gps_error_d + np.random.normal(0, self._noise_std_d)

    def get_gps_error(self):
        return np.array([
            self._gps_error_n,
            self._gps_error_e,
            self._gps_error_d
        ])

    def get_gps_data(self, pos_truth):
        # self.update_gps_error()
        return pos_truth + self.get_gps_error()
 

############### TODO: Implement and test these #######################


#     def get_sensor_data(self):
#         return self.sensor_data

# class AirspeedSensor(SensorModel):
#     def __init__(self):
#         super().__init__("Airspeed Sensor")

#     def get_sensor_data(self):
#         return self.sensor_data

# class BarometricPressureSensor(SensorModel):
#     def __init__(self):
#         super().__init__("Barometric Pressure Sensor")

#     def get_sensor_data(self):
#         return self.sensor_data

# class Magnetometer(SensorModel):
#     def __init__(self):
#         super().__init__("Magnetometer")

#     def get_sensor_data(self):
#         return self.sensor_data # TODO: Add magnetometer data
