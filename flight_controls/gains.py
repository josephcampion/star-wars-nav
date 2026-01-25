
####################################################
            #   Lateral Gains
####################################################

#------- Roll Rate Control --------#
kp_p = 4.0
omega_p = 15.0
ki_p = kp_p * omega_p
print(f"kp_p = {kp_p}, ki_p = {ki_p}, omega_p = {omega_p}")
# kd_p = 0.0

#-------- Roll Angle Control ------#
kp_phi = 0.0
ki_phi = 0.0
# kd_phi = 0.0

####################################################
            #   Longitudinal Gains
####################################################

#------- Pitch Rate Control --------#
kp_q = 1.e2
ki_q = 0.0
# kd_q = 0.0

#-------- Pitch Angle Control ------#
kp_theta = 0.0
ki_theta = 0.0
# kd_theta = 0.0

# ------- Altitude Control --------#
kp_h = 0.0
ki_h = 0.0

# ------- Airspeed Control --------#
kp_Va = 0.0
ki_Va = 0.0


####################################################
            #   Directional Gains
####################################################

#-------- Yaw Rate Control --------#
kp_r = 0.0
ki_r = 0.0
# kd_r = 0.0

