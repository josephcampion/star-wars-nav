
####################################################
            #   Lateral Gains
####################################################

#------- Roll Rate Control --------#
kp_p = 4.0
omega_p = 15.0
ki_p = kp_p * omega_p
# kd_p = 0.0

#-------- Roll Angle Control ------#
kp_phi = 0.4
ki_phi = 10.0
# kd_phi = 0.0

####################################################
            #   Longitudinal Gains
####################################################

#------- Pitch Rate Control --------#
kp_q = 0.2
omega_q = 2.0
ki_q = kp_q * omega_q
# kd_q = 0.0

#-------- Pitch Angle Control ------#
kp_theta = 0.4
omega_theta = 10.0
ki_theta = kp_theta * omega_theta
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

if __name__ == "__main__":
    print(f"kp_p = {kp_p}, ki_p = {ki_p}, omega_p = {omega_p}")
    print(f"kp_phi = {kp_phi}, ki_phi = {ki_phi}, omega_phi = {omega_phi}")
    print(f"kp_q = {kp_q}, ki_q = {ki_q}, omega_q = {omega_q}")
    print(f"kp_theta = {kp_theta}, ki_theta = {ki_theta}, omega_theta = {omega_theta}")
    # print(f"kp_h = {kp_h}, ki_h = {ki_h}")
    # print(f"kp_Va = {kp_Va}, ki_Va = {ki_Va}")
    # print(f"kp_r = {kp_r}, ki_r = {ki_r}")