
import numpy as np
import control as ct
import matplotlib.pyplot as plt

sys = ct.ss(A, B, C, D)

sys = ct.ss(
    A, B, C, D, name='sys',
    states=['x1', 'x2'], inputs=['u1', 'u2'], outputs=['y'])

# random state space
sys = ct.rss(states=4, outputs=1, inputs=1, strictly_proper=True)

num = [a0, a1, ..., am]
den = [b0, b1, ..., bn]

sys = ct.tf(num, den)

s = ct.tf('s')

sys = 5 * (s + 10)/(s**2 + 2*s + 1)

val = sys(1 + 0.5j)

# frequency response data
sys = ct.frd(frdata, omega)

frd_sys = ct.frd(sys_lti, omega)

response = ct.frequency_response(sys_lti)
mag, phase, omega = response

# MIMO
sys = ct.tf(
    [[num11, num12], [num21, num22]],
    [[den11, den12], [den21, den22]])


sys = ct.ss(A, B, C, D, inputs=['u1', 'u2'], outputs=['y1', 'y2', 'y3'])

subsys = sys[[0, 2], 0:2]
subsys = sys[['y1', 'y3'], ['u1', 'u2']]

response.magnitude['y[0]', 'u[1]']

mag, phase, omega = response(squeeze=False)

# discrete time systems
sys_ss = ct.ss(A, B, C, D, dt)
sys_tf = ct.tf(num, den, dt)

sys = ct.rss(2, 1, 1, dt=True)
sys.poles()

# state feedback design
K = ct.place(sys.A, sys.B, E)
# E = eigenvalues

# state estimation
L = ct.place(sys.A.T, sys.C.T, E).T

sysnorm = ct.system_norm(sys, p=<val>)

# stability margins
sys = ct.tf(10, [1, 2, 3, 4])
gm, pm, sm, wpc, wgc, wms = ct.stability_margins(sys)
print(f"Gain margin: {gm:2.2} at omega = {wpc:2.2} rad/sec")

# time delays
num, den = ct.pade(0.1, 3)
delay = ct.tf(num, den, name='delay')
print(delay)

# conversions
sys_ss = ct.rss(4, 2, 2, name='sys_ss')
sys_tf = ct.tf(sys_ss, name='sys_tf')

sys_tf = ct.ss2tf(A, B, C, D)

sys_ss = ct.ss(sys_tf)
sys_ss = ct.tf2ss(sys_tf)
sys_ss = ct.tf2ss(num, den)

# time sampling
sys_ct = ct.rss(4, 2, 2, name='sys')
sys_dt = ct.sample_system(sys_ct, 0.1, method='bilinear')
print(sys_dt)

sys_dt = sys_ct.sample(0.1)

sys_ss = ct.rss(4, 1, 1, name='sys_ss')
sys_frd = ct.frd(sys_ss, np.logspace(-1, 1, 5))
print(sys_frd)

sys = ct.rss(4, 2, 2, name='sys_2x2')
print(sys)

sys = ct.rss(2, 1, 1, name='sys_siso')
sys

sys_tf = ct.tf([1, 0], [1, 2, 1], 0.1, name='sys')
print(sys_tf)

