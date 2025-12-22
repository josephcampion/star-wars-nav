
import numpy as np
import control as ct
import matplotlib.pyplot as plt

####################################################
    # Define the parameters for the system
####################################################

m, c, k = 1, 0.1, 2
# Create a linear system
A = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [-2*k/m, k/m, -c/m, 0],
    [k/m, -2*k/m, 0, -c/m]
])
B = np.array([[0], [0], [0], [k/m]])
C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
D = 0

sys = ct.ss(A, B, C, D, outputs=['q1', 'q2'], name="coupled spring mass")
print(sys)

####################################################
                # Time Responses
####################################################

response = ct.initial_response(sys, X0=[1, 0, 0, 0])
print(dir(response))
# cplt = response.plot()

# Plot the outputs of the system on the same graph, in different colors
t = response.time
x = response.states
plt.plot(t, x[0], 'b', t, x[1], 'r')
plt.legend(['$x_1$', '$x_2$'])
plt.xlim(0, 50)
plt.ylabel('States')
plt.xlabel('Time [s]')
plt.title("Initial response from $x_1 = 1$, $x_2 = 0$");

plt.grid(True)
plt.show()

####################################################
                # Time Responses
####################################################

for X0 in [[1, 0, 0, 0], [0, 2, 0, 0], [1, 2, 0, 0], [0, 0, 1, 0], [0, 0, 2, 0]]:
  response = ct.initial_response(sys, T=20, X0=X0)
#   response.plot(label=f"{X0=}")

# TODO: Plot all these

####################################################
            # Print Step Info
####################################################

step_info = ct.step_info(sys)
print("Input 0, output 0 rise time = ",
      step_info[0][0]['RiseTime'], "seconds\n")
step_info

"""
[[{'RiseTime': 0.6153902252990775,
   'SettlingTime': 89.02645259326653,
   'SettlingMin': -0.13272845655369417,
   'SettlingMax': 0.9005994876222034,
   'Overshoot': 170.17984628666102,
   'Undershoot': 39.81853696610825,
   'Peak': 0.9005994876222034,
   'PeakTime': 2.3589958636464634,
   'SteadyStateValue': 0.33333333333333337}],
 [{'RiseTime': 0.6153902252990775,
   'SettlingTime': 73.6416969607896,
   'SettlingMin': 0.2276019820782241,
   'SettlingMax': 1.13389337710215,
   'Overshoot': 70.08400656532254,
   'Undershoot': 0.0,
   'Peak': 1.13389337710215,
   'PeakTime': 6.564162403190159,
   'SteadyStateValue': 0.6666666666666665}]]
"""

####################################################
                # Force Response
####################################################

T = np.linspace(0, 50, 500)
U1 = np.cos(T)
U2 = np.sin(3 * T)

resp1 = ct.forced_response(sys, T, U1)
resp2 = ct.forced_response(sys, T, U2)
resp3 = ct.forced_response(sys, T, U1 + U2)

# Plot the individual responses
resp1.sysname = 'U1'; resp1.plot(color='b')
resp2.sysname = 'U2'; resp2.plot(color='g')
resp3.sysname = 'U1 + U2'; resp3.plot(color='r');

# Show that the system response is linear
cplt = resp3.plot(label="G(U1 + U2)")
cplt.axes[0, 0].plot(resp1.time, resp1.outputs[0] + resp2.outputs[0], 'k--', label="G(U1) + G(U2)")
cplt.axes[1, 0].plot(resp1.time, resp1.outputs[1] + resp2.outputs[1], 'k--')
cplt.axes[2, 0].plot(resp1.time, resp1.inputs[0] + resp2.inputs[0], 'k--')
cplt.axes[0, 0].legend(loc='upper right', fontsize='x-small');


####################################################
    # State Space to Transfer Function
####################################################

try:
    G = ct.ss2tf(sys, name='u to q1, q2')
except ct.ControlMIMONotImplemented:
    # Create SISO transfer functions, in case we don't have slycot
    G = ct.ss2tf(sys[0, 0], name='u to q1')
print(G)

# Gain and phase for the simulation above
from math import pi
val = G(1.35j)
print(f"{G(1.35j)=}")
print(f"Gain: {np.absolute(val)}")
print(f"Phase: {np.angle(val)}", " (", np.angle(val) * 180/pi, "deg)")

####################################################
            # Freq. Resp. and Bode
####################################################

freqresp = ct.frequency_response(sys)
cplt = freqresp.plot()

cplt = ct.bode_plot(sys, overlay_outputs=True)

