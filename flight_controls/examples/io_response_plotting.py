
import numpy as np
import control as ct
import matplotlib.pyplot as plt

bode_plot(sys)
nyquist_plot([sys1, sys2])
phase_plane_plot(sys, limits)
pole_zero_plot(sys)
root_locus_plot(sys)

response = ct.nyquist_response([sys1, sys2])
count = ct.response.count          # number of encirclements of -1
cplt = ct.nyquist_plot(response)   # Nyquist plot

step_response(sys).plot()
frequency_response(sys).plot()
nyquist_response(sys).plot()
pp.streamlines(sys, limits).plot()
root_locus_map(sys).plot()

T = [t1,     t2,     t3,     ..., tn    ]

