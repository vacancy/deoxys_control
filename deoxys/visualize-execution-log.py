#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : debug.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 09/09/2024
#
# Distributed under terms of the MIT license.

import jacinle

with open('./debug3.log', 'r') as f:
    lines = f.readlines()

"""Example:

JI::q: -0.601745  0.151858  0.283066  -2.13187 -0.940313   2.75886   2.22629
JI::dq:  0.000785933 -8.58571e-05 -0.000227183  0.000702673 -7.55349e-05 -0.000230873 -7.93557e-05
JI::qh: -0.601753  0.151853  0.283063  -2.13188 -0.940328   2.75886    2.2263
JI::err: -8.10775e-06 -4.38375e-06 -3.13342e-06 -5.33543e-06 -1.52992e-05 -6.62919e-06  9.22883e-06
JI::tau: -0.000810775 -0.000438375 -0.000313342 -0.000533543  -0.00114744 -0.000994378  0.000461441
"""
logs = list()
current_log = dict()
for line in lines:
    line = line.strip()
    if line.startswith('JI::q:'):
        current_log = dict()
        current_log['q'] = list(map(float, line.split()[1:]))
    elif line.startswith('JI::dq:'):
        current_log['dq'] = list(map(float, line.split()[1:]))
    elif line.startswith('JI::qh:'):
        current_log['qh'] = list(map(float, line.split()[1:]))
    elif line.startswith('JI::err:'):
        current_log['err'] = list(map(float, line.split()[1:]))
    elif line.startswith('JI::tau:'):
        current_log['tau'] = list(map(float, line.split()[1:]))
        logs.append(current_log)
    elif line.startswith('OSC::q '):
        current_log = dict()
        current_log['q'] = list(map(float, line.split()[1:]))
    elif line.startswith('OSC::dq '):
        current_log['dq'] = list(map(float, line.split()[1:]))
    elif line.startswith('OSC::tau_d '):
        current_log['tau'] = list(map(float, line.split()[1:]))
        logs.append(current_log)
    else:
        print('Invalid line:', line.strip())

# For each joint, plot q and qh.

import matplotlib.pyplot as plt

fig, axes = plt.subplots(14, 1, figsize=(20, 20))
for i in range(7):
    q = [log['q'][i] for log in logs]
    qh = [log['qh'][i] if 'qh' in log else float('nan') for log in logs]
    tau = [log['tau'][i] for log in logs]
    axes[2*i].plot(q, label='q')
    axes[2*i].plot(qh, label='qh')
    axes[2*i].legend()
    axes[2*i].set_title(f'Joint {i+1}')
    axes[2*i].set_ylabel('Position')
    axes[2*i+1].plot(tau, label='tau')
    axes[2*i+1].set_title(f'Torque {i+1}')
    axes[2*i+1].set_ylabel('Torque')
    axes[2*i+1].legend()
plt.tight_layout()
plt.show()
