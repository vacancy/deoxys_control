#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : debug.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 09/09/2024
#
# Distributed under terms of the MIT license.

import jacinle
from concepts.math.frame_utils_xyzw import mat2posquat
from concepts.math.rotationlib_xyzw import quat2euler

trajectory = jacinle.load('./data/trajectories_testflipping/trajectory_20240906-224245.pkl')

import matplotlib.pyplot as plt

fig, axes = plt.subplots(7, 1, figsize=(20, 20))
for i in range(7):
    q = [log['qpos'][i] for log in trajectory]
    axes[i].plot(q, label=f'qpos_{i}')
    axes[i].set_title(f'Joint {i+1}')
    axes[i].set_ylabel('Position')
    axes[i].legend()

plt.suptitle('Recorded Joint Positions', fontsize=16)
plt.tight_layout(pad=3.0)
plt.show()

ee_pose_list = [mat2posquat(log['ee_pose']) for log in trajectory]
ee_pos_list = [pose[0] for pose in ee_pose_list]
ee_quat_list = [pose[1] for pose in ee_pose_list]
ee_euler_list = [quat2euler(quat) for quat in ee_quat_list]

fig, axes = plt.subplots(6, 1, figsize=(20, 20))
for i, name in enumerate(['x', 'y', 'z']):
    axes[i].plot([ee_pos[i] for ee_pos in ee_pos_list], label=f'ee_{name}')
    axes[i].set_title(f'EE {name}')
    axes[i].set_ylabel('Position')
    axes[i].legend()

for i, name in enumerate(['rx', 'ry', 'rz']):
    axes[i+3].plot([ee_euler[i] for ee_euler in ee_euler_list], label=f'ee_{name}')
    axes[i+3].set_title(f'EE {name}')
    axes[i+3].set_ylabel('Axis Angle')
    axes[i+3].legend()

plt.suptitle('Recorded EE Pose', fontsize=16)
plt.tight_layout(pad=3.0)
plt.show()
