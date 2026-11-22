#!/usr/bin/env python3
"""Genera las graficas y metricas a partir de los CSV de ~/ackermann_results.

Uso:
    python3 plot_results.py                      # usa el run_*.csv mas reciente
    python3 plot_results.py run_infinity_XXX.csv # un archivo concreto
Dependencias: pandas, numpy, matplotlib (sudo apt install python3-pandas python3-matplotlib)
"""
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REAR_OFFSET = 0.12   # eje trasero -> origen del modelo (m)
RES = os.path.expanduser('~/ackermann_results')

run_file = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.join(RES, 'run_*.csv')))[-1]
if not os.path.isabs(run_file) and not os.path.exists(run_file):
    run_file = os.path.join(RES, run_file)
path_file = run_file.replace('run_', 'path_')
tag = os.path.splitext(os.path.basename(run_file))[0].replace('run_', '')
out = os.path.join(RES, 'figures')
os.makedirs(out, exist_ok=True)

d = pd.read_csv(run_file)
p = pd.read_csv(path_file)
P = p[['x', 'y']].to_numpy()

# --- error transversal exacto: distancia del eje trasero al segmento de trayectoria mas cercano
rx = d.x - REAR_OFFSET * np.cos(d.yaw)
ry = d.y - REAR_OFFSET * np.sin(d.yaw)
A, B = P[:-1], P[1:]
AB = B - A
den = (AB ** 2).sum(axis=1)
cte = np.empty(len(d))
for i, (px, py) in enumerate(zip(rx, ry)):
    q = np.array([px, py])
    t = np.clip(((q - A) * AB).sum(axis=1) / den, 0, 1)
    proj = A + t[:, None] * AB
    cte[i] = np.min(np.hypot(*(q - proj).T))
d['cte_exact'] = cte

rmse = float(np.sqrt((cte ** 2).mean()))
print(f'Archivo        : {os.path.basename(run_file)}')
print(f'Duracion       : {d.t.iloc[-1]:.1f} s')
print(f'RMSE cte       : {rmse:.3f} m')
print(f'cte medio      : {cte.mean():.3f} m')
print(f'cte maximo     : {cte.max():.3f} m  (t = {d.t.iloc[int(np.argmax(cte))]:.1f} s)')
print(f'delta maximo   : {np.degrees(d.delta_est.abs().max()):.1f} grados')
print(f'Fin - inicio   : {np.hypot(d.x.iloc[-1] - d.x.iloc[0], d.y.iloc[-1] - d.y.iloc[0]):.3f} m')

plt.rcParams.update({'font.size': 10, 'figure.dpi': 150})

# 1) trayectoria real vs deseada
fig, ax = plt.subplots(figsize=(7, 3.6))
ax.plot(p.x, p.y, 'k--', lw=1.5, label='Desired path')
ax.plot(rx, ry, 'r-', lw=1.2, label='Actual (rear axle)')
ax.plot(rx.iloc[0], ry.iloc[0], 'go', label='Start')
ax.set_aspect('equal'); ax.grid(alpha=.3)
ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.legend(loc='best')
fig.tight_layout(); fig.savefig(os.path.join(out, f'trajectory_{tag}.png')); plt.close(fig)

# 2) error transversal vs tiempo
fig, ax = plt.subplots(figsize=(7, 2.8))
ax.plot(d.t, cte * 100, 'b-', lw=1)
ax.axhline(rmse * 100, color='r', ls='--', label=f'RMSE = {rmse * 100:.1f} cm')
ax.set_xlabel('t [s]'); ax.set_ylabel('Cross-track error [cm]'); ax.grid(alpha=.3); ax.legend()
fig.tight_layout(); fig.savefig(os.path.join(out, f'cte_{tag}.png')); plt.close(fig)

# 3) angulo de direccion vs tiempo
fig, ax = plt.subplots(figsize=(7, 2.8))
ax.plot(d.t, np.degrees(d.delta_est), 'g-', lw=1)
lim = np.degrees(d.delta_est.abs().max())
ax.axhline(lim, color='gray', ls=':'); ax.axhline(-lim, color='gray', ls=':')
ax.set_xlabel('t [s]'); ax.set_ylabel('Steering angle [deg]'); ax.grid(alpha=.3)
fig.tight_layout(); fig.savefig(os.path.join(out, f'steering_{tag}.png')); plt.close(fig)

print(f'Figuras en     : {out}')
