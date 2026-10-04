"""Render archived raw maps and recorded map-frame trajectories; no ROS/API."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
import numpy as np

ROOT = Path(__file__).resolve().parent
TASKS = ['7e879dbb-75cc-4537-b832-736939afacf9',
         'c66ed4f5-a8ed-4b1a-a1ac-917cfbf1beda']


def read_map(name):
    with np.load(ROOT/'memory-maps'/name) as data:
        cells = data['data'].copy()
        resolution = float(data['resolution'])
        origin = data['origin'].copy()
    if abs(origin[2]) > 1e-9:
        raise ValueError('This figure requires axis-aligned archived maps')
    classes = np.zeros(cells.shape, dtype=np.uint8)
    classes[(cells >= 0) & (cells < 25)] = 1
    classes[cells >= 65] = 2
    classes[(cells >= 25) & (cells < 65)] = 3
    extent = [origin[0], origin[0]+cells.shape[1]*resolution,
              origin[1], origin[1]+cells.shape[0]*resolution]
    return classes, extent, np.count_nonzero(cells >= 0)*resolution**2


def main():
    summary = json.loads((ROOT/'evidence/memory-trial-summary.json').read_text())
    maps = [read_map(TASKS[0]+'.detail.gain_before.npz'),
            read_map(TASKS[0]+'.detail.gain_settled.npz'),
            read_map(TASKS[1]+'.detail.gain_settled.npz')]
    traces = [json.loads((ROOT/'memory-maps'/(task+'.detail.trace.json')).read_text())
              for task in TASKS]
    colors = ['#687d85', '#f1f3f4', '#172a38', '#e7ac55']
    routes = ['#df6b30', '#285ece']
    fig, axes = plt.subplots(1, 3, figsize=(15, 7), layout=None)
    fig.patch.set_facecolor('#fafbfc')
    titles = ['Before this trial', 'After step 1', 'After step 2']
    bounds = [min(m[1][0] for m in maps), max(m[1][1] for m in maps),
              min(m[1][2] for m in maps), max(m[1][3] for m in maps)]
    for i, (ax, (cells, extent, known)) in enumerate(zip(axes, maps)):
        ax.imshow(cells, cmap=ListedColormap(colors), vmin=0, vmax=3,
                  origin='lower', extent=extent, interpolation='nearest')
        ax.set_xlim(bounds[:2]); ax.set_ylim(bounds[2:]); ax.set_aspect('equal')
        for step in range(i):
            xy = np.array([point['pose'][:2] for point in traces[step]])
            ax.plot(xy[:, 0], xy[:, 1], color=routes[step], linewidth=2.4)
            end = summary['steps'][step]['pose_after']
            ax.scatter(end[0], end[1], s=80, color=routes[step], edgecolor='white', zorder=4)
            ax.annotate(str(step+1), end[:2], xytext=(0, 9), textcoords='offset points',
                        ha='center', weight='bold', color=routes[step])
        start = summary['steps'][0]['pose_before']
        ax.scatter(start[0], start[1], marker='*', s=140, color='#d94a58', edgecolor='white', zorder=4)
        ax.set_title(f'{titles[i]}\nKnown area: {known:.2f} m²', fontsize=12, pad=12)
        ax.set_xlabel('Map x (m)'); ax.set_ylabel('Map y (m)' if i == 0 else '')
        ax.tick_params(labelsize=9)
    fig.suptitle('Bounded AI exploration | memory mode | 2026-10-04', fontsize=18, weight='bold', y=.98)
    fig.legend([Patch(facecolor=c) for c in colors], ['Unknown', 'Free', 'Occupied', 'Uncertain'],
               loc='upper center', bbox_to_anchor=(.5, .91), ncol=4, frameon=False)
    fig.text(.5, .16, '2 goals succeeded  ·  4.03 m travelled  ·  Minimum LiDAR 2.51 m  ·  Stop confirmed',
             ha='center', fontsize=12, weight='bold')
    fig.text(.5, .105, 'Net known area +31.43 m². Step 2 uses fractional-grid registration and is excluded from gain trends.',
             ha='center', fontsize=10)
    fig.text(.5, .06, 'Raw map snapshots and recorded map-frame trajectories. Not world coverage or a paired strategy comparison.',
             ha='center', fontsize=10, color='#4b5660')
    fig.subplots_adjust(left=.05, right=.985, top=.80, bottom=.25, wspace=.16)
    fig.savefig(ROOT/'images/memory-map-progression.png', dpi=170, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == '__main__':
    main()
