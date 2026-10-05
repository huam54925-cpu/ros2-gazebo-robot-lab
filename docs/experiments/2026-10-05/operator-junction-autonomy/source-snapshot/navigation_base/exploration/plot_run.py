#!/usr/bin/env python3
"""Render stored exploration evidence offline; never feeds the exploration policy."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D


def main():
    parser=argparse.ArgumentParser();parser.add_argument('report');args=parser.parse_args();path=Path(args.report)
    report=json.loads(path.read_text());trace=json.loads(path.with_suffix('.trace.json').read_text())
    fig,axes=plt.subplots(1,2,figsize=(14,7))
    for ax,label in zip(axes,['initial','final']):
        m=np.load(path.with_suffix('.'+label+'.npz'));data=m['data'];resolution=float(m['resolution']);ox,oy,angle=m['origin']
        pixels=np.full(data.shape,.60);pixels[(data>=0)&(data<25)]=1.;pixels[data>=65]=0.
        transform=Affine2D().rotate(angle).translate(ox,oy)
        ax.imshow(pixels,origin='lower',cmap='gray',vmin=0,vmax=1,extent=[0,data.shape[1]*resolution,0,data.shape[0]*resolution],transform=transform+ax.transData)
        corners=transform.transform([[0,0],[0,data.shape[0]*resolution],[data.shape[1]*resolution,0],[data.shape[1]*resolution,data.shape[0]*resolution]])
        ax.set_xlim(corners[:,0].min(),corners[:,0].max());ax.set_ylim(corners[:,1].min(),corners[:,1].max())
        ax.set_aspect('equal');ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)')
        ax.set_title(f'{label.title()} map: {int((data>=0).sum()):,} known cells')
    if trace:
        points=np.array([s['pose'][:2] for s in trace]);axes[1].plot(points[:,0],points[:,1],color='#1677cc',linewidth=1.6,label='SLAM-frame trajectory')
    for i,goal in enumerate(report['goals'],1):
        c=goal['candidate'];ok=goal['result']['status']==4;color='#15935d' if ok else '#c63440'
        axes[1].scatter(c['x'],c['y'],marker='*' if ok else 'x',s=95,color=color)
        axes[1].annotate(str(i),(c['x'],c['y']),xytext=(5,5),textcoords='offset points')
    axes[1].legend(loc='best')
    fig.suptitle('Map-only Frontier baseline — no preset waypoints\n'+f"Stop: {report['stop_reason']} | goals: {report['successful_goals']} succeeded / {report['failed_goals']} failed",fontsize=13)
    fig.tight_layout();fig.savefig(path.with_suffix('.overview.png'),dpi=140)
    samples=[report['initial']]+trace+[report['final']]
    fig,ax=plt.subplots(figsize=(10,4));ax.plot([s['wall_elapsed'] for s in samples],[s['known_area_m2'] for s in samples],color='#1677cc')
    ax.set(xlabel='Wall time (s)',ylabel='Known-cell area (m²)',title='Map growth (not coverage percentage)');ax.grid(alpha=.25);fig.tight_layout();fig.savefig(path.with_suffix('.growth.png'),dpi=140)


if __name__=='__main__':main()
