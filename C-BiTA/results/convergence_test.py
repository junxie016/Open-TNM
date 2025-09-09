# -*- coding: utf-8 -*-
"""
Created on Thu Sep  4 01:39:04 2025

@author: 84091
"""

import matplotlib.pyplot as plt
from matplotlib import font_manager
import pandas as pd

colormaplist = [ '#fd8d49', '#ec2d01',  '#2242c7']

def read_ite_gene2(filename):
    df1 = pd.DataFrame([["TNM_TAP", 0, 100000000, 1.0, 1, 0]], columns=["ObjID","Iter", "OFV", "StepSize", "ConvIndc", "Time"] )
    efficient_line = []
    with open(filename, 'r') as f1:
        l1 = f1.readlines()
        for i in range(len(l1)):
            if l1[i][0]=='-':
                efficient_line.append(i)
        l1 = l1[efficient_line[0]+1 : efficient_line[1]]
    df = pd.read_table(filename,sep='\s+', skiprows=efficient_line[0]+1, nrows=efficient_line[1]-efficient_line[0]-2)
    df1 = pd.concat([df1, df], ignore_index=True)
    return (df1)

def read_ite_gene(filename):
    df1 = pd.read_table(filename,sep='\s+')
    return (df1)

def draw_convergence(SBAdata, T2data, xlength, net_name, plot_path='./figures/convergence_test/'):
    font1 = {'family':'Times New Roman','weight':'heavy','size': 23}
    ticks_font = font_manager.FontProperties(family='STIXGeneral', style='normal',
                                             size=20, weight='bold', stretch='normal')
    
    fig = plt.figure(figsize=(8, 5))
    ax1 = fig.add_subplot(111)
    ax1.set_yscale('log')
    ax1.plot(T2data["Time"][0:], T2data["ConvIndc"][0:],label='FW',color = colormaplist[0], linestyle='-', linewidth=2)
    ax1.plot(SBAdata["Time"][0:], SBAdata["ConvIndc"][0:],label='SBA',  color = colormaplist[1], linestyle=':',marker='o', linewidth=3)
    
    ax1.set_xlabel('CPU Time (s)',fontdict=font1)
    ax1.set_ylabel('Relative Gap', fontdict=font1)
    ax1.set_xlim(-0.02 * xlength, xlength)
    ax1.set_ylim(pow(10, -13),2)
    ax1.set_yticks([pow(10, -12), pow(10, -10), pow(10, -8), pow(10, -6), pow(10, -4), pow(10, -2), pow(10, 0)])
    
    for label in ax1.get_xticklabels():
        label.set_fontproperties(ticks_font)
    for label in ax1.get_yticklabels():
        label.set_fontproperties(ticks_font)
    ax1.legend(loc=1, prop={'family' : 'STIXGeneral', 'size': 20,'weight':'bold'})
    ax1.grid(color='darkgrey', linestyle=':', linewidth=1)
    plt.tight_layout()
    fig.savefig(plot_path + net_name + '_conv.pdf',
                dpi=900,
                bbox_inches='tight')
    plt.show()


# # Plot convergence test results:
base_path = './raw_results/convergence_test/'
convergence_data = {}
networks = ['sf', 'Anaheim', 'Barcelona', 'Winnipeg', 'chicagosketch','chi']
x_lengths = [7, 15, 52, 250, 1550, 78000]
for i, nt in enumerate(networks):
    convergence_data[nt] = {}
    convergence_data[nt]['x_length'] = x_lengths[i]
    try:
        convergence_data[nt]['SBA'] = read_ite_gene(base_path + nt + "_SBA.ite")
    except:
        convergence_data[nt]['SBA'] = read_ite_gene2(base_path + nt + "_SBA.ite")
    try:
        convergence_data[nt]['T2'] = read_ite_gene(base_path + nt + "_T2theta.ite")
    except:
        convergence_data[nt]['T2'] = read_ite_gene2(base_path + nt + "_T2theta.ite")
    draw_convergence(convergence_data[nt]['SBA'], 
                      convergence_data[nt]['T2'],
                      convergence_data[nt]['x_length'],
                      nt)