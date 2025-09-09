# -*- coding: utf-8 -*-
"""
Created on Thu Sep  4 07:30:00 2025

@author: 84091
"""
import matplotlib.pyplot as plt
from matplotlib import font_manager
import pandas as pd


def read_ite_gene(filename):
    df1 = pd.DataFrame([["TNM_TAP", 0, 100000000, 1.0, 1, 0]], columns=["ObjID","Iter", "OFV", "StepSize", "ConvIndc", "Time"] )
    efficient_line = []
    with open(filename, 'r') as f1:
        l1 = f1.readlines()
        for i in range(len(l1)):
            if l1[i][0]=='-':
                efficient_line.append(i)
        l1 = l1[efficient_line[0]+1 : efficient_line[1]]
    df = pd.read_table(filename,sep='\\s+', skiprows=efficient_line[0]+1, nrows=efficient_line[1]-efficient_line[0]-2)
    df1 = pd.concat([df1, df], ignore_index=True)
    return (df1)


base_path = './raw_results/sens_congestion/'
plot_path='./figures/sens_congestion/'
demandratios = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]

csSBAconstant = {}
for k, i in enumerate(demandratios):
    csSBAconstant[i] = read_ite_gene(base_path + "chicagosketch_SBA_constant_toll_%s.ite"%i)
    
csSBAchange = {}
for k, i in enumerate(demandratios):
    csSBAchange[i] = read_ite_gene(base_path + "chicagosketch_SBA_change_toll_%s.ite"%i)



def draw_congestion_converge(data, x_length, scenario='constant'):
    fig = plt.figure(figsize=(8, 6))
    ax1 = fig.add_subplot(111)
    ax1.set_yscale('log')
    colors = ['mediumblue', 'dodgerblue', 'steelblue', 'forestgreen', 'limegreen', 'gold', 'orange','coral' ,'red', 'brown']
    linestyles=[':', ':', ':', ':', '-.', '-', '-', '-', '-', '-']
    linewidths = [1.5, 2, 3, 4, 3, 1.5, 2, 2.5, 3, 3.5]
    for k, i in enumerate(demandratios):
        ax1.plot(data[i]["Time"][0:], data[i]["ConvIndc"][0:],label="%s%%"%(int(i*100)),color = colors[k], linestyle=linestyles[k], linewidth=linewidths[k])
    font1 = {'family':'Times New Roman', 'weight':'bold', 'size': 24}
    ax1.set_xlabel('CPU Time (s)', fontdict=font1)
    ax1.set_xlim(-0.02 * x_length, x_length)
    ax1.set_ylim(6 * pow(10, -11), 2)
    ax1.set_ylabel('Relative Gap', fontdict=font1)
    ax1.set_yticks([pow(10, -10), pow(10, -8), pow(10, -6), pow(10, -4), pow(10, -2), pow(10, 0)])
    ticks_font = font_manager.FontProperties(family='STIXGeneral', style='normal',
        size=20, weight='bold', stretch='normal')
    for label in ax1.get_xticklabels():
        label.set_fontproperties(ticks_font)
    for label in ax1.get_yticklabels():
        label.set_fontproperties(ticks_font)
    ax1.legend(loc=1,prop={'family' : 'STIXGeneral', 'size' : 20,
                           'weight':'bold'}, ncol=2)
    ax1.grid(color='darkgrey', linestyle=':', linewidth=1)
    plt.tight_layout()
    fig.savefig(plot_path + 'demand_%s.pdf'%scenario, dpi=900, bbox_inches='tight')
    plt.show()

# Figure 9(a)
draw_congestion_converge(csSBAconstant, 800, scenario='constant')

# Figure 9(b)
draw_congestion_converge(csSBAchange, 2000, scenario='change')