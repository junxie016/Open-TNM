# -*- coding: utf-8 -*-
"""
Created on Thu Sep  4 02:48:39 2025

@author: 84091
"""

import matplotlib.pyplot as plt
from matplotlib import font_manager
import pandas as pd
import seaborn as sns

def read_ite_gene(filename):
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


colormaplist=sns.color_palette("Reds", len([0, 2, 6, 12, 20, 30, 45, 70, 100, 160]))
linestyles = ['-', '--', '-.', ':']

base_path = './raw_results/num_of_tolled_links/'
plot_path='./figures/num_of_tolled_links/'


# Plot Figure 6(a)
csFW0 = read_ite_gene(base_path + "chicagosketch_FW.ite")
csFW1  = read_ite_gene(base_path + "chicagosketch_T2theta_1.ite")
csFW18 = read_ite_gene(base_path + "chicagosketch_T2theta_18.ite")
csFWUE = read_ite_gene(base_path + "chicagosketch_T2theta_UE.ite")
x_length = 1050
fig = plt.figure(figsize=(8,6))
ax1 = fig.add_subplot(111)
ax1.set_yscale('log')
ax1.plot(csFW0["Iter"][0:], csFW0["ConvIndc"][0:],label='No toll',color =colormaplist[2], linestyle=linestyles[0], linewidth=2)
ax1.plot(csFW1["Iter"][0:], csFW1["ConvIndc"][0:],label='Toll scheme 1',color =colormaplist[4], linestyle=linestyles[1], linewidth=2)
ax1.plot(csFW18["Iter"][0:], csFW18["ConvIndc"][0:],label='Toll scheme 2',color =colormaplist[6], linestyle=linestyles[2], linewidth=2)
ax1.plot(csFWUE["Iter"][0:], csFWUE["ConvIndc"][0:],label='Toll scheme 3',color =colormaplist[9], linestyle=linestyles[3], linewidth=2)
font1 = {'family':'Times New Roman','weight':'bold','size': 24}
font2 = {'family':'Times New Roman','weight':'normal','size': 20}
ax1.set_xlabel('Iteration', fontdict=font1)
ax1.set_ylabel('Relative Gap', fontdict=font1)
ax1.set_xlim(-0.02 * x_length, x_length)
ax1.set_ylim(pow(10, -13),2)
ax1.set_yticks([pow(10, -12), pow(10, -10), pow(10, -8), pow(10, -6), pow(10, -4), pow(10, -2), pow(10, 0)])
ticks_font = font_manager.FontProperties(family='STIXGeneral', style='normal',
    size=20, weight='bold', stretch='normal')
for label in ax1.get_xticklabels():
    label.set_fontproperties(ticks_font)
for label in ax1.get_yticklabels():
    label.set_fontproperties(ticks_font)
ax1.legend(loc=1,prop={'family' : 'STIXGeneral', 'size' : 20,'weight':'bold'})
ax1.grid(color='darkgrey',linestyle=':',linewidth=1)
ax1.text(400, 3*pow(10,-13),
         "Time to iter 1000:\n  No toll : %.2f s \n  Toll scheme 1: %.2f s \n  Toll scheme 2: %.2f s \n  Toll scheme 3: %.2f s \n"%(csFW0["Time"][1001], csFW1["Time"][1001], csFW18["Time"][1001], csFWUE["Time"][1001]), 
         fontdict=font2)
plt.tight_layout()
fig.savefig(plot_path + 'FW.pdf',
            dpi=900,
            bbox_inches='tight')
plt.show()


# Plot Figure 6(b)
csSBA0 = read_ite_gene(base_path + "chicagosketch_GPG.ite")
csSBA1  = read_ite_gene(base_path + "chicagosketch_SBA_1.ite")
csSBA18 = read_ite_gene(base_path + "chicagosketch_SBA_18.ite")
csSBAUE = read_ite_gene(base_path + "chicagosketch_SBA_UE.ite")

fig = plt.figure(figsize=(8,6))
ax1 = fig.add_subplot(111)
ax1.set_yscale('log')
ax1.plot(csSBA0["Iter"][0:], csSBA0["ConvIndc"][0:],label='No toll',color =colormaplist[2], linestyle=linestyles[0], marker='^', linewidth=2)
ax1.plot(csSBA1["Iter"][0:], csSBA1["ConvIndc"][0:],label='Toll scheme 1',color =colormaplist[4], linestyle=linestyles[1], marker='+', linewidth=2)
ax1.plot(csSBA18["Iter"][0:], csSBA18["ConvIndc"][0:],label='Toll scheme 2',color =colormaplist[6], linestyle=linestyles[2], marker='o',linewidth=2)
ax1.plot(csSBAUE["Iter"][0:], csSBAUE["ConvIndc"][0:],label='Toll scheme 3',color =colormaplist[9], linestyle=linestyles[3], marker='x', linewidth=2)
font1 = {'family':'Times New Roman','weight':'bold','size': 24}
font2 = {'family':'Times New Roman','weight':'normal','size': 20}
ax1.set_xlabel('Iteration',fontdict=font1)
ax1.set_ylabel('Relative Gap', fontdict=font1)
x_length = 35
ax1.set_xlim(-0.02 * x_length, x_length)
ax1.set_ylim(pow(10, -13), 2)
ax1.set_yticks([pow(10, -12), pow(10, -10), pow(10, -8), pow(10, -6), pow(10, -4), pow(10, -2), pow(10, 0)])
ticks_font = font_manager.FontProperties(family='STIXGeneral', style='normal',
    size=20, weight='bold', stretch='normal')
for label in ax1.get_xticklabels():
    label.set_fontproperties(ticks_font)
for label in ax1.get_yticklabels():
    label.set_fontproperties(ticks_font)
ax1.legend(loc=1,prop={'family' : 'STIXGeneral', 'size' : 16,'weight':'bold'})
ax1.grid(color='darkgrey',linestyle=':',linewidth=1)
ax1.text(20, 2*pow(10,-9),"Time to iter 5:\n  No toll : %.2f s \n  Toll scheme 1: %.2f s \n  Toll scheme 2: %.2f s \n  Toll scheme 3: %.2f s \n"%(csSBA0["Time"][6], csSBA1["Time"][6], csSBA18["Time"][6], csSBAUE["Time"][6]), fontdict=font2)
plt.tight_layout()
fig.savefig(plot_path + 'SBA.pdf',
            dpi=900,
            bbox_inches='tight')
plt.show()
