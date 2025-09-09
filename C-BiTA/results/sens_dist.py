# -*- coding: utf-8 -*-
"""
Created on Thu Sep  4 07:54:55 2025

@author: 84091
"""

import matplotlib.pyplot as plt
from matplotlib import font_manager
import pandas as pd
import numpy as np

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


base_path = './raw_results/sens_dist/'
plot_path='./figures/sens_dist/'
selected_params = [-0.9, -0.5, -0.1, 0, 1, 5, 9]


# Figure 10(a)
fig = plt.figure(figsize=(8, 7))
ax1 = fig.add_subplot(111)
ax1.set_yscale('log')
linewidths = [3, 2, 1, 2, 1, 2, 3]
colors = ['mediumblue', 'dodgerblue', 'forestgreen', 'limegreen', 'gold', 'orange', 'red']
for k,i in enumerate(selected_params):
    df = read_ite_gene(base_path + "chicagosketch_SBA_dist_%s.ite"%i)
    if (i < 0):
        line = '--'
    elif ((i > 0) and (i < 10)):
        line = '-'
    elif ((i > 0) and (i >= 10)):
        line = '-'
    else:
        line='-.'
    ax1.plot(df["Time"][0:], df["ConvIndc"][0:],label=r'$\rho$=%s'%i, linestyle=line, color = colors[k], linewidth = linewidths[k])
font1 = {'family':'Times New Roman','weight':'bold','size': 24}
ax1.set_xlabel('CPU Time (s)',fontdict=font1)
ax1.set_ylabel('Relative Gap', fontdict=font1)
x_length = 400
ax1.set_xlim(-0.02 * x_length, x_length)
ax1.set_ylim(pow(10, -11), 2)
ax1.set_yticks([pow(10, -10), pow(10, -8), pow(10, -6), pow(10, -4), pow(10, -2), pow(10, 0)])
ticks_font = font_manager.FontProperties(family='STIXGeneral', style='normal',
    size= 20, weight='bold', stretch='normal')
for label in ax1.get_xticklabels():
    label.set_fontproperties(ticks_font)
for label in ax1.get_yticklabels():
    label.set_fontproperties(ticks_font)
ax1.legend(prop={'family' : 'STIXGeneral', 'size' : 20,'weight':'bold'}, ncol=2)
ax1.grid(color='darkgrey',linestyle=':',linewidth=1)
plt.tight_layout()
fig.savefig(plot_path + 'distribution_iter.pdf', dpi=900, bbox_inches='tight')
plt.show()



# Figure 10(b)
beta_u = 1/6
beta_l = 1/30
rho = 0
def F(beta):
    return (beta_u - beta)/(rho*(beta - beta_l) + beta_u - beta_l)
def g(theta):
    return ((1+rho)*(beta_u-beta_l))/pow(rho*(theta - beta_l) + beta_u - beta_l, 2)

betalist = np.linspace(beta_l, beta_u, 100)
linewidths = [3, 2, 1, 2, 1, 2, 3]
ticks_font = font_manager.FontProperties(family='STIXGeneral',
                                         weight='bold', style='normal',
                                         size=20, stretch='normal')
fig = plt.figure(figsize = (8, 7))
ax1 = fig.add_subplot(111)
for i, rho in enumerate(selected_params):
    if (rho < 0):
        line = '--'
    elif ((rho > 0) and (rho < 10)):
        line = '-'
    elif ((rho > 0) and (rho >= 10)):
        line = '-'
    else:
        line='-.'
    ax1.plot(betalist, [g(i) for i in betalist], label = r'$\rho$=%s'%rho,color = colors[i], linewidth=linewidths[i], ls=line)
ax1.legend(prop={'family' : 'STIXGeneral', 'size' : 20,'weight':'bold'}, ncol=2)
ax1.set_xlim(beta_l, beta_u)
ax1.set_xlabel(r'$\theta$',fontdict=font1)
ax1.set_ylabel(r'$g(\theta)$',fontdict=font1)
ax1.grid(color='darkgrey',linestyle=':',linewidth=1)
for label in ax1.get_xticklabels():
    label.set_fontproperties(ticks_font)
for label in ax1.get_yticklabels():
    label.set_fontproperties(ticks_font)
fig.tight_layout()
plt.savefig(plot_path + "rational_distribution.pdf", dpi=900, bbox_inches='tight')
plt.show()