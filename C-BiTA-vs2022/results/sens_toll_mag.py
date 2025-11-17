# -*- coding: utf-8 -*-
"""
Created on Thu Sep  4 03:18:02 2025

@author: 84091
"""

import matplotlib.pyplot as plt
from matplotlib import font_manager
import pandas as pd
import seaborn as sns
from itertools import islice
import numpy as np


def read_ite_gene(filename):
    try:
        df1 = pd.read_table(filename,sep='\s+')
        if 'SBA' in filename:
            [0, 100000000, 1.0, 0]
            df1['Iter'] = df1['Iter'] + 1
            first_row = pd.DataFrame({
                'Iter': [0],
                'OFV': [100000000],
                'ConvIndc': [1.0],
                'Time':[0]})
            df1 = pd.concat([first_row, df1], ignore_index=True)
        return (df1)
    except:
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

def read_info(fileclassname):
    f = open(base_path + "chicagosketch_SBA_toll_mag_%s.info"%fileclassname, "r")
    data = [list(map(float, line.split())) for line in islice(f,2,None)]
    df2 = pd.DataFrame.from_dict(data)
    effpanum=df2[2].sum()
    efficientdist = df2[2].values
    df2.fillna(0)
    max_boundnum = max(efficientdist)+1
    diffsize = max_boundnum - 1
    for i in range(int(diffsize)):
        df2[max_boundnum+3+i] = df2.apply(lambda x: x[4+i] -x[3+i], axis=1)
    df2[df2 < 0] = np.nan
    diff = df2.loc[:,max_boundnum+3:]
    diff1 = diff.values.reshape((1,-1))
    diff1[np.isnan(diff1)] = 100
    diff2 = diff1[diff1<1]
    return (diff2, max_boundnum)

def read_info2(fileclassname):
    f = open(base_path + "chicagosketch_SBA_toll_mag_%s.info"%fileclassname, "r")
    data = [list(map(float, line.split())) for line in islice(f,2,None)]
    df2 = pd.DataFrame.from_dict(data)
    effpanum=df2[2].sum()
    efficientdist = df2[2].values
    df2.fillna(0)
    max_boundnum = max(efficientdist)+1
    diffsize = max_boundnum - 1
    for i in range(int(diffsize)):
        df2[max_boundnum+3+i] = df2.apply(lambda x: x[4+i] -x[3+i], axis=1)
    df2[df2 < 0] = np.nan
    diff = df2.loc[:,max_boundnum+3:]
    diff1 = diff.values.reshape((1,-1))
    diff1[np.isnan(diff1)] = 100
    diff2 = diff1[diff1<0.133334]
    return (diff2, max_boundnum)


base_path = './raw_results/sens_toll_mag/'
plot_path='./figures/sens_toll_mag/'
mag_test = [2, 6, 12, 20, 30, 60, 100, 160]


# # Figure 7(a)
print("Now Plotting Figure 7(a)!")
totaltreedict = {}
efficientpathnum = {}
efficientdist = {}
for i in mag_test:
    df = pd.read_table(base_path + "chicagosketch_SBA_treenum_toll_mag_%s.tnum"%i, sep = " ")
    f = open(base_path + "chicagosketch_SBA_toll_mag_%s.info"%i, "r")
    data = [list(map(float, line.split())) for line in islice(f,2,None)]
    df2 = pd.DataFrame.from_dict(data)
    df['col_sum']=df.apply(lambda x:x.sum(), axis=1)
    effpanum=df2[2].sum()
    efficientdist[i] = df2[2].values
    totaltreedict[i] = df.loc[df.shape[0]-1,'col_sum']
    efficientpathnum[i]  = effpanum
fig = plt.figure(figsize=(14,6))
palette=sns.color_palette("Reds", len(mag_test))
font1 = {'family':'Times New Roman','weight':'bold','size': 20}
font2 = {'family':'Times New Roman','weight':'normal','size': 15}
ticks_font = font_manager.FontProperties(family='STIXGeneral', style='normal',
    size=17, weight='bold', stretch='normal')
for pos in range(len(mag_test)):
    ax = fig.add_subplot(2,4,pos+1)
    ax.set_axisbelow(True) 
    ax.grid(linestyle=':', zorder=0)
    ax.hist(efficientdist[mag_test[pos]], bins=int(max(efficientdist[mag_test[pos]])), color=palette[pos],label = r'$\alpha_{0}$=%s'%mag_test[pos],
            rwidth=0.85)
    ax.set_yscale('log')
    ax.set_xlim(0,25)
    ax.set_ylim(0.6,pow(10,5))
    ax.text(14, 3*pow(10,2), 'mean=%.2f\nmedian=%.d\nmax=%.d'%(efficientdist[mag_test[pos]].mean(), np.median(efficientdist[mag_test[pos]]), efficientdist[mag_test[pos]].max()), fontdict=font2)
    ax.text(14, 1*pow(10,2), 'total='+format(efficientdist[mag_test[pos]].sum(),'0,.0f') , fontdict=font2)
    for label in ax.get_xticklabels():
        label.set_fontproperties(ticks_font)
    for label in ax.get_yticklabels():
        label.set_fontproperties(ticks_font)
    if pos==8-1:
        ax.set_xlabel('Efficient Path Counts', fontdict=font1)
    if pos==1-1:
        ax.set_ylabel('OD Counts', fontdict=font1)
    ax.legend(prop={'family':'STIXGeneral', 'size':18, 'weight':'bold'},
          handlelength=1.5,
          handletextpad=0.3,
          borderaxespad=0.2,
          labelspacing=0.2,
          borderpad=0.2
          )
plt.tight_layout()
fig.savefig(plot_path + 'efficientpath_change.pdf', 
            dpi=900, bbox_inches='tight')

for i in mag_test:
    print("More than 5 efficient paths OD ratio at alpha_0=%s is "%i, (efficientdist[i] > 5).sum() / efficientdist[i].size)

# Figure 7(b)
print("Now Plotting Figure 7(b)!")
Diff = {} 
Diff_no0 = {}
Max_boundnum = {}
Max_boundnun_no0 = {}
for i in mag_test:
    Diff[i], Max_boundnum[i] = read_info(i)
    Diff_no0[i], Max_boundnun_no0[i] = read_info2(i)
fig = plt.figure(figsize=(14, 6))
palette=sns.color_palette("Reds", len(mag_test))
font1 = {'family': 'Times New Roman', 'weight':'bold', 'size': 20}
font2 = {'family': 'Times New Roman', 'weight':'normal', 'size': 15}
ticks_font = font_manager.FontProperties(family='STIXGeneral', style='normal',
    size=14, weight='bold', stretch='normal')
ticks_font2 = font_manager.FontProperties(family='STIXGeneral', style='normal',
    size=17, weight='bold', stretch='normal')
for pos in range(len(mag_test)):
    ax = fig.add_subplot(2, 4, pos+1)
    ax.set_axisbelow(True) 
    ax.grid(linestyle=':', zorder=0)
    ax.grid(linestyle=':')
    ax.hist(Diff[mag_test[pos]], bins=int(Max_boundnum[mag_test[pos]]), 
            color=palette[pos], label=r'$\alpha_{0}$=%s'%mag_test[pos],
            rwidth=0.85)
    ax.set_xlim(-0.01, 0.15)
    if (pos == 0):
        ax.set_ylim(-1,70000)
        ax.text(0.00, 39000, 'mean=%.4f\nmedian=%.4f\nvariance=%.6f'%(Diff[mag_test[pos]].mean(), 
                                                                      np.median(Diff[mag_test[pos]]), 
                                                                      np.var(Diff[mag_test[pos]]) ), fontdict=font2)
    elif (pos <= 3):
        ax.set_ylim(-1,55000)
        if (pos==1):
            ax.text(0.00, 31000, 'mean=%.4f\nmedian=%.4f\nvariance=%.6f'%(Diff[mag_test[pos]].mean(), 
                                                                          np.median(Diff[mag_test[pos]]), 
                                                                          np.var(Diff[mag_test[pos]]) ), fontdict=font2)
        else:
            ax.text(0.05, 31000, 'mean=%.4f\nmedian=%.4f\nvariance=%.6f'%(Diff[mag_test[pos]].mean(), 
                                                                            np.median(Diff[mag_test[pos]]), 
                                                                            np.var(Diff[mag_test[pos]]) ), fontdict=font2)
    else:
        ax.set_ylim(-1,82000)
        ax.text(0.05, 46000, 'mean=%.4f\nmedian=%.4f\nvariance=%.6f'%(Diff[mag_test[pos]].mean(), 
                                                                        np.median(Diff[mag_test[pos]]), 
                                                                        np.var(Diff[mag_test[pos]]) ), fontdict=font2)
    ax.tick_params(pad=0.001, axis='y')
    for label in ax.get_xticklabels():
        label.set_fontproperties(ticks_font2)
    for label in ax.get_yticklabels():
        label.set_fontproperties(ticks_font)
    if pos == 8-1:
        ax.set_xlabel('TEM Interval Length', fontdict=font1)
    if pos == 1-1:
        ax.set_ylabel('Interval Counts', fontdict=font1)
    ax.legend(prop={'family':'STIXGeneral', 'size':18, 'weight':'bold'}, 
              handlelength=1.5,
              handletextpad=0.3, 
              borderaxespad=0.2, 
              labelspacing=0.2,
              borderpad=0.2)
plt.tight_layout()
plt.subplots_adjust(wspace=0.22)
fig.savefig(plot_path + 'boundarydiff_change.pdf', dpi=900,bbox_inches='tight')

# # Figure 8
print("Now Plotting Figure 8!")
fig = plt.figure(figsize=(8,7))
ax1 = fig.add_subplot(111)
ax1.set_yscale('log')
colors = ['mediumblue', 'dodgerblue',  'forestgreen', 'limegreen', 'gold', 'orange','red', 'brown']
linestyles=[':',  '-.', '--', '--', '-.', '-', '-', '-']
linewidths = [ 3, 1.5, 1.5, 3, 2.5, 1.5, 2.5, 3.5]
for k, i in enumerate(mag_test):
    df = read_ite_gene(base_path + "chicagosketch_SBA_toll_mag_%s.ite"%i)
    ax1.plot(df["Time"][0:], df["ConvIndc"][0:],
              label=r'$\alpha_{0}$=%s'%i, 
              linestyle=linestyles[k], 
              linewidth = linewidths[k], 
              color=colors[k])
font1 = {'family':'Times New Roman','weight':'heavy','size': 24}
ax1.set_xlabel('CPU Time (s)',fontdict=font1)
ax1.set_ylabel('Relative Gap', fontdict=font1)
x_length = 250
ax1.set_xlim(-0.02 * x_length, x_length)
ax1.set_ylim(pow(10, -11), 2)
ax1.set_yticks([ pow(10, -10), pow(10, -8), pow(10, -6), pow(10, -4), pow(10, -2), pow(10, 0)])
ticks_font = font_manager.FontProperties(family='STIXGeneral', style='normal',
    size=20, weight='heavy', stretch='normal')
for label in ax1.get_xticklabels():
    label.set_fontproperties(ticks_font)
for label in ax1.get_yticklabels():
    label.set_fontproperties(ticks_font)
ax1.legend(prop={'family' : 'STIXGeneral', 'size' : 20,'weight':'heavy'}, ncol=2)
ax1.grid(color='darkgrey',linestyle=':',linewidth=1)
plt.tight_layout()
fig.savefig(plot_path + 'toll_mag_conv.pdf', dpi=900, bbox_inches='tight')
plt.show()
print("All plots finished!")
