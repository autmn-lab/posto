import os,sys,copy
import time

PROJECT_ROOT = os.environ['MNTR_BB_ROOT_DIR']
sys.path.append(PROJECT_ROOT)

from Parameters import *
import numpy as np
import matplotlib.pyplot as plt


def plt1(save=False,name="Random"):
    '''
    Date: Feb 15, 2024.
    '''
    # Sample data
    par_change = [3, 5, 7, 9, 11]  # x-axis values (replace with your actual data)
    time_taken = [17.256429195404053, 26.06997299194336, 47.04519271850586, 90.87833285331726, 226.43406772613525]  # replace with your actual time data
    total_samples = [3700, 5400, 9700, 15800, 42300]  # replace with your actual total samples data
    valid_samples = [1150, 1150, 1154, 1147, 1148]    # replace with your actual valid samples data

    # Calculate percentage of valid samples
    percentage_valid_samples = [(vs / ts) * 100 for vs, ts in zip(valid_samples, total_samples)]

    # Plotting
    fig, ax1 = plt.subplots()

    # Plotting time taken
    color = 'maroon'
    ax1.set_xlabel('Logging Probability (%)',fontsize=18,fontweight='bold')
    ax1.set_ylabel('Time Taken', color=color,fontsize=18,fontweight='bold')
    ax1.plot(par_change, time_taken, color=color, marker='o',linewidth=6,markersize=15)
    ax1.tick_params(axis='y', labelcolor=color)

    # Creating a second y-axis for other data
    ax2 = ax1.twinx()
    color = 'tab:blue'

    # Plotting total samples
    ax2.set_ylabel('Total Trajectories', color=color,fontsize=18,fontweight='bold')
    ax2.plot(par_change, total_samples, color=color, marker='s',linewidth=6,markersize=15,alpha=0.7)
    ax2.tick_params(axis='y', labelcolor=color)

    # Creating a third y-axis for percentage of valid samples
    ax3 = ax1.twinx()
    color = 'tab:green'
    ax3.spines['right'].set_position(('outward', 60))

    # Plotting percentage of valid samples
    ax3.set_ylabel('% Valid Trajectories', color=color,fontsize=18,fontweight='bold')
    ax3.plot(par_change, percentage_valid_samples, color=color, marker='^',linewidth=6,markersize=15)
    ax3.tick_params(axis='y', labelcolor=color)

    # Adding legend
    ax1.legend(['Time Taken'], loc='lower center', prop={'size': 15})
    ax2.legend(['Total\nTrajs'], loc='center left', prop={'size': 15})
    ax3.legend(['% Valid Trajs'], loc='upper center', prop={'size': 15})

    #plt.title('Performance Metrics over Logging Probability')
    #plt.show()

    if save:
        plt.savefig(name+".pdf", format="pdf", bbox_inches="tight")
    else:
        plt.show()
    plt.clf()




# main code
plt1(save=False,name="nLogs")