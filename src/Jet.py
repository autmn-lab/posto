import os,sys,copy

PROJECT_ROOT = os.environ['MNTR_BB_ROOT_DIR']
sys.path.append(PROJECT_ROOT)

from Parameters import *
from lib.GenLog import *
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d
import random

class Jet:

    def getNextState(state):

        dt=DT
        ep=DELTA_STATE
        
        x_cur=copy.copy(state[0])
        y_cur=copy.copy(state[1])

        x_cur+=random.uniform(0,ep)
        y_cur+=random.uniform(0,ep)

        x_next=x_cur+(dt*(-y_cur-(1.5*(x_cur*x_cur)-(0.5*(x_cur*x_cur*x_cur))-0.5)));
        y_next=y_cur+(dt*((3*x_cur)-y_cur));

        nextState=(x_next,y_next)
        return nextState
    
    def getTraj(initState,T):
        traj=[]
        state=copy.copy(initState)
        for t in range(T):
            traj.append(state)
            nextState=Jet.getNextState(state)
            state=copy.copy(nextState)
        return traj
    
    def getRandomTrajs(initSet,T,K):
        trajs=[]
        for i in range(K):
            x_init_rand=random.uniform(initSet[0][0], initSet[0][1])
            y_init_rand=random.uniform(initSet[1][0], initSet[1][1])
            traj=Jet.getTraj((x_init_rand,y_init_rand),T)
            trajs.append(traj)
        #Jet.vizTrajs(trajs)
        return trajs

    def vizTrajs(trajs,logUn=None):

        ax = plt.axes(projection='3d')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('time')

        if logUn!=None:
            for lg in logUn:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                p = plt.Rectangle((lg[0][0][0], lg[0][1][0]), wd, ht, facecolor='none', edgecolor='black',linewidth=0.4,alpha=0.8)
                ax.add_patch(p)
                art3d.pathpatch_2d_to_3d(p, z=lg[1], zdir="z")

        for traj in trajs:
            x=[p[0] for p in traj]
            y=[p[1] for p in traj]
            t=list(range(0,len(traj)))
            ax.plot3D(x, y, t)
    
        plt.show()

    def getLog(initSet,T,K):
        trajs=Jet.getRandomTrajs(initSet,T,100)
        
        logger=GenLog(trajs[0])
        log=logger.genLog()

        Jet.vizTrajs(trajs,log[0])



    



x_init=0.8
y_init=0.8
T=2000
K=5
initState=(x_init,y_init)
initSet=([0.8,1.2],[0.8,1.2])
Jet.getLog(initSet,T,K)