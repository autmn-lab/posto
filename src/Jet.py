import os,sys,copy
import time

PROJECT_ROOT = os.environ['MNTR_BB_ROOT_DIR']
sys.path.append(PROJECT_ROOT)

from Parameters import *
from lib.GenLog import *
from lib.TrajValidity import *
from lib.TrajSafety import *
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
                p = plt.Rectangle((lg[0][0][0], lg[0][1][0]), wd, ht, facecolor='none', edgecolor='cyan',linewidth=0.4,alpha=0.5)
                ax.add_patch(p)
                art3d.pathpatch_2d_to_3d(p, z=lg[1], zdir="z")

        for traj in trajs:
            x=[p[0] for p in traj]
            y=[p[1] for p in traj]
            t=list(range(0,len(traj)))
            ax.plot3D(x, y, t)
    
        plt.show()

    def vizTrajsVal(trajsVal,trajsInVal,logUn=None):

        ax = plt.axes(projection='3d')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('time')

        if logUn!=None:
            for lg in logUn:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                p = plt.Rectangle((lg[0][0][0], lg[0][1][0]), wd, ht, facecolor='none', edgecolor='cyan',linewidth=0.4,alpha=0.5)
                ax.add_patch(p)
                art3d.pathpatch_2d_to_3d(p, z=lg[1], zdir="z")

        for traj in trajsVal:
            x=[p[0] for p in traj]
            y=[p[1] for p in traj]
            t=list(range(0,len(traj)))
            ax.plot3D(x, y, t,color='blue')

        for traj in trajsInVal:
            x=[p[0] for p in traj]
            y=[p[1] for p in traj]
            t=list(range(0,len(traj)))
            ax.plot3D(x, y, t,color='red')
    
        plt.show()

    def vizTrajsVal2D(trajsVal,logUn=None,unsafe=0.0,state=0):

        t=list(range(len(trajsVal[0])))

        #print(t)
        plt.xlabel("Time")
        plt.ylabel("State-"+str(state))

        for traj in trajsVal:
            x=[p[state] for p in traj]
            plt.plot(t,x,linewidth=5)

        if logUn!=None:
            for lg in logUn:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                #print([lg[0][0][0], lg[0][0][1]],[lg[1],lg[1]])
                p = plt.plot([lg[1],lg[1]],[lg[0][state][0], lg[0][state][1]], color='black',linewidth=5)

        p = plt.plot(t, [unsafe]*len(t),color='red',linewidth=5,linestyle='dashed')

        plt.show()

    def getLog(initSet,T):
        trajs=Jet.getRandomTrajs(initSet,T,1)
        
        logger=GenLog(trajs[0])
        log=logger.genLog()

        Jet.vizTrajs(trajs,log[0])

    def getValidTrajs(initSet,T,K,logUn):
        #trajsL=Jet.getRandomTrajs(initSet,T,1)
        #logger=GenLog(trajsL[0])
        #logUn=logger.genLog()[0]
        #print(logUn[0][0])
        #ts=time.time()
        valTrajObj=TrajValidity(logUn)
        valTrajs=[]
        while len(valTrajs)<=K:
            trajs=Jet.getRandomTrajs(logUn[0][0],T,100)
            valTrajsIt,inValTrajsIt=valTrajObj.getValTrajs(trajs)
            valTrajs=valTrajs+valTrajsIt
            if len(valTrajs)>=K:
                break
        #ts=time.time()-ts
        #print("Time: ",ts)
        #Jet.vizTrajsVal(valTrajs[:5],inValTrajsIt[:5],logUn)
        #Jet.vizTrajsVal2D(valTrajs,logUn)
        return valTrajs

    def checkSafety(initSet,T):
        trajsL=Jet.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
        K=1300
        ts=time.time()
        unsafe=0
        state=0
        op='le'
        validTrajs=Jet.getValidTrajs(initSet,T,K,logUn)
        ts=time.time()-ts
        print("Time taken to generate ", len(validTrajs)," valid trajectories: ",ts)
        Jet.vizTrajsVal2D(validTrajs,logUn,unsafe,state)
        safeTrajObj=TrajSafety([state,op,unsafe])
        (safeTrajs,unsafeTrajs)=safeTrajObj.getSafeUnsafeTrajs(validTrajs)
        print(len(safeTrajs),len(unsafeTrajs))
        (safeSamps,unsafeSamps)=safeTrajObj.getSafeUnsafeLog(logUn)
        print(len(safeSamps),len(unsafeSamps))
        





    



x_init=0.8
y_init=0.8
T=2000
K=5
initState=(x_init,y_init)
initSet=([0.8,1.2],[0.8,1.2])
#initSet=([0.8,0.9],[0.8,0.9])
#Jet.getValidTrajs(initSet,T,K)
#Jet.getRandomTrajs(initSet,T,5)
#Jet.getLog(initSet,T)
Jet.checkSafety(initSet,T)