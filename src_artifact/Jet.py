import os,sys,copy
import time

PROJECT_ROOT = os.environ['MNTR_BB_ROOT_DIR']
sys.path.append(PROJECT_ROOT)

from Parameters import *
from lib.GenLog import *
from lib.TrajValidity import *
from lib.TrajSafety import *
from lib.JFBF import *
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

    def vizTrajs(trajs,logUn=None,save=False,name="Untitled"):

        ax = plt.axes(projection='3d')
        ax.set_xlabel('x',fontsize=20,fontweight='bold')
        ax.set_ylabel('y',fontsize=20,fontweight='bold')
        ax.set_zlabel('time',fontsize=8,fontweight='bold')

        if logUn!=None:
            for lg in logUn:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                p = plt.Rectangle((lg[0][0][0], lg[0][1][0]), wd, ht, facecolor='none', edgecolor='black',linewidth=0.4,alpha=0.5)
                ax.add_patch(p)
                art3d.pathpatch_2d_to_3d(p, z=lg[1], zdir="z")

        for traj in trajs:
            x=[p[0] for p in traj]
            y=[p[1] for p in traj]
            t=list(range(0,len(traj)))
            ax.plot3D(x, y, t)

        if save:
            plt.savefig(name+".pdf", format="pdf", bbox_inches="tight")
        else:
            plt.show()
        plt.clf()
        
    def vizTrajsVal(trajsVal,trajsInVal,logUn=None,save=False,name="Untitled"):

        ax = plt.axes(projection='3d')
        ax.set_xlabel('x',fontsize=20,fontweight='bold')
        ax.set_ylabel('y',fontsize=20,fontweight='bold')
        ax.set_zlabel('time',fontsize=8,fontweight='bold')

        if logUn!=None:
            for lg in logUn:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                p = plt.Rectangle((lg[0][0][0], lg[0][1][0]), wd, ht, facecolor='none', edgecolor='black',linewidth=0.4,alpha=0.5)
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
            ax.plot3D(x, y, t,color='red',alpha=0.3)
    
        if save:
            plt.savefig(name+".pdf", format="pdf", bbox_inches="tight")
        else:
            plt.show()
        plt.clf()

    def vizTrajsVal2D(trajsVal,logUn=None,unsafe=0.0,state=0,save=False,name="Untitled"):

        lnWd=2

        t=list(range(len(trajsVal[0])))

        #print(t)
        plt.xlabel("Time",fontsize=20,fontweight='bold')
        plt.ylabel("State-"+str(state),fontsize=20,fontweight='bold')

        for traj in trajsVal:
            x=[p[state] for p in traj]
            plt.plot(t,x,linewidth=lnWd)

        if logUn!=None:
            for lg in logUn:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                #print([lg[0][0][0], lg[0][0][1]],[lg[1],lg[1]])
                p = plt.plot([lg[1],lg[1]],[lg[0][state][0], lg[0][state][1]], color='black',linewidth=lnWd,alpha=0.6)

        if unsafe!=None:
            p = plt.plot(t, [unsafe]*len(t),color='red',linewidth=lnWd,linestyle='dashed')

        if save:
            plt.savefig(name+".pdf", format="pdf", bbox_inches="tight")
        else:
            plt.show()
        plt.clf()

    def vizTrajsValInVal2D(trajsVal,inValTrajs,logUn=None,unsafe=None,state=0,save=False,name="Untitled"):

        lnWd=2

        t=list(range(len(trajsVal[0])))

        #print(t)
        plt.xlabel("Time",fontsize=20,fontweight='bold')
        plt.ylabel("State-"+str(state),fontsize=20,fontweight='bold')

        for traj in trajsVal:
            x=[p[state] for p in traj]
            plt.plot(t,x,linewidth=lnWd,color='blue')

        for traj in inValTrajs:
            x=[p[state] for p in traj]
            plt.plot(t,x,linewidth=lnWd,color='magenta')

        if logUn!=None:
            for lg in logUn:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                #print([lg[0][0][0], lg[0][0][1]],[lg[1],lg[1]])
                p = plt.plot([lg[1],lg[1]],[lg[0][state][0], lg[0][state][1]], color='black',linewidth=lnWd)

        if unsafe!=None:
            p = plt.plot(t, [unsafe]*len(t),color='red',linewidth=lnWd,linestyle='dashed')

        if save:
            plt.savefig(name+".pdf", format="pdf", bbox_inches="tight")
        else:
            plt.show()
        plt.clf()

    def vizTrajsSafeUnsafe2D(safeTrajs,unsafeTrajs,safeSamps,unsafeSamps,unsafe=0.0,state=0,save=False,name="Untitled"):

        lnWd=2

        if len(safeTrajs)>0:
            t=list(range(len(safeTrajs[0])))
        else:
            t=list(range(len(unsafeTrajs[0])))

        #print(t)
        plt.xlabel("Time",fontsize=20,fontweight='bold')
        plt.ylabel("State-"+str(state),fontsize=20,fontweight='bold')

        for traj in safeTrajs:
            x=[p[state] for p in traj]
            plt.plot(t,x,linewidth=lnWd,color='blue')
        
        for traj in unsafeTrajs:
            x=[p[state] for p in traj]
            plt.plot(t,x,linewidth=lnWd,color='red',linestyle='dashdot',alpha=0.8)

        if safeSamps!=None:
            for lg in safeSamps:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                #print([lg[0][0][0], lg[0][0][1]],[lg[1],lg[1]])
                p = plt.plot([lg[1],lg[1]],[lg[0][state][0], lg[0][state][1]], color='black',linewidth=lnWd)
        
        if unsafeSamps!=None:
            for lg in unsafeSamps:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                #print([lg[0][0][0], lg[0][0][1]],[lg[1],lg[1]])
                p = plt.plot([lg[1],lg[1]],[lg[0][state][0], lg[0][state][1]], color='brown',linewidth=lnWd)

        p = plt.plot(t, [unsafe]*len(t),color='red',linewidth=lnWd,linestyle='dashed')

        if save:
            plt.savefig(name+".pdf", format="pdf", bbox_inches="tight")
        else:
            plt.show()
        plt.clf()

    def vizLogsSafeUnsafe2D(T,safeSamps,unsafeSamps,unsafe=0.0,state=0,save=False,name="Untitled"):

        lnWd=2

        t=list(range(T))

        #print(t)
        plt.xlabel("Time",fontsize=20,fontweight='bold')
        plt.ylabel("State-"+str(state),fontsize=20,fontweight='bold')

        if safeSamps!=None:
            for lg in safeSamps:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                #print([lg[0][0][0], lg[0][0][1]],[lg[1],lg[1]])
                p = plt.plot([lg[1],lg[1]],[lg[0][state][0], lg[0][state][1]], color='black',linewidth=lnWd)
        
        if unsafeSamps!=None:
            for lg in unsafeSamps:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                #print([lg[0][0][0], lg[0][0][1]],[lg[1],lg[1]])
                p = plt.plot([lg[1],lg[1]],[lg[0][state][0], lg[0][state][1]], color='brown',linewidth=lnWd)

        p = plt.plot(t, [unsafe]*len(t),color='red',linewidth=lnWd,linestyle='dashed')

        if save:
            plt.savefig(name+".pdf", format="pdf", bbox_inches="tight")
        else:
            plt.show()
        plt.clf()

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
        totTrajs=0
        valTrajObj=TrajValidity(logUn)
        valTrajs=[]
        while len(valTrajs)<=K:
            trajs=Jet.getRandomTrajs(logUn[0][0],T,100)
            totTrajs+=1
            valTrajsIt,inValTrajsIt=valTrajObj.getValTrajs(trajs)
            valTrajs=valTrajs+valTrajsIt
            if len(valTrajs)>=K:
                break
        #ts=time.time()-ts
        #print("Time: ",ts)
        #Jet.vizTrajsVal(valTrajs[:5],inValTrajsIt[:5],logUn)
        #Jet.vizTrajsVal2D(valTrajs,logUn)
        print("Total Trajectories Generated: ",totTrajs*100,"; Valid Trajectories: ",len(valTrajs))
        return valTrajs

    def checkSafety2(initSet,T):
        trajsL=Jet.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
        K=1300
        ts=time.time()
        unsafe=-0.121
        state=0
        op='le'
        validTrajs=Jet.getValidTrajs(initSet,T,K,logUn)
        ts=time.time()-ts
        print("Time taken to generate ", len(validTrajs)," valid trajectories: ",ts)
        #Jet.vizTrajsVal2D(validTrajs,logUn,unsafe,state)

        ts=time.time()
        safeTrajObj=TrajSafety([state,op,unsafe])
        (safeTrajs,unsafeTrajs)=safeTrajObj.getSafeUnsafeTrajs(validTrajs)
        (safeSamps,unsafeSamps)=safeTrajObj.getSafeUnsafeLog(logUn)
        ts=time.time()-ts

        print("[Trajs] Safe, Unsafe: ",len(safeTrajs),len(unsafeTrajs))
        print("[Log] Safe, Unsafe: ",len(safeSamps),len(unsafeSamps))
        print("Time taken to filter the trajs/logs: ",ts)

        if len(unsafeTrajs)>0 and len(safeTrajs)>0:
            Jet.vizTrajsSafeUnsafe2D([safeTrajs[0]],[unsafeTrajs[0]],safeSamps,unsafeSamps,unsafe,state)

    def isSafe(initSet,T,unsafe,state,op,Bi,ci):
        ts=time.time()
        trajsL=Jet.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
        K=JFB(Bi,ci).getNumberOfSamples()
        #K=200
        isSafe=True
        totTrajs=0
        valTrajObj=TrajValidity(logUn)
        valTrajs=[]
        safeTrajs=[]
        unsafeTrajs=[]
        safeTrajObj=TrajSafety([state,op,unsafe])
        (safeSamps,unsafeSamps)=safeTrajObj.getSafeUnsafeLog(logUn)
        if len(unsafeSamps)==0 or False:
            while len(valTrajs)<=K:
                trajs=Jet.getRandomTrajs(logUn[0][0],T,100)
                totTrajs+=1
                valTrajsIt,inValTrajsIt=valTrajObj.getValTrajs(trajs)
                valTrajs=valTrajs+valTrajsIt
                print(totTrajs*100,len(valTrajs))
                # Check safety of valTrajsIt
                (safeTrajs,unsafeTrajs)=safeTrajObj.getSafeUnsafeTrajs(valTrajsIt)
                if len(unsafeTrajs)>0:
                    isSafe=False
                    break
                ############################

                if len(valTrajs)>=K:
                    break
        else:
            isSafe=False
        
        ts=time.time()-ts
        print("Time Taken: ",ts)
        print("Safety: ",isSafe)
        print("[Trajs] Safe, Unsafe: ",len(safeTrajs),len(unsafeTrajs))
        print("[Log] Safe, Unsafe: ",len(safeSamps),len(unsafeSamps))
        print("Total Trajectories Generated: ",totTrajs*100,"; Valid Trajectories: ",len(valTrajs))
        return (ts,isSafe)
        
    def vizVaryC(cList,sList,tList,save=False,name="Untitled"):
        plt.xlabel(r'$c$',fontsize=20,fontweight = 'bold')
        plt.ylabel(r'Time taken',fontsize=20,fontweight = 'bold')
        L=len(cList)
        
        plt.plot(cList,tList,linewidth=5,linestyle='dashed')

        for i in range(L):
            if sList[i]==True:
                plt.scatter(cList[i], tList[i], s=350, c='green')
            else:
                plt.scatter(cList[i], tList[i], s=350, c='red')
        
        if save:
            plt.savefig(name+".pdf", format="pdf", bbox_inches="tight")
        else:
            plt.show()
        plt.clf()

    def checkSafety(initSet,T,unsafe,state,op):
        ts=time.time()
        trajsL=Jet.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
        K=JFB(B,c).getNumberOfSamples()
        #K=200
        isSafe=True
        totTrajs=0
        valTrajObj=TrajValidity(logUn)
        valTrajs=[]
        safeTrajs=[]
        unsafeTrajs=[]
        safeTrajObj=TrajSafety([state,op,unsafe])
        (safeSamps,unsafeSamps)=safeTrajObj.getSafeUnsafeLog(logUn)
        if len(unsafeSamps)==0 or False:
            while len(valTrajs)<=K:
                trajs=Jet.getRandomTrajs(logUn[0][0],T,100)
                totTrajs+=1
                valTrajsIt,inValTrajsIt=valTrajObj.getValTrajs(trajs)
                print(totTrajs*100,len(valTrajs))
                # Check safety of valTrajsIt
                (safeTrajs,unsafeTrajs)=safeTrajObj.getSafeUnsafeTrajs(valTrajsIt)
                if len(unsafeTrajs)>0:
                    isSafe=False
                    break
                ############################

                valTrajs=valTrajs+valTrajsIt
                if len(valTrajs)>=K:
                    break
        else:
            isSafe=False
        
        ts=time.time()-ts
        print("Time Taken: ",ts)
        print("Safety: ",isSafe)
        print("[Trajs] Safe, Unsafe: ",len(safeTrajs),len(unsafeTrajs))
        print("[Log] Safe, Unsafe: ",len(safeSamps),len(unsafeSamps))
        print("Total Trajectories Generated: ",totTrajs*100,"; Valid Trajectories: ",len(valTrajs))

        sv=False

        if len(unsafeSamps)>0:
            Jet.vizLogsSafeUnsafe2D(T,safeSamps,unsafeSamps,unsafe,state,save=sv,name="JetSafeUnsafeLogs")
        
        if len(unsafeTrajs)>0 and len(safeTrajs)>0:
            #print("A")
            Jet.vizTrajsSafeUnsafe2D([safeTrajs[0]],[unsafeTrajs[0]],safeSamps,unsafeSamps,unsafe,state,save=sv,name="JetSafeUnsafeTrajs")
        elif len(safeTrajs)>0 and len(unsafeTrajs)==0:
            #print("B")
            Jet.vizTrajsVal2D(safeTrajs,logUn,unsafe,state,save=sv,name="JetSafeTrajs")
        elif len(unsafeTrajs)>0:
            #print("C")
            Jet.vizTrajsVal2D(unsafeTrajs,logUn,unsafe,state,save=True,name="JetUnsafeTrajs")


    def showBehavior(initSet,T):
        print("Plotting Behavior")
        trajsL=Jet.getRandomTrajs(initSet,T,10)
        Jet.vizTrajs(trajsL,save=True,name="JetBehavior")
    
    def showLogGeneration(initSet,T):
        trajsL=Jet.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
        Jet.vizTrajs(trajsL,logUn,save=True,name="JetLog3D")
        Jet.vizTrajsVal2D(trajsL,logUn,unsafe=-0.10,state=0,save=True,name="JetLogS0")
        Jet.vizTrajsVal2D(trajsL,logUn,unsafe=None,state=1,save=True,name="JetLogS1")
    
    def showValidTrajs(initSet,T,K):
        trajsL=Jet.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
    
        ts=time.time()
        valTrajObj=TrajValidity(logUn)
        valTrajs=[]
        while len(valTrajs)<=K:
            trajs=Jet.getRandomTrajs(logUn[0][0],T,100)
            valTrajsIt,inValTrajsIt=valTrajObj.getValTrajs(trajs)
            valTrajs=valTrajs+valTrajsIt
            if len(valTrajs)>=K:
                break
        ts=time.time()-ts
        print("Time taken: ",ts)
        Jet.vizTrajsVal(valTrajs[:5],inValTrajsIt[:5],logUn,save=True,name="JetValTrajs")
        Jet.vizTrajsValInVal2D(valTrajs[:1],inValTrajsIt[:1],logUn,unsafe=-0.10,state=0,save=True,name="JetValTrajsS0")
        Jet.vizTrajsValInVal2D(valTrajs[:1],inValTrajsIt[:1],logUn,unsafe=None,state=1,save=True,name="JetValTrajsS1")

    def varyC(initSet,T,unsafe,state,op):
        cList=[0.6,0.7,0.8,0.9,0.99]
        tList=[]
        sList=[]
        for ci in cList:
            print(">> c = ",ci)
            (t,sF)=Jet.isSafe(initSet,T,unsafe,state,op,B,ci)
            tList.append(t)
            sList.append(sF)
            print("=====================\n\n")

        print(tList)
        print(sList)
        Jet.vizVaryC(cList,sList,tList,save=True,name="JetVaryC")







    



x_init=0.8
y_init=0.8
T=2000

initState=(x_init,y_init)
#initSet=([0.8,1.2],[0.8,1.2])
#initSet=([0.8,0.9],[0.8,0.9])
initSet=([0.8,1],[0.8,1])
#Jet.getValidTrajs(initSet,T,K)
#Jet.getRandomTrajs(initSet,T,5)
#Jet.getLog(initSet,T)
#Jet.checkSafety(initSet,T)

########### Results ########### 

unsafe=-0.10
state=0
op='le'

#Jet.showBehavior(initSet,T)
#Jet.showLogGeneration(initSet,T)
#Jet.showValidTrajs(initSet,T,K=20)
#Jet.checkSafety(initSet,T,unsafe,state,op)
#Jet.varyC(initSet,T,unsafe,state,op)

# Generating Fig 4(a)
PROBABILITY_LOG=5
DELTA_LOG=0.02
Jet.checkSafety(initSet,T,unsafe,state,op)

# Generating Fig 4(b)
PROBABILITY_LOG=5
DELTA_LOG=0.04
Jet.checkSafety(initSet,T,unsafe,state,op)

# Generating Fig 4(c)
PROBABILITY_LOG=3
DELTA_LOG=0.02
Jet.checkSafety(initSet,T,unsafe,state,op)

# Generating Fig 4(d)
PROBABILITY_LOG=5
DELTA_LOG=0.04
Jet.checkSafety(initSet,T,unsafe,state,op)