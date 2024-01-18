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
from tqdm import *

class VanderPol:

    def getNextState(state):
        DELTA_MU=0.01
        dt=DT
        ep=DELTA_STATE
        mu=1
        
        x_cur=copy.copy(state[0])
        y_cur=copy.copy(state[1])

        x_cur+=random.uniform(0,ep)
        y_cur+=random.uniform(0,ep)
        mu+=random.uniform(0,DELTA_MU)

        x_next=x_cur+(dt*y_cur)
        y_next=y_cur+(dt*((mu*(1-(x_cur*x_cur))*y_cur)-x_cur))

        nextState=(x_next,y_next)
        return nextState

    def getTraj(initState,T):
        traj=[]
        state=copy.copy(initState)
        for t in range(T):
            traj.append(state)
            nextState=VanderPol.getNextState(state)
            state=copy.copy(nextState)
        return traj

    def getRandomTrajs(initSet,T,K):
        trajs=[]
        for i in range(K):
            x_init_rand=random.uniform(initSet[0][0], initSet[0][1])
            y_init_rand=random.uniform(initSet[1][0], initSet[1][1])
            traj=VanderPol.getTraj((x_init_rand,y_init_rand),T)
            trajs.append(traj)
        #VanderPol.vizTrajs(trajs)
        return trajs

    def vizTrajs(trajs,logUn=None,save=False,name="Untitled"):

        ax = plt.axes(projection='3d')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('time')

        if logUn!=None:
            for lg in logUn:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                p = plt.Rectangle((lg[0][0][0], lg[0][1][0]), wd, ht, facecolor='none', edgecolor='black',linewidth=0.4,alpha=1)
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

    def getLog(initSet,T):
        trajs=VanderPol.getRandomTrajs(initSet,T,1)
        
        logger=GenLog(trajs[0])
        log=logger.genLog()

        #print(log[0])

        VanderPol.vizTrajs(trajs,log[0])

    def vizTrajsVal(trajsVal,trajsInVal,logUn=None,save=False,name="Untitled"):

        ax = plt.axes(projection='3d')
        ax.set_xlabel('x',fontsize=20,fontweight='bold')
        ax.set_ylabel('y',fontsize=20,fontweight='bold')
        ax.set_zlabel('time',fontsize=20,fontweight='bold')

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
            ax.plot3D(x, y, t,color='red')
    
        if save:
            plt.savefig(name+".pdf", format="pdf", bbox_inches="tight")
        else:
            plt.show()
        plt.clf()

    def vizTrajsVal2D(trajsVal,logUn=None,unsafe=None,state=0,save=False,name="Untitled"):

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
                p = plt.plot([lg[1],lg[1]],[lg[0][state][0], lg[0][state][1]], color='black',linewidth=lnWd)

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

    def getValidTrajs(initSet,T,K,logUn):
        trajsL=VanderPol.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
        ts=time.time()
        valTrajObj=TrajValidity(logUn)
        valTrajs=[]
        pbar = tqdm(total=K)
        while len(valTrajs)<=K:
            trajs=VanderPol.getRandomTrajs(logUn[0][0],T,100)
            valTrajsIt,inValTrajsIt=valTrajObj.getValTrajs(trajs)
            valTrajs=valTrajs+valTrajsIt
            #print(len(valTrajs),"/",K)
            pbar.update(len(valTrajsIt))
            if len(valTrajs)>=K:
                pbar.close()
                break
        pbar.close()
        #ts=time.time()-ts
        #print("Time: ",ts)
        VanderPol.vizTrajsVal(valTrajs[:5],inValTrajsIt[:5],logUn)
        VanderPol.vizTrajsVal2D(valTrajs,logUn,unsafe=2.9,state=0)
        return valTrajs

    def checkSafety2(initSet,T):
        trajsL=VanderPol.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
        K=1300
        ts=time.time()
        unsafe=2.7
        state=1
        op='ge'
        validTrajs=VanderPol.getValidTrajs(initSet,T,K,logUn)
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
            VanderPol.vizTrajsSafeUnsafe2D([safeTrajs[0]],[unsafeTrajs[0]],safeSamps,unsafeSamps,unsafe,state)
        
    def isSafe(initSet,T,unsafe,state,op,Bi,ci):
        ts=time.time()
        trajsL=VanderPol.getRandomTrajs(initSet,T,1)
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
                trajs=VanderPol.getRandomTrajs(logUn[0][0],T,100)
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
    
    def showBehavior(initSet,T):
        trajsL=VanderPol.getRandomTrajs(initSet,T,10)
        VanderPol.vizTrajs(trajsL,save=True,name="VanderPolBehavior")

    def showLogGeneration(initSet,T):
        trajsL=VanderPol.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
        VanderPol.vizTrajs(trajsL,logUn,save=True,name="VanderPolLog3D")
        VanderPol.vizTrajsVal2D(trajsL,logUn,unsafe=None,state=0,save=True,name="VanderPolLogS0")
        VanderPol.vizTrajsVal2D(trajsL,logUn,unsafe=3,state=1,save=True,name="VanderPolLogS1")

    def showValidTrajs(initSet,T,K):
        trajsL=VanderPol.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
    
        ts=time.time()
        valTrajObj=TrajValidity(logUn)
        valTrajs=[]
        while len(valTrajs)<=K:
            trajs=VanderPol.getRandomTrajs(logUn[0][0],T,100)
            valTrajsIt,inValTrajsIt=valTrajObj.getValTrajs(trajs)
            valTrajs=valTrajs+valTrajsIt
            print(len(valTrajs))
            if len(valTrajs)>=K:
                break
        ts=time.time()-ts
        print("Time taken: ",ts)
        VanderPol.vizTrajsVal(valTrajs[:5],inValTrajsIt[:5],logUn,save=True,name="VanderPolValTrajs")
        VanderPol.vizTrajsValInVal2D(valTrajs[:1],inValTrajsIt[:1],logUn,unsafe=None,state=0,save=True,name="VanderPolValTrajsS0")
        VanderPol.vizTrajsValInVal2D(valTrajs[:1],inValTrajsIt[:1],logUn,unsafe=3,state=1,save=True,name="VanderPolValTrajsS1")

    def checkSafety(initSet,T,unsafe,state,op):
        ts=time.time()
        trajsL=VanderPol.getRandomTrajs(initSet,T,1)
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
                trajs=VanderPol.getRandomTrajs(logUn[0][0],T,100)
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
        
        if len(unsafeSamps)>0:
            VanderPol.vizLogsSafeUnsafe2D(T,safeSamps,unsafeSamps,unsafe,state,save=True,name="VanderPolSafeUnsafeLogs")

        if len(unsafeTrajs)>0 and len(safeTrajs)>0:
            #print("A")
            VanderPol.vizTrajsSafeUnsafe2D([safeTrajs[0]],[unsafeTrajs[0]],safeSamps,unsafeSamps,unsafe,state,save=True,name="VanderPolSafeUnsafeTrajs")
        elif len(safeTrajs)>0 and len(unsafeTrajs)==0:
            #print("B")
            VanderPol.vizTrajsVal2D(safeTrajs,logUn,unsafe,state,save=True,name="VanderPolSafeTrajs")
        elif len(unsafeTrajs)>0:
            #print("C")
            VanderPol.vizTrajsVal2D(unsafeTrajs,logUn,unsafe,state,save=True,name="VanderPolUnsafeTrajs")

    def varyC(initSet,T,unsafe,state,op):
        cList=[0.6,0.7,0.8,0.9,0.99]
        tList=[]
        sList=[]
        for ci in cList:
            print(">> c = ",ci)
            (t,sF)=VanderPol.isSafe(initSet,T,unsafe,state,op,B,ci)
            tList.append(t)
            sList.append(sF)
            print("=====================\n\n")

        print(tList)
        print(sList)
        VanderPol.vizVaryC(cList,sList,tList,save=True,name="VanderPolVaryC")


T=2000
initSet=([1.25,1.45],[2.25,2.35])

#trajs=VanderPol.getRandomTrajs(initSet,T,K)
#VanderPol.vizTrajs(trajs)
#VanderPol.getLog(initSet,T)
#VanderPol.checkSafety(initSet,T)
#VanderPol.getValidTrajs(initSet,T,K,None)


########### Results ########### 

unsafe=2.78
state=1
op='ge'

#VanderPol.showBehavior(initSet,T)
#VanderPol.showLogGeneration(initSet,T)
#VanderPol.showValidTrajs(initSet,T,K=20)
#VanderPol.checkSafety(initSet,T,unsafe,state,op)
#VanderPol.varyC(initSet,T,unsafe,state,op)