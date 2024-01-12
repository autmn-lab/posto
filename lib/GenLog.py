import os,sys,copy

PROJECT_ROOT = os.environ['MNTR_BB_ROOT_DIR']
sys.path.append(PROJECT_ROOT)

from Parameters import *
import random
import numpy as np

class GenLog:
    def __init__(self,traj):
        self.traj=traj;
        self.T=len(self.traj)
        self.nState=len(self.traj[0])
    
    def genLog(self,ep=DELTA_LOG,pr=PROBABILITY_LOG):
        log=[]
        logUn=[]
        for t in range(self.T):
            logFlag=True if random.randint(1, 100)<=pr else False
            if logFlag or t==0:
                log.append((copy.copy(self.traj[t]),t))
                unState=[]
                for st in range(self.nState):
                    #pert=random.uniform(ep,2*ep)
                    #pert=random.uniform(ep/2,ep*2)
                    #pert=random.uniform(ep/2,ep)
                    #pert=np.random.normal(ep-0.001,0.005)
                    pert=ep
                    unState.append([self.traj[t][st]-pert,self.traj[t][st]+pert])
                logUn.append((unState,t))

        return (logUn,log)
    
