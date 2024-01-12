import os,sys,copy

PROJECT_ROOT = os.environ['MNTR_BB_ROOT_DIR']
sys.path.append(PROJECT_ROOT)

from Parameters import *

class TrajSafety:
    def __init__(self,unsafeConstraint):
        self.state=unsafeConstraint[0]
        self.inequal=unsafeConstraint[1]
        self.const=unsafeConstraint[2]

    
    def getSafeUnsafeTrajs(self,trajs):
        safeTrajs=[]
        unsafeTrajs=[]
        for traj in trajs:
            (safeFlag,tViolate)=self.isTrajSafe(traj)
            if safeFlag==False:
                unsafeTrajs.append(traj)
            else:
                safeTrajs.append(traj)
        return (safeTrajs,unsafeTrajs)
    
    def isTrajsSafe(self,trajs):
        for traj in trajs:
            (safeFlag,tViolate)=self.isTrajSafe(traj)
            if safeFlag==False:
                return (traj,tViolate)
        return (True,-1)
    
    def isTrajSafe(self,traj):
        T=len(traj)
        for t in range(T):
            pt=traj[t][self.state]
            if self.inequal=='ge':
                if pt>=self.const:
                    return (False,t)
            elif self.inequal=='le':
                if pt<=self.const:
                    return (False,t)
            else:
                print("FATAL ERROR: Wrong operator!")
                exit(0)
        return (True,-1)

    def getSafeUnsafeLog(self,log):
        safeSamps=[]
        unsafeSamps=[]
        K=len(log)
        for k in range(K):
            sample=log[k]
            if self.isSampleSafe(sample)!=True:
                unsafeSamps.append(sample)
            else:
                safeSamps.append(sample)
        return (safeSamps,unsafeSamps)


    def isLogSafe(self,log):
        K=len(log)
        for k in range(K):
            sample=log[k]
            if self.isSampleSafe(sample)==False:
                return k
        return True

    def isSampleSafe(self,sample):
        sampleTime=sample[1]
        smLb=sample[0][self.state][0]
        smUb=sample[0][self.state][1]
        if self.inequal=='ge':
                if smUb>=self.const:
                    return False
        elif self.inequal=='le':
            if smLb<=self.const:
                return False
        else:
            print("FATAL ERROR: Wrong operator!")
            exit(0)
        return True
                

class TrajReach:
    def __init__(self,unsafeConstraint):
        self.state=unsafeConstraint[0]
        self.inequal=unsafeConstraint[1]
        self.const=unsafeConstraint[2]

    
    def getSafeUnsafeTrajs(self,trajs):
        safeTrajs=[]
        unsafeTrajs=[]
        for traj in trajs:
            (safeFlag,tViolate)=self.isTrajSafe(traj)
            if safeFlag==False:
                unsafeTrajs.append(traj)
            else:
                safeTrajs.append(traj)
        return (safeTrajs,unsafeTrajs)
    
    def isTrajsSafe(self,trajs):
        for traj in trajs:
            (safeFlag,tViolate)=self.isTrajSafe(traj)
            if safeFlag==False:
                return (traj,tViolate)
        return (True,-1)
    
    def isTrajSafe(self,traj):
        T=len(traj)
        #for t in range(T):
        pt=traj[-1][self.state]
        if self.inequal=='ge':
            if pt>=self.const:
                return (True,-1)
        elif self.inequal=='le':
            if pt<=self.const:
                return (True,-1)
        else:
            print("FATAL ERROR: Wrong operator!")
            exit(0)
        return (False,-1)
                
