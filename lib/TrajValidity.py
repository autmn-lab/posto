import os,sys,copy

PROJECT_ROOT = os.environ['MNTR_BB_ROOT_DIR']
sys.path.append(PROJECT_ROOT)

from Parameters import *

class TrajValidity:

    def __init__(self,log):
        self.log=log
    
    def getValTrajs(self,trajs):
        '''
        Check if all the trajectories in `traj` 
        touch all the samples in `log` 
        '''
        valTrajs=[]
        inValTrajs=[]
        for traj in trajs:
            #print("A")
            if self.isTrajVal(traj):
                #print(">>>>>>>>>")
                valTrajs.append(traj)
            else:
                inValTrajs.append(traj)
        '''
        print("# Stats from finding valid trajs")
        print("\t* Total Trajs: ",len(trajs))
        print("\t* Val Trajs: ",len(valTrajs))
        '''
        return (valTrajs,inValTrajs)

    def isTrajVal(self,traj):
        nState=len(traj[0])
        T=len(traj)
        logLen=len(self.log)

        for k in range(logLen):
            tk=self.log[k][1]
            sm=traj[tk]
            isSampVal=True
            for i in range(nState):
                if sm[i]<self.log[k][0][i][0] or sm[i]>self.log[k][0][i][1]:
                    isSampVal=False
                    break
            if isSampVal==False:
                #print(sm,self.log[k][0])
                return False
        return True
