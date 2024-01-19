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
import math


import tensorflow as tf
#from tensorflow.keras.models import model_from_yaml,load_model
#from tensorflow.keras import Sequential
import yaml

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d
from tqdm import *

import random

class MC:

    def getNextState(state):
        
        p_cur=copy.copy(state[0])
        v_cur=copy.copy(state[1])

        u=MC.controller(p_cur,v_cur)

        p_next=p_cur+v_cur
        v_next=v_cur+(0.0015*u)-(0.0025*math.cos(3*p_cur))

        nextState=(p_next,v_next)
        return nextState
    
    def getTraj(initState,T):
        traj=[]
        state=copy.copy(initState)
        for t in range(T):
            traj.append(state)
            nextState=MC.getNextState(state)
            state=copy.copy(nextState)
        return traj
    
    def getRandomTrajs(initSet,T,K):
        trajs=[]
        for i in range(K):
            p_init_rand=random.uniform(initSet[0][0], initSet[0][1])
            v_init_rand=random.uniform(initSet[1][0], initSet[1][1])
            traj=MC.getTraj((p_init_rand,v_init_rand),T)
            trajs.append(traj)
        #MC.vizTrajs(trajs)
        return trajs

    def vizTrajs2(trajsVal,logUn=None,unsafe=0.8,goal=0.6,state=0,save=False,name="Untitled"):

        lnWd=2

        t=list(range(len(trajsVal[0])))

        #print(t)
        plt.xlabel("Time",fontsize=20,fontweight='bold')
        plt.ylabel("Position",fontsize=20,fontweight='bold')

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

        #p = plt.plot(t, [unsafe]*len(t),color='red',linewidth=lnWd,linestyle='dashed')
        p = plt.plot(t, [goal]*len(t),color='green',linewidth=lnWd,linestyle='dashed')

        if save:
            plt.savefig(name+".pdf", format="pdf", bbox_inches="tight")
        else:
            plt.show()
        plt.clf()

    def vizTrajs(trajs,logUn=None,save=False,name="Untitled"):

        ax = plt.axes(projection='3d')
        ax.set_xlabel('p',fontsize=20,fontweight='bold')
        ax.set_ylabel('v',fontsize=20,fontweight='bold')
        ax.set_zlabel('time',fontsize=8,fontweight='bold')

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

    def getLog(initSet,T):
        trajs=MC.getRandomTrajs(initSet,T,1)
        
        logger=GenLog(trajs[0])
        log=logger.genLog()

        #MC.vizTrajs(trajs,log[0])

    def getValidTrajs(initSet,T,K,logUn):
        #trajsL=MC.getRandomTrajs(initSet,T,1)
        #logger=GenLog(trajsL[0])
        #logUn=logger.genLog()[0]
        ts=time.time()
        valTrajObj=TrajValidity(logUn)
        valTrajs=[]
        pbar = tqdm(total=K)
        while len(valTrajs)<=K:
            #print("SFSF")
            ts=time.time()
            trajs=MC.getRandomTrajs(logUn[0][0],T,100)
            ts=time.time()-ts
            #print(ts)
            #print("SFSF")
            valTrajsIt,inValTrajsIt=valTrajObj.getValTrajs(trajs)
            valTrajs=valTrajs+valTrajsIt
            #print(len(valTrajs),"/",K)
            pbar.update(len(valTrajsIt))
            if len(valTrajs)>=K:
                pbar.close()
                break
        pbar.close()
        ts=time.time()-ts
        print("Time: ",ts)
        #MC.vizTrajsVal(valTrajs[:2],inValTrajsIt[:2],logUn)
        #VanderPol.vizTrajsVal(valTrajs[:5],inValTrajsIt[:5],logUn)
        #VanderPol.vizTrajsVal2D(valTrajs,logUn,unsafe=2.9,state=0)
        return valTrajs
    
    def checkSafety2(initSet,T):
        trajsL=MC.getRandomTrajs(initSet,T,1)
        print(trajsL[0])
        exit(0)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
        MC.vizTrajs(trajsL,logUn)
        K=20
        ts=time.time()
        unsafe=0.6
        state=0
        op='ge'
        validTrajs=MC.getValidTrajs(initSet,T,K,logUn)
        ts=time.time()-ts
        print("Time taken to generate ", len(validTrajs)," valid trajectories: ",ts)
        
        ts=time.time()
        safeTrajObj=TrajReach([state,op,unsafe])
        (safeTrajs,unsafeTrajs)=safeTrajObj.getSafeUnsafeTrajs(validTrajs)
        #(safeSamps,unsafeSamps)=safeTrajObj.getSafeUnsafeLog(logUn)
        ts=time.time()-ts

        print("[Trajs] Safe, Unsafe: ",len(safeTrajs),len(unsafeTrajs))
        #print("[Log] Safe, Unsafe: ",len(safeSamps),len(unsafeSamps))
        print("Time taken to filter the trajs/logs: ",ts)

        if len(unsafeTrajs)>0 and len(safeTrajs)>0:
            MC.vizTrajsSafeUnsafe([safeTrajs[0]],[unsafeTrajs[0]],logUn,None,unsafe,state)
        

    def vizTrajsSafeUnsafe(safeTrajs,unsafeTrajs,safeSamps,unsafeSamps,unsafe=0.0,state=0):

        lnWd=4

        if len(safeTrajs)>0:
            t=list(range(len(safeTrajs[0])))
        else:
            t=list(range(len(unsafeTrajs[0])))

        #print(t)
        plt.xlabel("Time")
        plt.ylabel("Position")

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

        p = plt.plot(t, [unsafe]*len(t),color='green',linewidth=lnWd,linestyle='dashed')

        plt.show()

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
    
    def vizTrajsVal2(trajsVal,trajsInval,logUn=None,unsafe=0.8,goal=0.6,state=0):

        lnWd=2

        t=list(range(len(trajsVal[0])))

        #print(t)
        plt.xlabel("Time")
        plt.ylabel("Position")

        for traj in trajsVal:
            x=[p[state] for p in traj]
            plt.plot(t,x,linewidth=lnWd,color='blue')
        
        for traj in trajsInval:
            x=[p[state] for p in traj]
            plt.plot(t,x,linewidth=lnWd,color='red')

        if logUn!=None:
            for lg in logUn:
                wd=abs(lg[0][0][1]-lg[0][0][0])
                ht=abs(lg[0][1][1]-lg[0][1][0])
                #print(wd,ht)
                #print([lg[0][0][0], lg[0][0][1]],[lg[1],lg[1]])
                p = plt.plot([lg[1],lg[1]],[lg[0][state][0], lg[0][state][1]], color='black',linewidth=lnWd)

        #p = plt.plot(t, [unsafe]*len(t),color='red',linewidth=lnWd,linestyle='dashed')
        p = plt.plot(t, [goal]*len(t),color='green',linewidth=lnWd,linestyle='dashed')

        plt.show()
    
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
            ax.plot3D(x, y, t,color='red',alpha=0.35)
    
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

    def controllerNN(p,v):
        model = tf.keras.models.Sequential()
        model.add(tf.keras.Input(shape=(2,)))
        layer_h1=tf.keras.layers.Dense(16, activation='sigmoid')
        #layer_h1.set_weights(np.zeros((16,2),dtype=float))
        model.add(layer_h1)
        layer_h2=tf.keras.layers.Dense(16, activation='sigmoid')
        #layer_h2.set_weights(np.zeros((16,16),dtype=float))
        model.add(layer_h2)
        layer_op=tf.keras.layers.Dense(1, activation='tanh')
        #layer_op.set_weights(np.zeros((1,16),dtype=float))
        model.add(layer_op)
        

        #print(layer_h1.weights)
        layer_h1_wt=np.array(
            [[ 0.04227863624691963,
                 -1.9995534420013428],
             [ 0.041098590940237045,
                 -1.9995777606964111],
             [ 0.041627366095781326,
                 -1.9995670318603516],
             [ 0.043073661625385284,
                 -1.9995362758636475],
             [ 0.04422900453209877,
                 -1.9995110034942627],
             [ 0.04332656413316727,
                 -1.9995307922363281],
             [ 0.043973203748464584,
                 1.9995167255401611],
             [ 0.04328867048025131,
                 1.9995317459106445],
             [ 0.04342345520853996,
                 -1.9995288848876953],
             [ 0.043365199118852615,
                 1.9995298385620117],
             [ 0.041691698133945465,
                 -1.999565601348877],
             [ 0.04055285453796387,
                 1.999589204788208],
             [ 0.04333337023854256,
                 1.9995307922363281],
             [ 0.04336114227771759,
                 1.9995300769805908],
             [ 0.043294526636600494,
                 1.9995315074920654],
             [ 0.04001681134104729,
               -1.999599814414978]]
        ).transpose()
        layer_h1_bias=np.array(
            [
                -0.07837510108947754,
                -0.05347610265016556,
                -0.0643407478928566,
                -0.09531356394290924,
                -0.12081193178892136,
                -0.10083713382482529,
                0.07502780109643936,
                0.06033983826637268,
                -0.1028212308883667,
                0.061947017908096313,
                -0.06572159379720688,
                0.0037265445571392775,
                0.06128089502453804,
                0.06189911440014839,
                0.06049951910972595,
                -0.030991174280643463,
            ]
        )
        layer_h1.set_weights([layer_h1_wt,layer_h1_bias])
        #print(layer_h1.weights)

        #print(layer_h2.weights)
        layer_h2_wt=np.array(
            [
                [ 0.5029309391975403
                    ,0.4971928000450134
                    , 0.5099807381629944
                    , 0.5159589052200317
                    , 0.5034105777740479
                    , 0.5188817381858826
                    , -0.4725945293903351
                    , -0.47723013162612915
                    , 0.5193406343460083
                    , -0.4895830750465393
                    , 0.5071413516998291
                    , -0.5271836519241333
                    , -0.48717042803764343
                    , -0.4789147675037384
                    , -0.4904608130455017
                    , 0.49804142117500305
                ],
                [ -0.4724397659301758
                    , -0.461521178483963
                    , -0.46742483973503113
                    , -0.480314701795578
                    , -0.49358582496643066
                    , -0.48407354950904846
                    , 0.5235933661460876
                    , 0.528522789478302
                    , -0.48439428210258484
                    , 0.5294955372810364
                    , -0.46660953760147095
                    , 0.5539658665657043
                    , 0.5285621285438538
                    , 0.530071496963501
                    , 0.5296034812927246
                    , -0.4502568542957306
                ],
                [ 0.4986144006252289
                    , 0.49474331736564636
                    , 0.49822625517845154
                    , 0.5095161199569702
                    , 0.5067102909088135
                    , 0.5056649446487427
                    , -0.48090246319770813
                    , -0.49981144070625305
                    , 0.5069534778594971
                    , -0.4885890781879425
                    , 0.5010932087898254
                    , -0.544204592704773
                    , -0.4945949912071228
                    , -0.4889794588088989
                    , -0.4886799156665802
                    , 0.4896315038204193
                ],
                [ 0.5035915970802307
                    , 0.4987798035144806
                    , 0.504871129989624
                    , 0.5093209147453308
                    , 0.5125168561935425
                    , 0.512190580368042
                    , -0.47623276710510254
                    , -0.48341768980026245
                    , 0.5166367292404175
                    , -0.4810134768486023
                    , 0.503686785697937
                    , -0.5380683541297913
                    , -0.48419779539108276
                    , -0.4866218864917755
                    , -0.49403297901153564
                    , 0.4909047484397888
                ],
                [  -0.489179790019989
                    , -0.47700586915016174
                    , -0.48257336020469666
                    , -0.49788784980773926
                    , -0.5087304711341858
                    , -0.5005542039871216
                    , 0.5013367533683777
                    , 0.5106797814369202
                    , -0.5008606314659119
                    , 0.5093964338302612
                    , -0.4830193519592285
                    , 0.5391728281974792
                    , 0.5098204612731934
                    , 0.5098650455474854
                    , 0.5092957615852356
                    , -0.46613791584968567
                ],
                [ 0.504866361618042
                    , 0.4916672706604004
                    , 0.5001195073127747
                    , 0.5072504281997681
                    , 0.5047734975814819
                    , 0.4977218806743622
                    , -0.4884985387325287
                    , -0.491172730922699
                    , 0.5030825734138489
                    , -0.4907379448413849
                    , 0.5026239156723022
                    , -0.5392987132072449
                    , -0.49660947918891907
                    , -0.4873531460762024
                    , -0.49522754549980164
                    , 0.4967621862888336
                ],
                [ -0.48462000489234924
                    , -0.4722896218299866
                    , -0.47797587513923645
                    , -0.4933628737926483
                    , -0.5049502849578857
                    , -0.49602076411247253
                    , 0.507117509841919
                    , 0.5160872340202332
                    , -0.4960697889328003
                    , 0.514862060546875
                    , -0.47848621010780334
                    , 0.5446606278419495
                    , 0.5151867270469666
                    , 0.5151325464248657
                    , 0.5148177742958069
                    , -0.4616331160068512
                ],
                [ -0.4705827236175537
                    , -0.4634515345096588
                    , -0.4669525623321533
                    , -0.4727509915828705
                    , -0.4767274260520935
                    , -0.47218817472457886
                    , 0.5328171849250793
                    , 0.5277367234230042
                    , -0.47678449749946594
                    , 0.5297655463218689
                    , -0.46857696771621704
                    , 0.5707888603210449
                    , 0.5300502777099609
                    , 0.5288282036781311
                    , 0.53561931848526
                    , -0.4568984806537628
                ],
                [ -0.504328191280365
                    , -0.49198082089424133
                    , -0.49813127517700195
                    , -0.48757484555244446
                    , -0.4942336082458496
                    , -0.4928172528743744
                    , 0.495082288980484
                    , 0.4899129271507263
                    , -0.49341046810150146
                    , 0.49045512080192566
                    , -0.4954262375831604
                    , 0.5661186575889587
                    , 0.5053495764732361
                    , 0.498648077249527
                    , 0.5054664015769958
                    , -0.4858243465423584
                ],
                [ -0.4876706600189209
                    , -0.4753883183002472
                    , -0.4809210002422333
                    , -0.4963958263397217
                    , -0.5075754523277283
                    , -0.4989100694656372
                    , 0.5034935474395752
                    , 0.5127038359642029
                    , -0.49887987971305847
                    , 0.5114561915397644
                    , -0.48151323199272156
                    , 0.5409368276596069
                    , 0.5115599036216736
                    , 0.5115177035331726
                    , 0.5113096237182617
                    , -0.4646050035953522
                ],
                [  -0.48425549268722534
                    , -0.47195327281951904
                    , -0.4775978922843933
                    , -0.4929877817630768
                    , -0.5046226382255554
                    , -0.49572017788887024
                    , 0.5075856447219849
                    , 0.5165019631385803
                    , -0.4957602918148041
                    , 0.5152753591537476
                    , -0.4781234562397003
                    , 0.545036256313324
                    , 0.5156111121177673
                    , 0.5155384540557861
                    , 0.515243649482727
                    , -0.4612559974193573
                ],
                [ -0.4766596555709839
                    , -0.467790424823761
                    , -0.4714244604110718
                    , -0.47550153732299805
                    , -0.47895339131355286
                    , -0.47628071904182434
                    , 0.527580201625824
                    , 0.5219315886497498
                    , -0.4817194640636444
                    , 0.525680422782898
                    , -0.47323137521743774
                    , 0.5682040452957153
                    , 0.5239134430885315
                    , 0.5232277512550354
                    , 0.530324399471283
                    , -0.46253520250320435
                ],
                [ 0.49903392791748047
                    , 0.5007376670837402
                    , 0.5096617341041565
                    , 0.5036900639533997
                    , 0.5099085569381714
                    , 0.5026342272758484
                    , -0.47438889741897583
                    , -0.48922136425971985
                    , 0.5023423433303833
                    , -0.4934077262878418
                    , 0.5092407464981079
                    , -0.5377271175384521
                    , -0.4823279082775116
                    , -0.48593634366989136
                    , -0.4912269413471222
                    , 0.5053555369377136
                ],
                [ 0.49921876192092896
                    , 0.501801609992981
                    , 0.5005511045455933
                    , 0.49568527936935425
                    , 0.4906848669052124
                    , 0.498063325881958
                    , -0.48645466566085815
                    , -0.49718183279037476
                    , 0.5094495415687561
                    , -0.490427166223526
                    , 0.49992161989212036
                    , -0.5544336438179016
                    , -0.4904481768608093
                    , -0.49785518646240234
                    , -0.4901188015937805
                    , 0.49402689933776855
                ],
                [ -0.4971213936805725
                    , -0.49469220638275146
                    , -0.4927273094654083
                    , -0.4965534806251526
                    , -0.49732881784439087
                    , -0.4984211027622223
                    , 0.4963313043117523
                    , 0.49690261483192444
                    , -0.5035313367843628
                    , 0.49743884801864624
                    , -0.49529939889907837
                    , 0.5505218505859375
                    , 0.4945501983165741
                    , 0.49492156505584717
                    , 0.5046933889389038
                    , -0.4859647750854492
                ],
                [ -0.4745565950870514
                    ,-0.4615785479545593
                    , -0.46387916803359985
                    , -0.4830751121044159
                    , -0.4952884614467621
                    , -0.48465436697006226
                    , 0.5246517062187195
                    , 0.5300837755203247
                    , -0.48608654737472534
                    , 0.5270015597343445
                    , -0.4660458564758301
                    , 0.5514376163482666
                    , 0.527611255645752
                    , 0.52862149477005
                    , 0.531677782535553
                    , -0.4482851028442383
                ]
            ]
        ).transpose()
        layer_h2_bias=np.array(
            [
                -0.06690079718828201,
                -0.2791731357574463,
                0.0024476086255162954,
                -0.04407797381281853,
                -0.12216667085886002,
                0.005178603809326887,
                -0.16597121953964233,
                -0.3164622187614441,
                -0.07963834702968597,
                -0.13744662702083588,
                -0.16941888630390167,
                -0.2753027677536011,
                -0.03330422565340996,
                0.025777416303753853,
                -0.060603246092796326,
                -0.2763088047504425,
            ]
        ).transpose()
        layer_h2.set_weights([layer_h2_wt,layer_h2_bias])
        #print(layer_h2.weights)

        #print(layer_op.weights)
        layer_op_wt=np.array(
            [[
                 -4.697135925292969
                , 3.5735561847686768
                , -4.786241054534912
                , -4.771707057952881
                , 5.51308012008667
                , -4.122315406799316
                , 7.20481014251709
                , 3.2560009956359863
                , 2.5152196884155273
                , 5.989198207855225
                , 6.756820201873779
                , 2.804394006729126
                , -3.9697272777557373
                , -4.2557172775268555
                , 3.3061230182647705
                , 3.380592107772827
            ]]
        ).transpose()
        layer_op_bias=np.array([-2.083979368209839])
        layer_op.set_weights([layer_op_wt,layer_op_bias])
        #print(layer_op.weights)

        numpy_array=np.array([
            [p, v]
        ])

        x = tf.convert_to_tensor(numpy_array)
        u = model(x)

        #print(float(y))

        return float(u)
    
    def controller(p,v):
        

        #print(layer_h1.weights)
        layer_h1_wt=np.array(
            [[ 0.04227863624691963,
                 -1.9995534420013428],
             [ 0.041098590940237045,
                 -1.9995777606964111],
             [ 0.041627366095781326,
                 -1.9995670318603516],
             [ 0.043073661625385284,
                 -1.9995362758636475],
             [ 0.04422900453209877,
                 -1.9995110034942627],
             [ 0.04332656413316727,
                 -1.9995307922363281],
             [ 0.043973203748464584,
                 1.9995167255401611],
             [ 0.04328867048025131,
                 1.9995317459106445],
             [ 0.04342345520853996,
                 -1.9995288848876953],
             [ 0.043365199118852615,
                 1.9995298385620117],
             [ 0.041691698133945465,
                 -1.999565601348877],
             [ 0.04055285453796387,
                 1.999589204788208],
             [ 0.04333337023854256,
                 1.9995307922363281],
             [ 0.04336114227771759,
                 1.9995300769805908],
             [ 0.043294526636600494,
                 1.9995315074920654],
             [ 0.04001681134104729,
               -1.999599814414978]]
        )
        layer_h1_bias=np.array(
            [
                -0.07837510108947754,
                -0.05347610265016556,
                -0.0643407478928566,
                -0.09531356394290924,
                -0.12081193178892136,
                -0.10083713382482529,
                0.07502780109643936,
                0.06033983826637268,
                -0.1028212308883667,
                0.061947017908096313,
                -0.06572159379720688,
                0.0037265445571392775,
                0.06128089502453804,
                0.06189911440014839,
                0.06049951910972595,
                -0.030991174280643463,
            ]
        )
        #print(layer_h1.weights)

        #print(layer_h2.weights)
        layer_h2_wt=np.array(
            [
                [ 0.5029309391975403
                    ,0.4971928000450134
                    , 0.5099807381629944
                    , 0.5159589052200317
                    , 0.5034105777740479
                    , 0.5188817381858826
                    , -0.4725945293903351
                    , -0.47723013162612915
                    , 0.5193406343460083
                    , -0.4895830750465393
                    , 0.5071413516998291
                    , -0.5271836519241333
                    , -0.48717042803764343
                    , -0.4789147675037384
                    , -0.4904608130455017
                    , 0.49804142117500305
                ],
                [ -0.4724397659301758
                    , -0.461521178483963
                    , -0.46742483973503113
                    , -0.480314701795578
                    , -0.49358582496643066
                    , -0.48407354950904846
                    , 0.5235933661460876
                    , 0.528522789478302
                    , -0.48439428210258484
                    , 0.5294955372810364
                    , -0.46660953760147095
                    , 0.5539658665657043
                    , 0.5285621285438538
                    , 0.530071496963501
                    , 0.5296034812927246
                    , -0.4502568542957306
                ],
                [ 0.4986144006252289
                    , 0.49474331736564636
                    , 0.49822625517845154
                    , 0.5095161199569702
                    , 0.5067102909088135
                    , 0.5056649446487427
                    , -0.48090246319770813
                    , -0.49981144070625305
                    , 0.5069534778594971
                    , -0.4885890781879425
                    , 0.5010932087898254
                    , -0.544204592704773
                    , -0.4945949912071228
                    , -0.4889794588088989
                    , -0.4886799156665802
                    , 0.4896315038204193
                ],
                [ 0.5035915970802307
                    , 0.4987798035144806
                    , 0.504871129989624
                    , 0.5093209147453308
                    , 0.5125168561935425
                    , 0.512190580368042
                    , -0.47623276710510254
                    , -0.48341768980026245
                    , 0.5166367292404175
                    , -0.4810134768486023
                    , 0.503686785697937
                    , -0.5380683541297913
                    , -0.48419779539108276
                    , -0.4866218864917755
                    , -0.49403297901153564
                    , 0.4909047484397888
                ],
                [  -0.489179790019989
                    , -0.47700586915016174
                    , -0.48257336020469666
                    , -0.49788784980773926
                    , -0.5087304711341858
                    , -0.5005542039871216
                    , 0.5013367533683777
                    , 0.5106797814369202
                    , -0.5008606314659119
                    , 0.5093964338302612
                    , -0.4830193519592285
                    , 0.5391728281974792
                    , 0.5098204612731934
                    , 0.5098650455474854
                    , 0.5092957615852356
                    , -0.46613791584968567
                ],
                [ 0.504866361618042
                    , 0.4916672706604004
                    , 0.5001195073127747
                    , 0.5072504281997681
                    , 0.5047734975814819
                    , 0.4977218806743622
                    , -0.4884985387325287
                    , -0.491172730922699
                    , 0.5030825734138489
                    , -0.4907379448413849
                    , 0.5026239156723022
                    , -0.5392987132072449
                    , -0.49660947918891907
                    , -0.4873531460762024
                    , -0.49522754549980164
                    , 0.4967621862888336
                ],
                [ -0.48462000489234924
                    , -0.4722896218299866
                    , -0.47797587513923645
                    , -0.4933628737926483
                    , -0.5049502849578857
                    , -0.49602076411247253
                    , 0.507117509841919
                    , 0.5160872340202332
                    , -0.4960697889328003
                    , 0.514862060546875
                    , -0.47848621010780334
                    , 0.5446606278419495
                    , 0.5151867270469666
                    , 0.5151325464248657
                    , 0.5148177742958069
                    , -0.4616331160068512
                ],
                [ -0.4705827236175537
                    , -0.4634515345096588
                    , -0.4669525623321533
                    , -0.4727509915828705
                    , -0.4767274260520935
                    , -0.47218817472457886
                    , 0.5328171849250793
                    , 0.5277367234230042
                    , -0.47678449749946594
                    , 0.5297655463218689
                    , -0.46857696771621704
                    , 0.5707888603210449
                    , 0.5300502777099609
                    , 0.5288282036781311
                    , 0.53561931848526
                    , -0.4568984806537628
                ],
                [ -0.504328191280365
                    , -0.49198082089424133
                    , -0.49813127517700195
                    , -0.48757484555244446
                    , -0.4942336082458496
                    , -0.4928172528743744
                    , 0.495082288980484
                    , 0.4899129271507263
                    , -0.49341046810150146
                    , 0.49045512080192566
                    , -0.4954262375831604
                    , 0.5661186575889587
                    , 0.5053495764732361
                    , 0.498648077249527
                    , 0.5054664015769958
                    , -0.4858243465423584
                ],
                [ -0.4876706600189209
                    , -0.4753883183002472
                    , -0.4809210002422333
                    , -0.4963958263397217
                    , -0.5075754523277283
                    , -0.4989100694656372
                    , 0.5034935474395752
                    , 0.5127038359642029
                    , -0.49887987971305847
                    , 0.5114561915397644
                    , -0.48151323199272156
                    , 0.5409368276596069
                    , 0.5115599036216736
                    , 0.5115177035331726
                    , 0.5113096237182617
                    , -0.4646050035953522
                ],
                [  -0.48425549268722534
                    , -0.47195327281951904
                    , -0.4775978922843933
                    , -0.4929877817630768
                    , -0.5046226382255554
                    , -0.49572017788887024
                    , 0.5075856447219849
                    , 0.5165019631385803
                    , -0.4957602918148041
                    , 0.5152753591537476
                    , -0.4781234562397003
                    , 0.545036256313324
                    , 0.5156111121177673
                    , 0.5155384540557861
                    , 0.515243649482727
                    , -0.4612559974193573
                ],
                [ -0.4766596555709839
                    , -0.467790424823761
                    , -0.4714244604110718
                    , -0.47550153732299805
                    , -0.47895339131355286
                    , -0.47628071904182434
                    , 0.527580201625824
                    , 0.5219315886497498
                    , -0.4817194640636444
                    , 0.525680422782898
                    , -0.47323137521743774
                    , 0.5682040452957153
                    , 0.5239134430885315
                    , 0.5232277512550354
                    , 0.530324399471283
                    , -0.46253520250320435
                ],
                [ 0.49903392791748047
                    , 0.5007376670837402
                    , 0.5096617341041565
                    , 0.5036900639533997
                    , 0.5099085569381714
                    , 0.5026342272758484
                    , -0.47438889741897583
                    , -0.48922136425971985
                    , 0.5023423433303833
                    , -0.4934077262878418
                    , 0.5092407464981079
                    , -0.5377271175384521
                    , -0.4823279082775116
                    , -0.48593634366989136
                    , -0.4912269413471222
                    , 0.5053555369377136
                ],
                [ 0.49921876192092896
                    , 0.501801609992981
                    , 0.5005511045455933
                    , 0.49568527936935425
                    , 0.4906848669052124
                    , 0.498063325881958
                    , -0.48645466566085815
                    , -0.49718183279037476
                    , 0.5094495415687561
                    , -0.490427166223526
                    , 0.49992161989212036
                    , -0.5544336438179016
                    , -0.4904481768608093
                    , -0.49785518646240234
                    , -0.4901188015937805
                    , 0.49402689933776855
                ],
                [ -0.4971213936805725
                    , -0.49469220638275146
                    , -0.4927273094654083
                    , -0.4965534806251526
                    , -0.49732881784439087
                    , -0.4984211027622223
                    , 0.4963313043117523
                    , 0.49690261483192444
                    , -0.5035313367843628
                    , 0.49743884801864624
                    , -0.49529939889907837
                    , 0.5505218505859375
                    , 0.4945501983165741
                    , 0.49492156505584717
                    , 0.5046933889389038
                    , -0.4859647750854492
                ],
                [ -0.4745565950870514
                    ,-0.4615785479545593
                    , -0.46387916803359985
                    , -0.4830751121044159
                    , -0.4952884614467621
                    , -0.48465436697006226
                    , 0.5246517062187195
                    , 0.5300837755203247
                    , -0.48608654737472534
                    , 0.5270015597343445
                    , -0.4660458564758301
                    , 0.5514376163482666
                    , 0.527611255645752
                    , 0.52862149477005
                    , 0.531677782535553
                    , -0.4482851028442383
                ]
            ]
        )
        layer_h2_bias=np.array(
            [
                -0.06690079718828201,
                -0.2791731357574463,
                0.0024476086255162954,
                -0.04407797381281853,
                -0.12216667085886002,
                0.005178603809326887,
                -0.16597121953964233,
                -0.3164622187614441,
                -0.07963834702968597,
                -0.13744662702083588,
                -0.16941888630390167,
                -0.2753027677536011,
                -0.03330422565340996,
                0.025777416303753853,
                -0.060603246092796326,
                -0.2763088047504425,
            ]
        ).transpose()
        #print(layer_h2.weights)

        #print(layer_op.weights)
        layer_op_wt=np.array(
            [
                 -4.697135925292969
                , 3.5735561847686768
                , -4.786241054534912
                , -4.771707057952881
                , 5.51308012008667
                , -4.122315406799316
                , 7.20481014251709
                , 3.2560009956359863
                , 2.5152196884155273
                , 5.989198207855225
                , 6.756820201873779
                , 2.804394006729126
                , -3.9697272777557373
                , -4.2557172775268555
                , 3.3061230182647705
                , 3.380592107772827
            ]
        )
        layer_op_bias=np.array([-2.083979368209839])
        #print(layer_op.weights)

        ip=np.array([
            [p, v]
        ])

        #print(layer_h1_wt.transpose().shape,ip.transpose().shape,layer_h1_bias.transpose().shape)
        #print(layer_op_wt.transpose())
        #exit()
        #print(layer_h1_bias.reshape(16,1))
        #print(ip.trasnpose)
        #exit(0)
        #l1_op=np.matmul(layer_h1_wt.transpose(),ip.transpose())
        l1_op=np.matmul(layer_h1_wt,ip.reshape(2,1))+layer_h1_bias.reshape(16,1)
        # apply sigmoid to l1_op
        l1_op_sig=MC.sig(l1_op)
        l2_op=np.matmul(layer_h2_wt,l1_op_sig)+layer_h2_bias.reshape(16,1)
        # apply sigmoid to l2_op
        l2_op_sig=MC.sig(l2_op)
        op=np.matmul(layer_op_wt,l2_op)+layer_op_bias.reshape(1,1)
        # apply tanh to l1_op
        op_tanh=np.tanh(op)
        #print(op.shape)
        #print(op.item())
        #exit()
        #print(float(y))

        return op_tanh.item()
    
    def sig(X):
        l=X.shape[0]
        Y=np.zeros((l,1))
        for i in range(l):
            Y[i]=1/(1 + np.exp(-X[i]))
        return Y 
    
    def showBehavior(initSet,T):
        trajsL=MC.getRandomTrajs(initSet,T,50)
        MC.vizTrajs(trajsL,save=True,name="MCBehavior")

    def showLogGeneration(initSet,T):
        trajsL=MC.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
        MC.vizTrajs(trajsL,logUn,save=True,name="MCLog3D")
        MC.vizTrajsVal2D(trajsL,logUn,unsafe=None,state=0,save=True,name="MCLogS0")
        MC.vizTrajsVal2D(trajsL,logUn,unsafe=0.1,state=1,save=True,name="MCLogS1")

    def showValidTrajs(initSet,T,K):
        trajsL=MC.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
    
        ts=time.time()
        valTrajObj=TrajValidity(logUn)
        valTrajs=[]
        while len(valTrajs)<=K:
            trajs=MC.getRandomTrajs(logUn[0][0],T,10)
            valTrajsIt,inValTrajsIt=valTrajObj.getValTrajs(trajs)
            valTrajs=valTrajs+valTrajsIt
            print(len(valTrajs))
            if len(valTrajs)>=K:
                break
        ts=time.time()-ts
        print("Time taken: ",ts)
        MC.vizTrajsVal(valTrajs[:5],inValTrajsIt[:5],logUn,save=True,name="MCValTrajs")
        MC.vizTrajsValInVal2D(valTrajs[:1],inValTrajsIt[:1],logUn,unsafe=None,state=0,save=True,name="MCValTrajsS0")
        MC.vizTrajsValInVal2D(valTrajs[:1],inValTrajsIt[:1],logUn,unsafe=None,state=1,save=True,name="MCValTrajsS1")

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

    def checkSafety(initSet,T,unsafe,state,op):
        ts=time.time()
        trajsL=MC.getRandomTrajs(initSet,T,1)
        logger=GenLog(trajsL[0])
        logUn=logger.genLog()[0]
        K=JFB(B,c).getNumberOfSamples()
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
                trajs=MC.getRandomTrajs(logUn[0][0],T,100)
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
            MC.vizLogsSafeUnsafe2D(T,safeSamps,unsafeSamps,unsafe,state,save=True,name="VanderPolSafeUnsafeLogs")
        
        if len(unsafeTrajs)>0 and len(safeTrajs)>0:
            #print("A")
            MC.vizTrajsSafeUnsafe2D([safeTrajs[0]],[unsafeTrajs[0]],safeSamps,unsafeSamps,unsafe,state,save=True,name="MCSafeUnsafeTrajs")
        elif len(safeTrajs)>0 and len(unsafeTrajs)==0:
            #print("B")
            MC.vizTrajsVal2D(safeTrajs,logUn,unsafe,state,save=True,name="MCSafeTrajs")
        elif len(unsafeTrajs)>0:
            #print("C")
            MC.vizTrajsVal2D(unsafeTrajs,logUn,unsafe,state,save=True,name="MCUnsafeTrajs")


    def loadControllerYAML(pathMC):
        '''with tf.io.gfile.GFile(pathMC, 'r') as yaml_file:
            yaml_string = yaml_file.read()'''

        with open(pathMC, 'r') as file:
            model_yaml = yaml.safe_load(file)

        yaml_file = open(pathMC, 'r')
        loaded_model_yaml = yaml_file.read()
        yaml_file.close()
        print(loaded_model_yaml)
        loaded_model = model_from_yaml(loaded_model_yaml,custom_objects={'class_name': Sequential})
        #mt=tf.keras.Model.load_model(PROJECT_ROOT+'/src/controllers/'+'eff.ckpt')
        #model_yaml = yaml.safe_load(yaml_string)

        #model=tf.keras.models.load_model(model_yaml)

        # model = tf.keras.models.model_from_yaml(model_yaml,custom_objects=None)

        #print(model_yaml['activations'])
        

        print("UC")



pathMC=PROJECT_ROOT+'/src/controllers/'+'mc.yml'
#p=[-1.2, 0.6]
#v=[-0.07,0.07]
p=[-1.2, -1.0]
v=[-0.07,0.07]
initSet=[p,v]
T=30
#K=10
#n=MC.checkSafety([p,v],T)
#MC.getRandomTrajs([p,v],T,K)
#MC.getLog([p,v],T)
#print(n)


################# Results ################# 

unsafe=0.055
op='ge'
state=1

#MC.showBehavior(initSet,T)
#MC.showLogGeneration(initSet,T)
#MC.showValidTrajs(initSet,T,K=10)
MC.checkSafety(initSet,T,unsafe,state,op)
