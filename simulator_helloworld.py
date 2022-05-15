import numpy as np
import time 
import pybullet_data
from math import sin
import math

from sqlalchemy import true
from grie_simulator import Simu
import pybullet as p
from grie_functions_simulator import *

 
tlimit=15
epochnum=10000000
popunum=20
bestreward=-10
murate=0.00
rewardrecord=[]

#param=np.load('bestparam_hc.npy',allow_pickle='TRUE').item()
param_list=np.load('bestparam_bs_0507.npy',allow_pickle='TRUE').item()
param=param_list[param_list['best_param_index']]
param=param_list[3]
#param={'amp': [0.107,-0.1569,-0.04519],'phase': [4188.4791, 4140.77345, 4206.8966],'devia': [ -0.009, 0.03 ,  -0.0085], 'womiga': 3.578}
#param={'amp': [-0.693310149917933, 1.461347023298298, -1.279729542885898], 'phase': [4213.708291611187, 4167.243764791378, 4233.289489526622], 'devia': [-0.06324882500969893, 0.9615458203954086, -0.2002058065438267], 'womiga': 4.965033858448624}
print(param_list['best_param_index'],param)
sim=Simu(gui=True)
#sim=Simu(gui=False)
sim.initialize()
dt=1./240.
for epoch in range(epochnum):

    newparam=param


    #move to configuration
    tempparam=param.copy()
    tempparam['phase']=[0,0,0]
    config0=param2action(tempparam,0)
    paths=trapezoid_path(0,config0[0],[0.2,0.8,0.2],dt)
    for mi in range(1,6):
        path=trapezoid_path(0,config0[mi],[0.2,0.8,0.2],dt)
        paths=np.vstack((paths,path))
    for ti in range(paths.shape[1]):
        actionarray=paths[:,ti]
        sim.robotcontrol(actionarray)
        sim.step()

    t=-0
    for i in range(10000):
        sim.step()
        actionarray=param2action(newparam,t)
        #actionarray=[0,0,0,0,0,0]
        #print(actionarray)
        if t>0:
            sim.robotcontrol(actionarray)
        else:
            sim.robotcontrol(config0)
        t+=1./240.

        if i%10==9:
            reward,fall=sim.poscheck()
            #print(t,reward,fall)
            if fall:
                reward-=2
                reward+=t*2/5
                break

        if t>tlimit:
            reward,fall=sim.poscheck()
            if not fall:
                reward+=1
            break
    print(t,reward)
    if epoch%100==1:
        print(epoch,reward,bestreward)
    rewardrecord.append(reward)
    if bestreward<reward:
        bestreward=reward
        param=newparam

        #np.save('bestparam_hc.npy',param)
        #np.save('rewardrecord.npy',rewardrecord)
        
    sim.reset()