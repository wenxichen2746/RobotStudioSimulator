import numpy as np
import time 
import pybullet_data
from math import sin
import math

from sqlalchemy import true
from grie_simulator import Simu
import pybullet as p
from grie_functions_simulator import *


 
tlimit=5
epochnum=10000000
popunum=20
bestreward=-10
murate=0.02
rewardrecord=[]

#inital guess
'''
param=np.zeros((3,3))#[param,motor index] Amp, phase shift, deviation
#param[0,:]=np.array([0.1,0.8,0.8])
param[0,:]=np.array([0.8,0.8,0.8])
param[1,:]=np.array([0,0,0])
param[2,:]=np.array([0,-0.1,0.5])#negtive forward,positive backward
womiga=2
'''
param={}

#param={'amp': [-0.693310149917933, 1.461347023298298, -1.279729542885898], 'phase': [4213.708291611187, 4167.243764791378, 4233.289489526622], 'devia': [-0.06324882500969893, 0.9615458203954086, -0.2002058065438267], 'womiga': 4.965033858448624}
param={'amp': [0.107,-0.1569,-0.04519],\
     'phase': [4188.4791, 4140.77345, 4206.8966],\
          'devia': [ -0.009, 0.03 ,  -0.0085], 'womiga': 3.578}

#param=np.load('bestparam_hc0504.npy',allow_pickle='TRUE').item()
print(param)
#sim=Simu(gui=True)
sim=Simu(gui=False)
sim.initialize()
dt=1./240.
for epoch in range(epochnum):
    if np.random.rand()>murate:
        
        newparam=climbparam(param)
    else:
        newparam=guessparam()

    reward=runsim(sim,newparam,tlimit)

    if epoch%100==1:
        print(epoch,reward,bestreward)
    rewardrecord.append(reward)
    if bestreward<reward:
        bestreward=reward
        param=newparam

        np.save('bestparam_hc.npy',param)
        np.save('rewardrecord_hc.npy',rewardrecord)
        print('saved')
    sim.reset()
 