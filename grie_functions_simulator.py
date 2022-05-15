import numpy as np
from math import sin
def trapezoid_path(ang1,ang2,t_req,dt):


    t_acce=t_req[0]
    t_slow=t_req[1]
    t_mid=t_req[2]
    vmax=(ang2-ang1)/(0.5*t_acce+t_mid+0.5*t_slow)
    ang=[ang1]
    v=[0]

    dt=0.01
    t=0

    while t<t_acce:
        new_v=v[-1]+dt*vmax/t_acce
        v.append(new_v)
        new_ang=ang[-1]+(v[-1]+v[-2])/2*dt
        ang.append(new_ang)
        t+=dt

    while t<t_acce+t_mid:
        v.append(v[-1])
        new_ang=ang[-1]+(v[-1]+v[-2])/2*dt
        ang.append(new_ang) 
        t+=dt

    while t<t_acce+t_mid+t_slow:
        new_v=v[-1]-dt*vmax/t_slow
        v.append(new_v)
        new_ang=ang[-1]+(v[-1]+v[-2])/2*dt
        ang.append(new_ang)
        t+=dt
    return ang

 
def climbparam(param):
    pi=np.random.randint(4)
    di=np.random.randint(3)
    
    if pi ==0:
        param['amp'][di]=param['amp'][di]+np.random.normal(loc=0,scale=0.1)
        param['amp'][di]=max(param['amp'][di],-1)
        param['amp'][di]=min(param['amp'][di],+1)
    if pi ==1:
        param['phase'][di]=param['phase'][di]+np.random.normal(loc=0.0,scale=0.1)
        param['phase'][di]=param['phase'][di]%(2*np.pi)

    if pi ==2:
        param['devia'][di]=param['devia'][di]+np.random.normal(loc=0,scale=0.1)
        param['devia'][di]=max(param['devia'][di],-0.5)
        param['devia'][di]=min(param['devia'][di],+0.5)
    if pi ==3:
        param['womiga']=param['womiga']+np.random.normal(loc=0.05,scale=0.1)
        param['womiga']=max(param['womiga'],0.1)
        param['womiga']=min(param['womiga'],8)
    return param

def param2action(param,t):
    actionarray=np.zeros((6))
    amp=param['amp']
    phase=param['phase']
    devia=param['devia']
    womiga=param['womiga']
    for i in range(3):
        actionarray[i]=amp[i]*sin(womiga*t+phase[i])+devia[i]
    actionarray[3]=amp[0]*sin(womiga*t+phase[0])+devia[0]
    #both hip acting same
    for i in range(1,3):
        actionarray[i+3]=amp[i]*sin(womiga*t+phase[i]+np.pi)+devia[i]
    #two leg act in opposite phase
    return actionarray

def guessparam():
    #inital guess

    param={}
    param['amp']=np.random.uniform(0.1,1.5,size=(3))
    param['phase']=np.random.uniform(-0.2,0.2,size=(3))
    param['devia']=np.random.uniform(-0.1,0.1,size=(3))
    param['womiga']=np.random.uniform(0.5,2,size=(1))
    return param  

def runsim(sim,param,tlimit):
    dt=1./240.
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
        actionarray=param2action(param,t)
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
                reward+=t/5
                break
        if t>tlimit:
            reward,fall=sim.poscheck()
            if not fall:
                reward+=1
            break
        #print(t,)
    #reward+=t/2
    return reward
