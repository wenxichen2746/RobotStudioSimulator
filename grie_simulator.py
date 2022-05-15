import pybullet as p
import time 
import pybullet_data
from math import sin
import math
import numpy as np
from sqlalchemy import false
from sympy import true

class Simu:
    def __init__(self,gui) -> None:
        self.roboturdf="D:\\2022_Spring_RobotStudio\\RobotSimulator\\urdfmodel\\griebot05022145\\urdf\\griebot05022145.urdf"
        self.gui=gui
        self.cubeStartPos=[0,0,0.6]
        self.cubeStartOrientation=p.getQuaternionFromEuler([0,0,-math.pi/2])
        #print(self.cubeStartOrientation,'ori!!!!!!!')
        pass

    def initialize(self):
        if self.gui:
            physicsClient= p.connect(p.GUI)
        else:
            physicsClient=p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0.2,0,-15.)
        self.planeId=p.loadURDF("plane.urdf")
        
        self.botId=p.loadURDF(self.roboturdf,self.cubeStartPos, self.cubeStartOrientation) 
        #p.createConstraint(self.botId,-1,self.botId,7,p.JOINT_PRISMATIC,\
            #[0,0,1],[0,0,0],[0,0,0])
    def step(self):
        p.stepSimulation()

        if self.gui:
            time.sleep(1./240.)

    def robotcontrol(self,actionarray):
        """
        actionarray 1x6 hip thigh and knee, ankle is calculated 
        """

        for i,mi in enumerate([0,1,2,4,5,6]):
            p.setJointMotorControl2(
            bodyIndex=self.botId,
            jointIndex=mi,
            force=20,
            controlMode=p.POSITION_CONTROL,
            targetPosition=actionarray[i]
            )
        ang3=(actionarray[1]+actionarray[2]+0.1)
        ang7=(actionarray[4]+actionarray[5]+0.1)#replace parrell link

        p.setJointMotorControl2(
        bodyIndex=self.botId,
        jointIndex=3,
        controlMode=p.POSITION_CONTROL,
        targetPosition=ang3
        )   
        p.setJointMotorControl2(
        bodyIndex=self.botId,
        jointIndex=7,
        controlMode=p.POSITION_CONTROL,
        targetPosition=ang7
        )       
    
    def poscheck(self):
        
        cubePos,cubeOrn=p.getBasePositionAndOrientation(self.botId)
        OrnEuler=p.getEulerFromQuaternion(cubeOrn)
        #print(cubeOrn, '!!',OrnEuler)
        xdis=cubePos[0]
        zdis=cubePos[2]
        fall=False
        if zdis<0.2 or OrnEuler[0]>1.7 or OrnEuler[1]>1.7:
            
            fall=True
        return xdis,fall

    def reset(self):
        actionarray=[0,0,0,0,0,0]
        self.robotcontrol(actionarray)
        self.step()
        p.resetBasePositionAndOrientation(self.botId,self.cubeStartPos, self.cubeStartOrientation)
        p.resetBaseVelocity(self.botId, [0, 0, 0], [0, 0, 0])
        for i in range(100):
            p.resetBaseVelocity(self.botId, [0, 0, 0], [0, 0, 0])
            self.step()