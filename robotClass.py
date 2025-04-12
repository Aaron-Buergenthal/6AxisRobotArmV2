import curses
import numpy as np
import sympy as sp
import json
import time 
from pyfirmata import ArduinoMega, Arduino, SERVO
import matplotlib.pyplot as plt

class robot:
    def __init__(self, servoPins, j270s, clawpin, MotorHomePos, cMin, cMax, ard, limits, ploting):
        
        self.ard = ard                                                          #If an Arduino should be connected 
        self.plot = ploting                                                     #For graphical outputs of pos vs time 

        #initilize arduino
        try:
            try: 
                self.board = ArduinoMega('/dev/ttyACM0')
                self.ard = True
            except:
                self.board = ArduinoMega('/dev/ttyACM1')
                self.ard = True
        except: 
            print('no bord found')
            if ard: raise()
        
        self.mPins = servoPins                                                  #Joint pins
        self.cPin = clawpin                                                     #Claw pin
        self.bigMs = j270s                                                      #Motors of 270 deg of range
        
        self.limits = limits                                                    #Motor limits

        self.hLoc = MotorHomePos                                                #Motor pos at 0,0,0
        self.cOpen = cMax                                                       #Tool open
        self.cClose = cMin                                                      #Tool closed

        self.rLoc = np.array([[0.],[0.],[0.],[0.],[0.],[0.]])
        self.zeroOffsets = list(self.hLoc)
        for i in self.bigMs:                                                    #Offsets from motor pos to real 0s
            self.zeroOffsets[i] = self.hLoc[i] * 1.5
        self.home()

    def DHdeclare(self, alphaArray, rArray, dArray, thetaArray):                #Declare valse for DH table
        self.av = list(alphaArray)
        self.rv = list(rArray)
        self.tv = list(thetaArray)
        self.dv = list(dArray)
        for i in range(len(self.av)):                                           #Convert deg to rad
            self.av[i] = self.av[i]*(np.pi/180)


    def getMAngles(self):                                                       #Returns motors angles
        self.mLoc = self.rLoc.tolist()
        for i in range(len(self.mLoc)):
            self.mLoc[i] = self.mLoc[i] + self.zeroOffsets[i]
            if i in self.bigMs: self.mLoc[i] = self.mLoc[i]/1.5

    def home(self):
        if self.ard:
            for i in range(len(self.mPins)):
                self.board.digital[self.mPins[i]].mode = SERVO                  #Initilize all joint servos 
                self.board.digital[self.mPins[i]].write(self.hLoc[i])           #Go to joint home pos 
                time.sleep(0.001)
            self.board.digital[self.cPin].mode = SERVO                          #Initilize claw
            self.board.digital[self.cPin].write(self.cOpen)                     #Open claw 
        self.mLoc = self.hLoc
        self.rLoc = np.array([[0.],[0.],[0.],[0.],[0.],[0.]])                   #Home position
        self.zeroPos = np.array([[0.],[0.],[0.],[0.],[0.],[0.]]) 
        self.clawState = "opn"                                                  #Set claw as Open 

    def cycleClaw(self):
        if self.clawState == "opn":                                             #If claw open, close it
            if self.ard: self.board.digital[self.cPin].write(self.cClose)
            self.clawState = "cls"
        else:                                                                   #If claw closed, open it
            if self.ard: self.board.digital[self.cPin].write(self.cOpen)
            self.clawState = "opn"
        time.sleep(.5)

    def getPos(self):                                                           #Returns all position info 
        return self.rLoc, self.mLoc, self.clawState
    
    def getrLoc(self):                                                          #Returns joints no rounding
        return self.rLoc
    
    def getJoints(self):                                                        #Returns joints rounded 
        return self.rLoc.round(1).flatten()
    
    def getWorldCords(self):                                                    #Returns robots EOAT world position
        self.wPos = self.compHomoMtx()[:3,-1].flatten().round(0)
        return self.wPos
    
    def compHomoMtx(self):                                                      #Computes Homogenous matrix 
        self.flat = [0.,0.,0.,0.,0.,0.]
        for i in range(len(self.flat)):
            self.flat[i] = (self.rLoc.flatten()[i]+self.tv[i])*(np.pi)/180      #Converts to Rad
        self.homoMtx = compute_homogeneous_matrix(self.flat,self.av,self.rv,self.dv)
        return self.homoMtx
    
    def incrimentJoint(self, direction, j):                                     #Increase a joint angle 
        self.move1Joint((self.rLoc[j] + direction), j)

    def move1Joint(self, newAngle, j):                                          #Move 1 joint to a desired pos w/out going past its limits 
        if self.limits[j,0] <= newAngle <= self.limits[j,1]:
            self.rLoc[j] = newAngle
            self.mLoc[j] = self.rLoc[j] + self.zeroOffsets[j]
            if j in self.bigMs: self.mLoc[j] = self.mLoc[j]/1.5
            if self.ard:
                self.board.digital[self.mPins[j]].write(self.mLoc[j])
    
    def updateAll(self, new_angles):                                            #Move all joints to a desired location
        ramp(self.rLoc, new_angles, self)
        self.rLoc = np.array(new_angles) 


vMax = 120                                                                      #min deg/sec from data sheets of all motors
acc = 200
decl = 100
damping_factor=0.001                                                            #Damping val for singularities during volocity cals

#Computations

#Constants and symbols
t,m,a,l,d= sp.symbols('t m a l d')
k1 = ((30*m)/((a)**5))
k2 = (-291600 * m )/ (-9720 * d**5 + 48600 * d**4 * l - 97200 * d**3 * l**2 + 97200 * d**2 * l**3 - 48600 * d * l**4 + 9720 * l**5)

#Acceleration Equations
aEq = k1 * ((t**2) * ((t - a)**2))
lEq = 0
dEq = -((t - l)**2 * (t - d)**2) * k2

#Volocity equations
vAEq = sp.integrate(aEq,  t)
vLEq = sp.integrate(lEq , t) + vAEq.subs(t, a)
vDEq = sp.integrate(dEq , t) - sp.integrate(dEq , (t, 0, l)) + vAEq.subs(t, a)

#Position equations
pAEq = sp.integrate(vAEq , t)
pLEq = sp.integrate(vLEq , t) - pAEq.subs(t, a)
pDEq = sp.integrate(vDEq , t) - sp.integrate(vDEq , (t, 0, l)) + pLEq.subs(t, l)

#Jerk equations
jAEq = aEq.diff(t)
jDEq = dEq.diff(t)

#Minimum angle to move for full acceleration and deceletation
ttA   = vMax/acc                                      
ttL   = ttA
ttD   = vMax/decl +ttA
mMove = pDEq.subs([(a, ttA),(l, ttL),(d,ttD),(t,ttD),(m,vMax)])

#Matrices
def homoMtx(theta, alpha, r, d): #Homogenous matrix j_(n-1) to j_n
    trans = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), r * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), r * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
        ])
    return trans

def rotMtx(theta, alpha):       #Rotational matrix j_(n-1) to j_n
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])

def compute_homogeneous_matrix(angs, alphaArr, rArr, dArr):
    #computes the homogenout matrix for the arm from between joints 0-EOAT
    homgen_matrices = np.eye(4)
    for i in range(len(angs)):
        homgen_matrices = homgen_matrices @ homoMtx(angs[i],alphaArr[i],rArr[i],dArr[i])
    return homgen_matrices


def calcJointVol(axis, direction, robo):
    #Computes the volocity needed for each joint to move the EOAT in a cartesian direction 
    
    homgen_0_6 = robo.compHomoMtx()
    T_0_6 = homgen_0_6[0:3, -1]                                                                 #Current world Pos
    Mi = np.array([0,0,1])

    #Calculate the jacobian matrix for the arm at its current location
    jacobian = np.zeros((len(robo.flat), len(robo.flat)))
    H_0_i = np.eye(4)
    R_0_i = H_0_i[0:3,0:3]
    top = np.cross((R_0_i @ Mi),T_0_6) 
    bot = R_0_i @ Mi
    jacobian[:3,0] = top
    jacobian[3:,0] = bot
    for i in range(len(robo.flat)-1):
        H_0_i = H_0_i @ homoMtx(robo.flat[i],robo.av[i],robo.rv[i],robo.dv[i])
        T_0_i = H_0_i[0:3, -1]
        R_0_i = H_0_i[0:3,0:3]
        top = np.cross((R_0_i @ Mi),T_0_6-T_0_i) 
        bot = R_0_i @ Mi
        jacobian[:3,i+1] = top
        jacobian[3:,i+1] = bot
    
    #Calculate inverse jacobian 
    try: 
        invJac=np.linalg.inv(jacobian)

    except: 
        #If no inverse exists, create a damped jacobian which has an inverse
        U, S, Vt = np.linalg.svd(jacobian)
        S_damped = np.array([s / (s**2 + damping_factor**2) if s > 1e-6 else 0 for s in S])
        invJac = Vt.T @ np.diag(S_damped) @ U.T

    #find the vol w/ matrix multiplication
    volocity = np.array([[0],
                         [0],
                         [0],
                         [0],
                         [0],
                         [0]])
    volocity[axis] = direction
    angleVol= (invJac@volocity)

    #convert from rad/sec to deg 
    angleVol = (angleVol *180)/(np.pi)
    angleVol = angleVol/(np.max(np.abs(angleVol))) # 1 deg/sec of rotation for max.
    angleVol = angleVol*0.5
    return angleVol

'''
Ramp requirements:
    All motors should start moving at the same time
    Max acceleration is defined by mVol
    All motors stop moving at the same time
'''
def ramp(tPos, goto, robo):                                                                                                 #Function for smooth joint movement
    startPos = np.array(tPos)
    #Arrays for plotting change of pos 
    change =[[startPos[0]],[startPos[1]],[startPos[2]],[startPos[3]],[startPos[4]],[startPos[5]]]
    times = [[0.],[0.],[0.],[0.],[0.],[0.]]
    
    deltaThetas = np.array(goto)
    for i in range(len(deltaThetas)):                                                                                       #Calculate the change in each angle
        deltaThetas[i] = goto[i] - tPos[i]
    mDelta = np.abs(deltaThetas).max()                                                                                      #Calculate the largest distance change

    if mDelta != 0.:
        if mDelta < mMove:                                                                                                  #If not enough movement to reach max v find new max v
            mSpeed= max(sp.solve(pDEq.subs([(t, (m/decl)+m/acc),(d,(m/decl)+m/acc),(l, (m/acc)), (a,(m/acc))])-mDelta,m))
            ttA = mSpeed/acc                                                                                                #How long to accelerate
            ttL = ttA                                                                                                       #How long to lin move
            ttD = mSpeed/decl + ttL                                                                                         #How long to decelerate
        else:
            mSpeed = vMax                                                                                                   #Max speed will not be exceeded
            ttA = mSpeed/acc                                                                                                #How long to accelerate
            ttL = ttA + (mDelta-mMove)/mSpeed                                                                               #How long to lin move
            ttD = mSpeed/decl + ttL                                                                                         #How long to decelerate

        #Calculate max speed of each motor based on the move
        mSpeeds = [0., 0., 0., 0., 0., 0.]
        for i in range(len(mSpeeds)):
            if deltaThetas[i] == 0: maxSpeedTemp = 0. 
            else:
                useless = deltaThetas[i]
                useless, maxSpeedTemp = sp.solve(pDEq.subs([(a,ttA),(l,ttL),(d,ttD),(t,ttD)])-deltaThetas[i],m).popitem()
            mSpeeds[i]= maxSpeedTemp

        #Setting up position equations based on the amount of time were moving
        p_at = pAEq.subs([(a,ttA),(l,ttL),(d,ttD)])
        p_lt = pLEq.subs([(a,ttA),(l,ttL),(d,ttD)])
        p_dt = pDEq.subs([(a,ttA),(l,ttL),(d,ttD)])

        startTime = time.time()
        tChange = time.time()-startTime

        #Accelerate Motors 
        while tChange <= ttA:
            for i in range(len(mSpeeds)):
                change[i].append(startPos[i]+(p_at.subs([(t,tChange),(m, mSpeeds[i])])))
                times[i].append(tChange)
                robo.move1Joint(change[i][-1],i)
                tChange = time.time()-startTime

        #linear move Motors
        while tChange <= ttL:
            for i in range(len(mSpeeds)):
                change[i].append((startPos[i])+p_lt.subs([(t,tChange),(m, mSpeeds[i])]))
                robo.move1Joint(change[i][-1],i)
                times[i].append(tChange)
                tChange = time.time()-startTime

        #Decelerate Motors
        while tChange <= ttD:
            for i in range(len(mSpeeds)):
                change[i].append((startPos[i])+p_dt.subs([(t,tChange),(m, mSpeeds[i])]))
                robo.move1Joint(change[i][-1],i)
                times[i].append(tChange)
                tChange = time.time()-startTime

        #Validate they are in the correct location
        for i in range(len(goto)):
            change[i].append(goto[i])
            times[i].append(ttD)
            robo.move1Joint(goto[i],i)

        #plot change in Pos
        if robo.plot:
            labels = ["Theta 1", "Theta 2", "Theta 3", "Theta 4", "Theta 5", "Theta 6"]
            markers = ["o", "s", "d", "^", "v", "x"]
            for i in range(len(times)):
                plt.plot(times[i], change[i], marker=markers[i], label=labels[i])
            plt.xlabel("Time")
            plt.ylabel("Angle (degrees)")
            plt.title(f"{goto} {tPos} {deltaThetas}")
            plt.legend()
            plt.grid(True)
            plt.savefig("plot.png", dpi=300) 
            plt.close()
