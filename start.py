import curses
import numpy as np
import json
import matplotlib.pyplot as plt
from robotClass import robot, calcJointVol
 

'''
This project was spawned from a comment I made during an internship that "These Fanuc robots are simple, they're just basic linier algebra"
I decided to test my theory try this out over winter break 2024 and quickly relized it was not just basic linier algebra
3.5 months later and over a dozen code rewrites I have created a 6 axis robot arm that is able smoothly move in cartesian directions as well as create and run saved programs
All parts were printed either on my Elegoo Saturn 2 resin printer or my Bamboo A1 mini based on material strenght and required precision 
This was my first time programming anything longer than ~100 lines long and my first time using python
I hope to continue adding functionality to the arm, I would love to add vision, user frames and non volocity based inverse kinematics
A full redesign of the arm is the next step to remove backlash, hopefully with BLDC motors and precision encoders to increase reliability

Code and arm created by Aaron Buergenthal 3/16/25
Feel free to email me with any question, suggestions or job openings :)
Email Buergenthal.1@osu.edu 

Fun fact this project was entirely deisgned and programmed on an Acer Chromebook
'''


'''
This is the main start program for the 6 axis robot arm routine creation and running
The code defines the robots perameters, creates the gui entity and interprets user inputs and sends to the robot
It outputs robot position to the user and saves programs
'''


class GUI:
    '''
    This class defines the robots GUI
    this includes:
    Vars to define robot movements
    Stores the active screen the user is seeing
    Holds and can edit all the robots known positions and routines 
    '''
    def __init__(self, routines, points, robot):
        '''
        Load all known joint positions from an nx6 numpy array
        Each column is one known position consisting of 6 angles in the rows
        ''' 
        self.knownPositionsPath = f'/home/aaronbuergenthal/vsCode/6axisRobot/data/{points}'
        self.knownPositions = np.load(self.knownPositionsPath).astype(float)
        
        '''
        All routines are stores in a json file
        Routines consist of points associated with a column in the known position array
        '''
        self.programsPath = f"/home/aaronbuergenthal/vsCode/6axisRobot/data/{routines}" 
        with open(self.programsPath,"r" ) as file:
            self.programs = json.load(file)
        
        self.axisJoints   = [['X', 'Y', 'Z', 'W', 'P', 'R'],                                #Array to store cartesian and joint diliniator
                             ['J0','J1','J2','J3','J4','J5']]                               #to refereance when user is moving robot
        
        self.frameWord = "JOINT"                                                            #What frame the robot is being minipulated in
        self.frameNum = 1                                                                   #Used to referance axisJoints row
        self.axis = 0                                                                       #Used to referance axisJoints column
        self.state = "home"                                                                 #What state the gui is in can be home, running, selecting, edit, create
        self.running = True                                                                 #Program is active
        self.rout = [0]                                                                     #Routine being edited, default is 1x1 array at home pos
        self.line = ["Press any key to start"]                                              #Stores current screen displayed to user
        self.next = ""                                                                      #Used if the next state is important (mainly when selecting routines)
        self.userSelection = ""                                                             #String for user numpad input when selecting a routine
        self.userSelectedProgram = ""                                                       #Accepted user input after selection complete
        self.point = 0                                                                      #Current location in routine(used when editing or running program)
        self.looping = False                                                                #True if running a routine cyclicly 

    def saveProgram(self):                                                                  #For saving completed/edited routines to data structure
        
        if self.state == "create": self.programs['arrays'].append(self.rout)                #Add newly created routine to end of the the list of routines
        elif self.state == "edit": self.programs['arrays'][self.userSelectedProgram]        #Replace old routine with newly edited routine in the list of routines
        with open(self.programsPath, "w") as file:                                          #save list of routines to original Json
            json.dump(self.programs, file, indent=4)
    
        np.save(self.knownPositionsPath, self.knownPositions)                               #save known positions np array to npy file
        
        self.line = ["Routine Saved!"]                                                      #Set screen to inform user routines saved sucsessfully
        self.state = "home"                                                                 #Return user home once saved




plot = False                                                    #Plot change if pos/time for joint movements
fNameRoutines = "routines.json"                                 #file name of saved routines
fNamePoints = "known_pos.npy"                                   #file name of known positions
j270 = [0,2,5]                                                #Robot joints with range 0-270
servo_pins = [2, 3, 4, 5, 6, 7]                                 #Servo pins
tool_pin = 11                                                   #Tool pin location
t_move = np.array([[-511],[-511],[-511],[-511],[-511],[-511]])  #Array referanced when EOAT need to move
tool_open = 180                                                 #Servo Pos for open EOAT
tool_close = 135                                                #Servo Pos for closed EOAT
ardWanted = True                                               #True if arduino meant to be connected

fHome = [90.,73.,30.,90.,90.,90.]                               #Motor position at robot at 0,0,0,0,0,0

#Limits in world posisiotn
limits = np.array([[-90,90 ],                                   #Limits for J0 axis
                   [-29,110],                                   #Limits for J1 axis
                   [-45,225],                                   #Limits for J2 axis
                   [-90,90 ],                                   #Limits for J3 axis
                   [-90,90 ],                                   #Limits for J4 axis
                   [-90,90 ]])                                  #Limits for J5 axis


#set up robot pins limits and home
arm = robot(servo_pins,j270, tool_pin, fHome, tool_open,tool_close, ardWanted, limits, plot)



'''
Define DH tables to define the arm mathematicly
This method utilizes relations of each joint relitive to the previous
Where the z axis of a joint is the axis or rotation
And the x axis is always perpendicular to the pervious and current Z axis
For more detailed explination Automatic Addison has great resources on this:
https://automaticaddison.com/how-to-find-denavit-hartenberg-parameter-tables/#Definition_of_the_Parameters
'''

#DH table perameters
j01x = 63.5                                                     #Change in X between J0 and J1 at home
j01z = 67.31                                                    #Change in Z between J0 and J1 at home
j12z = 267.5                                                    #Change in Z between J1 and J2 at home
j23x = 364.                                                     #Change in X between J2 and J3 at home
j34x = 113.6                                                    #Change in X between J3 and J4 at home
j45x = 52.9                                                     #Change in X between J4 and J5 at home
j56x = 100.                                                     #Change in X between J5 and End of EOAT at home

#           [O-J1 , J1-J2,J2-J3 ,J3-J4     ,J4-J5,J5-EOAT   ]
alphaVal =  [90.  , 180. , -90. ,90.      , -90. , 0.       ]   #Angle between Z axis of n-1 to n around X of n
rVal =      [-j01x,-j12z , 0.   , 0.       , 0.  , 0.       ]   #Distance from n-1 to n along axis X of frame n 
dVal =      [j01z , 0.   , 0.   , j23x+j34x, 0.  , j45x+j56x]   #Distance from n-1 to n along axis Z of frame n-1
thetaVals = [180. ,-90.  , 0.   , 0.       , 0.  , 0.       ]   #Angle between X axis of n-1 and x around Z of n-1 

arm.DHdeclare(alphaVal,rVal,dVal,thetaVals)



def checkState(gui):                                            #Sets the screen based on state
    if gui.state == "home":
        gui.rout = [0]
        gui.point = 0
        gui.userSelection = ""
        gui.userSelectedProgram = ""
        homeScreen(gui)
    elif gui.state == "edit":
        editScreen(gui)
    elif gui.state == "create":
        createScreen(gui)
    elif gui.state == "run":
        runScreen(gui)
    elif gui.state == "selecting":
        selectRoutScreen(gui)

def badInpt(gui):                                               #Sets screen to the invalid input screen
    gui.line = ["Invalid Input. Try Again"]

def checkkey(key, gui):                                         #Checks user input based on state most are straight forward
    
    if gui.state == "home":
        if key == ord('c'):
            gui.state = "create"
        elif key == ord('e'):
            gui.state = "selecting"
            gui.next = "edit"
        elif key == ord('r'):
            gui.state = "selecting"
            gui.next = "run"
        elif key == ord('q'):
            gui.running = False                                 #Closes the program
            arm.updateAll(arm.zeroPos)
            return                                              #I don't know why this is here im afraid to remove it tho
        else:
            badInpt(gui)
    
    elif key == ord('q'):                                       #Return home state is q is ever pressed
        gui.state = "home"
        arm.updateAll(arm.zeroPos)
    
    elif gui.state == "create":                                 #All possible user inputs for the create state
        
        if key == ord('e'):                                     #Edit the program being created
            gui.state = "edit"
        elif key == ord('f'):
            changeFrame(gui)
        elif key == ord('g'):
            clawMove(gui)
        elif key == ord(' '):
            savePoint(gui, False)
        elif key == ord('s'):
            gui.saveProgram()
        elif key == curses.KEY_LEFT:
            arrowLeft(gui)
        elif key == curses.KEY_RIGHT:
            arrowRight(gui)
        elif key == curses.KEY_UP:
            arrowUp(gui)
        elif key == curses.KEY_DOWN:
            arrowDown(gui)
        
    elif gui.state == "selecting":                                              #Used to select a program for editing or running
        
        if chr(key).isdigit():                                                  #If user inputed a number append it to user selection string
            gui.userSelection += chr(key)
        elif key == 263 and gui.userSelection != len(""):                       #Backspace to delete last input
            gui.userSelection = gui.userSelection[:-1]
        elif key == 10:                                                         #Enter to check check input is an existing program
            if int(gui.userSelection) in range(len(gui.programs['arrays'])):
                gui.rout = gui.programs['arrays'][int(gui.userSelection)]
                gui.userSelectedProgram = int(gui.userSelection)
                gui.userSelection = ""
                gui.state = gui.next                                            #Set state as either create of run based on next

            else:                                                               #If not a valid program prompt user not valid and restart
                gui.userSelection = ""
                badInpt(gui)
    
    elif gui.state == "edit":                                   #All possible user inputs for the edit state
        
        if key == ord('c'):
            gui.state = "create"                                #Goto create screen with current program 
        elif key == ord('f'):
            changeFrame(gui)
        elif key == ord('g'):
            clawMove(gui)
        elif key == ord(' '):
            savePoint(gui, False)
        elif key == ord("\n"):
            stepUp(gui)
        elif key == ord("\t"):
            stepDown(gui)
        elif key == ord("i"):
            insertPoint(gui)
        elif key == ord("s"):
            gui.saveProgram()
        elif key == curses.KEY_LEFT:
            arrowLeft(gui)
        elif key == curses.KEY_RIGHT:
            arrowRight(gui)
        elif key == curses.KEY_UP:
            arrowUp(gui)
        elif key == curses.KEY_DOWN:
            arrowDown(gui)
        elif chr(key).isdigit():                                #Allows user to input known position into routine (editing the json directly is easier ngl) 
            gui.userSelection += chr(key)
            if int(gui.userSelection) not in range(gui.knownPositions.shape[1]):
                gui.userSelection = ""
        else:
            badInpt(gui)
    
    elif gui.state == "run":                                    #All possible user inputs for the run state
        
        if key == ord(' '):                                     #Begin looping through program
            gui.looping = not gui.looping
            stepProgram(gui)
        elif key == curses.KEY_RIGHT:                           #Step forward in selected program
            if gui.looping == False:
                gui.looping = not gui.looping
                stepProgram(gui)
                gui.looping = not gui.looping
        elif key == curses.KEY_LEFT:                            #Step backward in selected program
            if gui.looping == False:
                gui.point -= 1
                gui.looping = not gui.looping
                stepProgram(gui)
                gui.looping = not gui.looping
        elif key == ord('g'):
            arm.cycleClaw()
    else:                                                       #Really should never be called becuase of the states but just in case
        badInpt(gui)

def homeScreen(gui):
    gui.line    = ["","","",""] 
    gui.line[0] = "What would you like to do?"
    gui.line[1] = "Press C to create a program"
    gui.line[2] = "Press R to run a program"
    gui.line[3] = "Press E to edit a program"

def selectRoutScreen(gui):
    gui.line = ["",""]
    gui.line[0] = f"Enter a number from 1 through {len(gui.programs['arrays'])-1} and press ENTER"
    gui.line[1] = f"Input: {gui.userSelection}"

def createScreen(gui):
    gui.line    = ["","","","","",""]
    gui.line[0] = f"Movement Frame: {gui.frameWord}, Moving {gui.axisJoints[gui.frameNum][gui.axis]} Axis "
    gui.line[1] = f"Current Program: {gui.rout} Mode: {gui.state}"
    gui.line[2] = f"Joint Positions: {arm.getJoints()}"
    gui.line[3] = f"World Cordinate: {arm.getWorldCords()} "
    gui.line[4] =  "F to Change Frame. G to Add a EOAT Move. Q to Quit. E to Edit"
    gui.line[5] =  "SPACE to Save Current Location. S to Save Program"

def editScreen(gui):
    gui.line    = ["","","","","","","",""]
    gui.line[0] = f"Movement Frame: {gui.frameWord}, Moving {gui.axisJoints[gui.frameNum][gui.axis]} Axis "
    gui.line[1] = f"Current Program: {gui.rout} Current Position: {gui.rout[gui.point]} Mode: {gui.state}"
    gui.line[2] = f"Joint Positions: {arm.getJoints()}"
    gui.line[3] = f"World Cordinate: {arm.getWorldCords()} "
    gui.line[4] =  "F to Change Frame. G to Add a EOAT Move. Q to Quit. C to return Create. "
    gui.line[5] =  "ENTER to Cycle Through Positions. Type New Point to Replace With Past Point"
    gui.line[6] =  "SPACE to Write Over Selected Point With Current Position. I to intsert point behind"
    gui.line[7] = f"Current Input: {gui.userSelection}"

def runScreen(gui):
    gui.line    = ["","","",""]
    gui.line[0] = f"Current Program: {gui.rout} Current Position: {gui.point}"
    gui.line[1] = f"World Cordinate: {arm.getWorldCords()} Mode: {gui.state}"
    gui.line[2] =  "SPACE to Play/Pause Program. Q to Quit"
    gui.line[3] =  "ENTER to Step Up. SHIFT to Step Down"

def stepProgram(gui):
    if gui.looping:                                                             #Only do this when program running in some way
        angles = np.array([[0.],[0.],[0.],[0.],[0.],[0.]])
        for i in range(len(angles)):                                            #Put next position in format the arm can use
            angles[i] = gui.knownPositions[i, gui.rout[gui.point]]
        if gui.point in range(len(gui.rout)-1):                                 #Iterate point in routine up or return to start of program
            gui.point += 1
        else:
            gui.point = 0 
        if np.any(angles == -511.):
            arm.cycleClaw()
        else:
            arm.updateAll(angles)                                               #Move robot to new pos

def checkWhenRun(gui, key):                                                     #Check keys when robot running
                                                                                #Needs to be seperate from checkkey because user inputs are not being waited on when bot looping
    if key == ord(' '):                                                         #Pause routine
        gui.looping = False
    elif key == ord('q'):                                                       #Stop running routine
        gui.looping = False
        gui.state = "home"
        arm.updateAll(arm.zeroPos)

def arrowRight(gui):                                                            #Increase movement axis
    if gui.axis < 5:
        gui.axis += 1
    
def arrowLeft(gui):                                                             #deecrease movement axis
    if gui.axis > 0:
        gui.axis -= 1
    
def arrowUp(gui):                                                               #Increase selected axis
    if gui.frameWord == "JOINT":
        arm.incrimentJoint(1, gui.axis)                                         #incriment selected joint by 1 deg
    elif gui.frameWord == "WORLD":
        delta = calcJointVol(gui.axis, 1, arm)                                  #calculate change of joint angles to decrease selected cartesian axis
        for i in range(len(delta)):
            arm.incrimentJoint(delta[i],i)                                      #Move each angle by its delta
    
def arrowDown(gui):                                                             #Decrease selected axis
    if gui.frameWord == "JOINT":
        arm.incrimentJoint(-1, gui.axis)                                        #Decrease selected joint by 1 deg
    elif gui.frameWord == "WORLD":
        delta = calcJointVol(gui.axis, -1,arm)                                  #Calculate change of joint angles to decrease selected cartesian axis
        for i in range(len(delta)):                                             #Move each angle by its delta
            arm.incrimentJoint(delta[i],i)
    
def clawMove(gui):                                                              #Change claw state and add to program
    arm.cycleClaw()                                                             #Claw is bianary: either open or closed
    savePoint(gui,True)

def stepUp(gui):                                                                #Increase selected edit point
    if gui.point < len(gui.rout)-1:
        gui.point += 1
        angles = np.array([[0.],[0.],[0.],[0.],[0.],[0.]])
        for i in range(len(angles)):                                            #Put next position in format the arm can use
            angles[i] = gui.knownPositions[i, gui.rout[gui.point]]
        if np.any(angles == -511.):
            arm.cycleClaw()
        else:
            arm.updateAll(angles)
    
def stepDown(gui):                                                              #Decrease selected edit point
    if gui.point > 0:
        gui.point -= 1
        angles = np.array([[0.],[0.],[0.],[0.],[0.],[0.]])
        for i in range(len(angles)):                                            #Put next position in format the arm can use
            angles[i] = gui.knownPositions[i, gui.rout[gui.point]]
        if np.any(angles == -511.):
            arm.cycleClaw()
        else:
            arm.updateAll(angles)
    

def changeFrame(gui):                                                           #Change frame(World[0] or Joint[1])
    if gui.frameNum == 0:
        gui.frameWord = "JOINT"
        gui.frameNum = 1
    else: 
        gui.frameWord = "WORLD"
        gui.frameNum = 0

def savePoint(gui, claw):                                                       #Save point to routine being created or edited
    
    anglesToStore = np.random.rand(6,1)
    if claw: anglesToStore = t_move                                             #If claw movement save EOAT move array as the angles to be saved
    
    else:                                                                       #If not a claw movement save current pos as the angles to be saved
        anglesFromArm = arm.getrLoc()
        for i in range(len(anglesFromArm)):
            anglesToStore[i] = anglesFromArm[i]
    
    gui.knownPositions = np.hstack((gui.knownPositions, anglesToStore))         #Append it to end of the nx6 known pos array(shouldnt always be happening but I dont care to fix)
    
    if gui.state == "create":                                                   #Append to end of rout if creating new program
        gui.rout.append((gui.knownPositions.shape[1]-1))
    
    elif gui.state == "edit":
        if gui.userSelection != "":                                             #Set editing point to users current input if there is an input
            gui.rout[gui.point] = int(gui.userSelection)
            gui.userSelection = "" 
        else: gui.rout[gui.point] = gui.knownPositions.shape[1]-1               #Set editing point to current pos if no user input
    
    gui.line = ["New Point Saved!"]                                             #Set screen to notify user of saved point
    

def insertPoint(gui):                                                           #Insert a point into routine being eddited
                                                                                #Same method as savePoint but using insert insead of directly setting a point
    anglesFromArm = arm.getrLoc()
    anglesToStore = np.random.rand(6,1)
    for i in range(len(anglesFromArm)):
        anglesToStore[i] = anglesFromArm[i]
    
    gui.knownPositions = np.hstack((gui.knownPositions, anglesToStore))
    
    if gui.userSelection != "":
        gui.rout.insert(gui.point, int(gui.userSelection))
        gui.userSelection = "" 
    
    else:
        gui.rout.insert(gui.point, gui.knownPositions.shape[1]-1)
    gui.line = ["New Point Added!"]
    
    
def print2Screen(lines):                                                        #Print something to the screen
    screen.clear()
    for i in range(len(lines)):
        screen.addstr(i,0, lines[i])
    screen.refresh()

#Initiate the screen
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)

#Create Gui object
gui = GUI(fNameRoutines,fNamePoints, arm)

#Loop while gui is running
while gui.running == True:
    
    if len(gui.line) == 1:                                                      #If only one line on the screen print it and wait for new input before continuing
        print2Screen(gui.line)                                                  #Mostly error or save messages
        screen.nodelay(False)
        userInpt = screen.getch()
    
    checkState(gui)                                                             #Check the state to get updated screen
    print2Screen(gui.line)                                                      #Print the screen to the GUI

    if gui.looping:                                                             #If program looping dont buffer for new user inputs
        screen.nodelay(True)
        while gui.looping:
            checkWhenRun(gui, screen.getch())
            stepProgram(gui)
            checkState(gui)
            print2Screen(gui.line)
        screen.nodelay(False)

    else:                                                                       #Check user inputs with buffer waiting for new input
        screen.nodelay(False)
        userInpt = screen.getch()
        checkkey(userInpt, gui)
    
arm.updateAll(arm.zeroPos)                                                      #Go to robot home pos before shutting down  
