#!/usr/bin/python

# Behavior GUI

# shell subprocess libs
import subprocess
import shlex  #for converting string commands to tokens
import sys

# GUI libs
import wx
import wx.lib.intctrl
import wx.lib.agw.floatspin

# not sure why these 2 are here
import string
import signal

# ros imports for spawning and killing scout
import rospy
import roslib
import os
roslib.load_manifest("scoutsim")

# scoutsim imports
from scoutsim.srv import *

from collections import defaultdict


class Behaviors(object):
    def __init__(self):
        self.behaviors = []
        self.loadBehaviorList()
    # this takes advantage of the fact that our behavior list start at 1    
    #behaviors = ["Pause", "CW Circle", "CCW Circle", "Odometry",
    #                      "Navigation Map", "Scheduler", "Warehouse",
    #                      "Line Follow", "WL Test", "Maze Solve"]
#    @classmethod
#    def getListFromDir(self):
#        behaviors = []
#        for path, dirname, fnames in sorted(os.walk('../libscout/src')):
#            if 'src/behaviors' in path or 'src/test_behaviors' in path:
#                print "The path is!!!!!!!!!!!!", path
#                path_parts = path.split('/')
#
#                for f in sorted(fnames):
#                    # The pause_scout behavior needs to go first!
#                    print f
#                    if f.endswith('.h') and 'pause_scout' in f:
#                        behaviors.insert(0, f.split('.')[0])
#                    # Everything else goes in alphabetical order
#                    elif f.endswith('.h'):
#                        behaviors.append(f.split('.')[0])
#        return behaviors

    
    def loadBehaviorList(self):
        filename = "behaviorList.txt"
        print "laoding behavior list!!!"
        with open(filename, "r") as f:
            for line in f.read().rstrip().split("\n"):
                self.behaviors+= [line]
    
    def getNumber(self, behaviorName):
        if (behaviorName in self.behaviors):
            return self.behaviors.index(behaviorName)
    
    def getName(self, index):
        if (0 <= index < len(self.Behavior)):
            return self.behaviors[index]
        else:
            return -1
    
    def getBehaviors(self):
        # "Pause" is not considered a behavior in GUI
        return self.behaviors[1:]

# each scout is represented by this class
class Scout(object):
    numOfScouts = 0 # class variable keeping track of scouts
    
    def __init__(self, x=0, y=0, theta=0, name=u"", behaviors=None,
                       Scouts=None, oninit = False):
        Scout.numOfScouts += 1 # reverted if not successful
        if len(name) == 0:
            # automatically give name according to numOfScouts
            name = self.autoNameGen(Scouts)
        # set vars
        self.name = name
        self.behavior = "chilling"
        self.process =  None# used to keep track of all processes
                        # related to this scout  
        self.paused = False
        self.valid = False
        # these vars are right now just the original position
        # but when ODOMETRY is done, we could actually use them
        self.x, self.y = x, y 
        self.theta = theta
        
        self.behaviors = behaviors

        # spawn the scout if it is not scout1 (automatically spawned)
        if (not oninit): 
            if (self.SpawnInSimulator()):
                # spawned correctly
                self.valid = True
            else:
                Scout.numOfScouts -= 1;
        

    def autoNameGen(self, Scouts):
        i = 1
        while (True):
            if not (("scout%d"%i) in Scouts):
                return "scout%d"%i
            i+=1


        # update the classvariable
        

    def SpawnInSimulator(self):
        try:
            # ros spawn routine
            rospy.wait_for_service('/spawn');
            service = rospy.ServiceProxy('/spawn', Spawn);
            response = service(self.x, self.y, self.theta, self.name)
            return True
        except rospy.ServiceException as Error:
            return False
   
    def terminateOldBehavior(self):
        if self.process != None:
            # now terminate the process
            self.process.kill()      # @todo: this kills the process
                                     #      but maybe not the rosnode
                                     #      change to "rosnode kill" instead
            # make sure old behavior is terminated before we run the new one
            self.process.wait()

            # the following lines causes lag and does not solve real issues
            # first unregister the node
            cmd = "rosnode kill /%sBehavior"%self.name
            temp = subprocess.Popen(shlex.split(cmd), shell=False)
            temp.wait() 
        
    def changeBehavior(self, behavior):
        self.behavior = behavior
        if (behavior == "pause"):
            self.paused = True
        else:
            self.paused = False
        self.terminateOldBehavior()
        # do rosprocess calls for new behavior
        roscommand = ("rosrun libscout libscout %s %s"%
                            (self.behaviors.getNumber(self.behavior), self.name))
        self.process = subprocess.Popen(roscommand, shell=True)

    def teleop(self):
        # teleop involved only one command \
        self.terminateOldBehavior()
        self.process = None
        cmd = "rosservice call /set_teleop %s"%self.name
        try:
            subprocess.Popen(shlex.split(cmd), shell=False)
        except:
            #not sure why this is happening....
            # seems to be a ros thing
            print "warning, socket error, ignored"
    
    def sonar_viz(self, on_off):
        # teleop involved only one command \
        cmd = "rosservice call /set_sonar_viz %s %s"%(on_off, self.name)
        try:
            subprocess.Popen(shlex.split(cmd), shell=False)
        except:
            #not sure why this is happening....
            # seems to be a ros thing
            print "warning, socket error, ignored"

    def killSelf(self):
        # terminate its current behavior
        self.terminateOldBehavior()
        
        #terminate old behavior seems to not kill the processes related
        # this brutally kill all the process related to the specific scout
        cmd = "pkill -9 -f %s"%self.name
        subprocess.Popen(shlex.split(cmd), shell=False)

        # ros call to kill scout in simulator
        try:
            rospy.wait_for_service("/kill")
            service = rospy.ServiceProxy("/kill", Kill)
            response = service(self.name)
            Scout.numOfScouts -= 1
            self.valid = False
        except rospy.ServiceException, e:
            print "warning: kill scout unsuccessful" 
            self.valid = True
        # remove from the class


# a wrapper for wx Frame
class Window(wx.App):
    def __init__(self, title=""):
        super(Window, self).__init__()
        self.initUIFrame()

    def initUIFrame(self):
        self.GUIFrame = GUI(None, "hallo!")


# actual GUI frame
class GUI(wx.Frame):
    def __init__(self, parent, title):
        super(GUI, self).__init__(parent, title=title,
            style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER, size=(600, 600))
        
        # open up scoutsim race
        if len(sys.argv) > 1:
            map_name = sys.argv[1]
        else:
            map_name = "race"
        command = shlex.split("rosrun scoutsim scoutsim_node " + map_name)
        self.simWindowProcess = subprocess.Popen(command, shell=False)
        #rospy.wait_for_service('/spawn') #don't know why this is here
        
        # register call back for on close cleanup
        self.Bind(wx.EVT_CLOSE, self.onClose) 


        self.initData()
        self.initUI()
        self.Show(True)     
    
    # do clean up after user close the window
    def onClose(self, event):
        print "Cleaning up all Processes"
        
        # kill all the behavior processes
        for scout in self.scouts:
            process = self.scouts[scout].process
            if (process != None):
                process.terminate()

        # kill the scout sim window
        self.simWindowProcess.terminate()
        print "done"
        self.Destroy()

    def initData(self):
        self.scouts = {}
        #FIXME: these arguments are not right...fix them!!!
        self.behaviors = Behaviors()
        self.scouts["scout1"] = Scout(x=0, y=0, theta=0, name="scout1",
                                      behaviors=self.behaviors,
                                      Scouts = self.scouts, oninit=True)
    
    # addButton callback
    def addScout(self, x_wx, y_wx, theta_wx, name_wx):
        # x, y, theta, name are given as wx Inputs
        x = x_wx.GetValue()
        y = y_wx.GetValue()
        theta = theta_wx.GetValue()
        name = name_wx.GetValue()
        newSc = Scout(x, y, theta, name, self.behaviors, self.scouts)
        if (newSc.valid):
            # successful
            self.scouts[newSc.name] = newSc
            self.addScoutBox(newSc.name)
            # alert user spawn successful
            wx.MessageBox(u"Scout '%s' created successfully at (%d,%d,%d\u00B0)"
                            %(newSc.name, newSc.x, newSc.y, newSc.theta), 
                            "Scout Created", wx.OK | wx.ICON_INFORMATION)

        else:
            #failed to create scout
            wx.MessageBox("Scout Creation failed :( Check for duplicate name",
                            "Add Scout failed", wx.OK | wx.ICON_ERROR)

    def removeScout(self, name):
        sct = self.scouts[name]
        sct.killSelf()
        if (sct.valid == False):
            # successful, remove from the dictionary
            del self.scouts[name]
            # delete the refresh the display
            self.mainArea.Hide(self.sizer[name]) #required by wx before remove
            self.mainArea.Remove(self.sizer[name])
            del self.sizer[name]
            self.window.Layout()
            self.window.Refresh()
            self.window.Update()
        else:
            raise Exception
    
    # runPressed
    # change UI display and invoke change Behavior in scout
    def changeBehavior(self, name, currBehaviorLabel, 
                        pauseButton, newBehavior):
        
        # to handle user pressing "Run" during pause
        if (newBehavior != "Pause" and self.scouts[name].paused == True):
            self.correctPauseButton(name, 
                                        pauseButton, pauseToResume=True)
        
        currBehaviorLabel.SetLabel(" | Current Behavior: %s"%newBehavior)
        scout = self.scouts[name]
        scout.changeBehavior(newBehavior)
        self.correctTeleopButton(None)


    def buttonDown(button):
        return button.GetValue()

    def pauseResumeScout(self, name, pauseButton, currBehaviorLabel, dropMenu):
        if (pauseButton.GetValue() == True): # 
            # change behavior to Pause
            self.changeBehavior(name, currBehaviorLabel, pauseButton, "Pause")    
            # change button to "Resume" & label to "Pause"
            currBehaviorLabel.SetLabel("Pause")
            self.correctPauseButton(name, 
                                        pauseButton, pauseToResume=False)
        else:
            # resume
            # change behavior to whatever is in the drop down menu
            self.changeBehavior(name, currBehaviorLabel, pauseButton, 
                        dropMenu.GetStringSelection())
            self.correctPauseButton(name, 
                                        pauseButton, pauseToResume=True)

    def teleop(self, name):
        self.correctTeleopButton(name)
        self.scouts[name].teleop()

    def sonar_viz(self, name, sonar_vizButton):
        if (sonar_vizButton.GetValue() == True):
            # turn on the sonar viz
            self.scouts[name].sonar_viz("on")
        else:
            # turn off sonar viz
            self.scouts[name].sonar_viz("off")

    ############################ UI stuff here ###########################
    
    def initUI(self):
        self.allTeleopButtons = {}
        self.initAddScoutArea()
    
    def correctTeleopButton(self, teleopingName):
        for name in self.allTeleopButtons:
            if name == teleopingName:
                self.allTeleopButtons[name].SetValue(True)
            else:
                self.allTeleopButtons[name].SetValue(False)
                

    def correctPauseButton(self, name, pauseButton, pauseToResume):
        if (pauseToResume):
            print "correcting button!"
            self.scouts[name].paused = False
            pauseButton.SetValue(False) # unpress it
            pauseButton.SetLabel("Pause")
        else:
            self.scouts[name].paused = True
            pauseButton.SetValue(True) # press it
            pauseButton.SetLabel("Resume")

    # the labels and input boxes for adding a scout through GUI
    def initAddScoutArea(self):
        # all the layout stuff copied over: using grid layout
        # button callbacks are changed
        self.totalCols = 8
        self.window = wx.ScrolledWindow(self, style=wx.VSCROLL)
        self.mainArea = wx.GridSizer(cols=1)
        sizer = wx.FlexGridSizer(rows=4, cols=self.totalCols, hgap=5, vgap=5)

        # Labels
        blankText = wx.StaticText(self.window, label="")
        newScout = wx.StaticText(self.window, label="New Scout")
        newScoutName = wx.StaticText(self.window, label="Name:")
        startXTitle = wx.StaticText(self.window, label="X:")
        startYTitle = wx.StaticText(self.window, label="Y:")
        startThetaTitle = wx.StaticText(self.window, label="Rad:")

        # Inputs
        newScoutInput = wx.TextCtrl(self.window)
        startX = wx.lib.agw.floatspin.FloatSpin(self.window, min_val=0, digits=5)
        startY = wx.lib.agw.floatspin.FloatSpin(self.window, min_val=0, digits=5)
        startTheta = wx.lib.agw.floatspin.FloatSpin(self.window, min_val=0, digits=5)
        addButton = wx.Button(self.window, id=wx.ID_ADD)

        # Pretty Stuff
        hLine = wx.StaticLine(self.window, size=(600, 5))
        bottomHLine = wx.StaticLine(self.window, size=(600, 5))
        
        # Row 0: just the label add scout
        sizer.Add(newScout)
        for i in range(7):
            sizer.AddStretchSpacer(1)
        # Row 1: input(name), x coord, y coord, rad, 
        sizer.AddMany([newScoutName, (newScoutInput, 0, wx.EXPAND), startXTitle,
            startX, startYTitle, startY, startThetaTitle, startTheta])
        # Row 2: just the add button to the right
        for i in range(7):
            sizer.AddStretchSpacer(1)
        sizer.Add(addButton)
        # Row 3
        
        # Events
        addButton.Bind(wx.EVT_BUTTON, lambda event: self.addScout(
            startX, startY, startTheta, newScoutInput))

        sizer.AddGrowableCol(idx=1)
        self.mainArea.Add(sizer, proportion=1, flag=wx.ALL|wx.EXPAND, border=10)
        self.window.SetSizer(self.mainArea)
        self.sizer = defaultdict()
        self.window.Layout()
        
        # make the scout1's controller
        self.addScoutBox("scout1");
    
    

    # copied over from old GUI
    def addScoutBox(self, name):
        self.sizer[name] = wx.FlexGridSizer(rows=2, cols=5, hgap=5, vgap=5)
        # Labels
        scoutName = wx.StaticText(self.window, label="Scout: %s"%name)
        behaviorLabel = wx.StaticText(self.window, label="Behavior: ")
        currBehaviorLabel = wx.StaticText(self.window,
            label="  |  Current Behavior: Slacking off")

        # Inputs
        # drop down menue
        scoutChoices = wx.Choice(self.window, 
                                    choices=self.behaviors.getBehaviors())
        #   buttons
        pauseButton = wx.ToggleButton(self.window, label="Pause")
        runButton = wx.Button(self.window, label="Run")
        killButton = wx.Button(self.window, label="Kill")
        teleopButton = wx.ToggleButton(self.window, label="Teleop")
        sonar_vizButton = wx.ToggleButton(self.window, label="sonar viz")
        self.allTeleopButtons[name] = teleopButton
        # row 0
        self.sizer[name].Add(scoutName)
        self.sizer[name].Add(currBehaviorLabel, wx.EXPAND | wx.ALIGN_RIGHT)
        self.sizer[name].AddStretchSpacer(1)
        self.sizer[name].Add(killButton, wx.ALIGN_RIGHT)
        self.sizer[name].Add(sonar_vizButton)

        # row 1
        self.sizer[name].Add(behaviorLabel)
        self.sizer[name].Add(scoutChoices, wx.EXPAND)
        self.sizer[name].Add(runButton)
        self.sizer[name].Add(pauseButton, wx.ALIGN_RIGHT)
        self.sizer[name].Add(teleopButton) 
        # Events
        killButton.Bind(wx.EVT_BUTTON, lambda event: self.removeScout(name))
        runButton.Bind(wx.EVT_BUTTON,
                lambda event: self.changeBehavior(name, currBehaviorLabel,
                    pauseButton, # needed to handle press "run" during pause
                    scoutChoices.GetStringSelection()))
        pauseButton.Bind(wx.EVT_TOGGLEBUTTON, 
            lambda event: self.pauseResumeScout(name, pauseButton,
                                        currBehaviorLabel, scoutChoices))
        teleopButton.Bind(wx.EVT_TOGGLEBUTTON, 
                                lambda event: self.teleop(name))
        sonar_vizButton.Bind(wx.EVT_TOGGLEBUTTON, 
                                lambda event: 
                                    self.sonar_viz(name, sonar_vizButton))
        self.mainArea.Add(self.sizer[name], proportion=1,
            flag=wx.ALL | wx.EXPAND, border=10)
        self.window.Layout()
        return True



if __name__ == '__main__':
    # open up GUI
    window = Window(title="Colony Scout Manager")
    window.MainLoop()
