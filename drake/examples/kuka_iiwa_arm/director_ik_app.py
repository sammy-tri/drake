'''
Usage: This program should be launched using the command line specified in the
       kuka_sim.pmd file.
'''

import time

from director import mainwindowapp
import director.objectmodel as om
from director import propertyset
from director import robotsystem
from director import applogic
from director import lcmUtils
from director.tasks import basictasks
from director.tasks.taskuserpanel import TaskUserPanel

from PythonQt import QtGui, QtCore
import drake as lcmdrake

import iiwaplanning
import optitrack_visualizer


class KukaSimInfoLabel(object):
    '''
    Displays simulation time and frequency in the status bar
    using information from the IIWA_STATUS lcm channel.
    '''

    def __init__(self, statusBar):
        self.label = QtGui.QLabel('')
        statusBar.addPermanentWidget(self.label)

        self.sub = lcmUtils.addSubscriber(
            'IIWA_STATUS', lcmdrake.lcmt_iiwa_status, self.onIiwaStatus)
        self.sub.setSpeedLimit(30)

        self.label.text = '[waiting for sim status]'

    def onIiwaStatus(self, msg):
        simTime = msg.utime * 1e-6
        simFreq = self.sub.getMessageRate()
        self.label.text = 'Sim freq: %d hz  |  Sim time: %.2f' % (simFreq,
                                                                  simTime)
class KukaWsgTaskPanel(TaskUserPanel):

    def __init__(self):
        TaskUserPanel.__init__(self, windowTitle='Task Panel')


def sendGripperCommand(targetPositionMM, force):
    msg = lcmdrake.lcmt_schunk_wsg_command()
    msg.utime = int(time.time() * 1e6)
    msg.force = force
    msg.target_position_mm = targetPositionMM
    lcmUtils.publish('SCHUNK_WSG_COMMAND', msg)


def gripperOpen():
    sendGripperCommand(100, 40)


def gripperClose():
    sendGripperCommand(15, 40)

def planGrasp():
    iiwaplanning.planReachGoal('grasp to world', release=False)

def planPreGrasp():
    iiwaplanning.planReachGoal('pregrasp to world', release=False)

def planRelease():
    iiwaplanning.planReachGoal('grasp to world', release=True)

def planPreRelease():
    iiwaplanning.planReachGoal('pregrasp to world', release=True)


class UpdateGraspTargetTask(basictasks.AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty(
            'Position', [0., 0., 0.], attributes=propertyset.PropertyAttributes(
                decimals=3, minimum=-1, maximum=1))
        properties.addProperty(
            'Orientation', [0.0, 0.0, 0.0],
            attributes=propertyset.PropertyAttributes(
                decimals=3, minimum=-360, maximum=360))
        properties.addProperty(
            'Dimensions', [0.1, 0.2, 0.3], attributes=propertyset.PropertyAttributes(
                decimals=3, minimum=0.001, maximum=1))


    def run(self):
        iiwaplanning.setGraspTarget(self.properties.position,
                                    self.properties.orientation,
                                    self.properties.dimensions)
        iiwaplanning.addGraspFrames()

class WaitForExecuteTask(basictasks.DelayTask):
    def __init__(self, robotSystem, **kwargs):
        basictasks.DelayTask.__init__(self, **kwargs)
        self.robotSystem = robotSystem

    def run(self):
        plan = self.robotSystem.ikPlanner.lastManipPlan
        lastPlanTime = self.robotSystem.planPlayback.getPlanElapsedTime(plan)
        self.properties.setProperty('Delay time', 3 * lastPlanTime + 0.1)
        yield basictasks.DelayTask.run(self)


class KukaWsgTaskPanel(TaskUserPanel):

    def __init__(self, robotSystem):
        TaskUserPanel.__init__(self, windowTitle='Task Panel')
        self.ui.imageFrame.hide()
        self.robotSystem = robotSystem
        # Robot toy dimensions
        self._default_target_dimensions = [0.06, 0.02, 0.09]
        # Water bottle dimensions
        #self._default_target_dimensions = [0.07, 0.07, 0.22]
        # Squishy ball dimensions
        #
        # TODO(sam.creasey): Why does the squishy ball generate such
        # weird / bound up plans at position 1?
        #self._default_target_dimensions = [0.06, 0.06, 0.06]


        self.addManualButton('add grasp frames', iiwaplanning.addGraspFrames)
        self.addManualButton('plan pregrasp', planPreGrasp)
        self.addManualButton('plan grasp', planGrasp)

        self.params.addProperty('balls', 3)
        self.addTasks()

    def onPropertyChanged(self, propertySet, propertyName):
        print "property changed", propertyName, propertySet.getProperty(propertyName)

    def commitManipPlan(self):
        self.robotSystem.manipPlanner.commitManipPlan(
            self.robotSystem.ikPlanner.lastManipPlan)

    def addTasks(self):
        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(name, func, parent=None):
            addTask(basictasks.CallbackTask(callback=func, name=name), parent=parent)

        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        addFolder('meh')
        addTask(UpdateGraspTargetTask(name="Update target",
                                      position=[0.81, -0.03, 0.0]))
        addTask(UpdateGraspTargetTask(name="Update target ref",
                                      position=[0.9, 0.0, 0.0],
                                      dimensions=[0.05, 0.05, 0.05]))

        addFolder('pick and place 1->2')
        addTask(UpdateGraspTargetTask(name="Update target 1",
                                      position=[0.8, 0.36, 0.30],
                                      dimensions=self._default_target_dimensions))
        addFunc('plan pregrasp', planPreGrasp)
        addTask(basictasks.DelayTask(name='wait', delayTime=0.25))
        addFunc('execute', self.commitManipPlan)
        addTask(WaitForExecuteTask(self.robotSystem, name='wait for execute'))
        addFunc('plan grasp', planGrasp)
        addTask(basictasks.DelayTask(name='wait', delayTime=0.25))
        addFunc('execute', self.commitManipPlan)
        addTask(WaitForExecuteTask(self.robotSystem, name='wait for execute'))
        addFunc('close gripper', gripperClose)
        addTask(basictasks.DelayTask(name='wait', delayTime=1.0))
        addFunc('plan prerelease', planPreRelease)
        addTask(basictasks.DelayTask(name='wait', delayTime=0.25))
        addFunc('execute', self.commitManipPlan)
        addTask(WaitForExecuteTask(self.robotSystem, name='wait for execute'))
        addTask(UpdateGraspTargetTask(name="Update target 2",
                                      position=[0.9, -0.36, 0.30],
                                      dimensions=self._default_target_dimensions))
        addFunc('plan prerelease', planPreRelease)
        addTask(basictasks.DelayTask(name='wait', delayTime=0.25))
        addFunc('execute', self.commitManipPlan)
        addTask(WaitForExecuteTask(self.robotSystem, name='wait for execute'))
        addFunc('plan release', planRelease)
        addTask(basictasks.DelayTask(name='wait', delayTime=0.25))
        addFunc('execute', self.commitManipPlan)
        addTask(WaitForExecuteTask(self.robotSystem, name='wait for execute'))
        addFunc('open gripper', gripperOpen)

        addFolder('pick and place 2->1')
        addTask(UpdateGraspTargetTask(name="Update target 2",
                                      position=[0.9, -0.36, 0.30],
                                      dimensions=self._default_target_dimensions))
        addFunc('plan pregrasp', planPreGrasp)
        addTask(basictasks.DelayTask(name='wait', delayTime=0.25))
        addFunc('execute', self.commitManipPlan)
        addTask(WaitForExecuteTask(self.robotSystem, name='wait for execute'))
        addFunc('plan grasp', planGrasp)
        addTask(basictasks.DelayTask(name='wait', delayTime=0.25))
        addFunc('execute', self.commitManipPlan)
        addTask(WaitForExecuteTask(self.robotSystem, name='wait for execute'))
        addFunc('close gripper', gripperClose)
        addTask(basictasks.DelayTask(name='wait', delayTime=1.0))
        addFunc('plan prerelease', planPreRelease)
        addTask(basictasks.DelayTask(name='wait', delayTime=0.25))
        addFunc('execute', self.commitManipPlan)
        addTask(WaitForExecuteTask(self.robotSystem, name='wait for execute'))
        addTask(UpdateGraspTargetTask(name="Update target 1",
                                      position=[0.8, 0.36, 0.30],
                                      dimensions=self._default_target_dimensions))
        addFunc('plan prerelease', planPreRelease)
        addTask(basictasks.DelayTask(name='wait', delayTime=0.25))
        addFunc('execute', self.commitManipPlan)
        addTask(WaitForExecuteTask(self.robotSystem, name='wait for execute'))
        addFunc('plan release', planRelease)
        addTask(basictasks.DelayTask(name='wait', delayTime=0.25))
        addFunc('execute', self.commitManipPlan)
        addTask(WaitForExecuteTask(self.robotSystem, name='wait for execute'))
        addFunc('open gripper', gripperOpen)


def onOpenTaskPanel():
    taskPanel.widget.show()
    taskPanel.widget.raise_()

def setupToolbar():
    toolBar = applogic.findToolBar('Main Toolbar')
    app.app.addToolBarAction(
        toolBar, 'Gripper Open', icon='', callback=gripperOpen)
    app.app.addToolBarAction(
        toolBar, 'Gripper Close', icon='', callback=gripperClose)
    app.app.addToolBarAction(
        toolBar, 'Task Panel', icon='', callback=onOpenTaskPanel)


# create a default mainwindow app
app = mainwindowapp.construct(globalsDict=globals())

# load a minimal robot system with ik planning
robotSystem = robotsystem.create(app.view, planningOnly=True)
iiwaplanning.init(robotSystem)

# add the teleop and playback panels to the mainwindow
app.app.addWidgetToDock(robotSystem.teleopPanel.widget,
                        QtCore.Qt.RightDockWidgetArea)
app.app.addWidgetToDock(robotSystem.playbackPanel.widget,
                        QtCore.Qt.BottomDockWidgetArea)

setupToolbar()

taskPanel = KukaWsgTaskPanel(robotSystem)

# show sim time in the status bar
infoLabel = KukaSimInfoLabel(app.mainWindow.statusBar())

# use pydrake ik backend
ikPlanner = robotSystem.ikPlanner
if ikPlanner.planningMode == 'pydrake':
    ikPlanner.plannerPub._setupLocalServer()

# change the default animation mode of the playback panel
robotSystem.playbackPanel.animateOnExecute = True

# disable pointwise ik by default
ikPlanner.getIkOptions().setProperty('Use pointwise', False)
ikPlanner.getIkOptions().setProperty('Max joint degrees/s', 60)

# initialize the listener for the pose gui
ikPlanner.addPostureGoalListener(robotSystem.robotStateJointController)

optitrack_vis = optitrack_visualizer.OptitrackVisualizer('OPTITRACK_FRAMES')

# set the default camera view
applogic.resetCamera(viewDirection=[-1, 0, 0], view=app.view)

# start!
app.app.start()
