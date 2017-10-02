'''
Usage: This program should be launched using the command line specified in the
       kinova_sim.pmd file.
'''

import time

from director import mainwindowapp
from director import robotsystem
from director import applogic
from director import lcmUtils
from PythonQt import QtGui, QtCore
import drake as lcmdrake


class KinovaSimInfoLabel(object):
    '''
    Displays simulation time and frequency in the status bar
    using information from the KINOVA_JACO_STATUS lcm channel.
    '''

    def __init__(self, statusBar):
        self.label = QtGui.QLabel('')
        statusBar.addPermanentWidget(self.label)

        self.sub = lcmUtils.addSubscriber(
            'KINOVA_JACO_STATUS', lcmdrake.lcmt_jaco_status, self.onJacoStatus)
        self.sub.setSpeedLimit(30)

        self.label.text = '[waiting for sim status]'

    def onJacoStatus(self, msg):
        simTime = msg.utime * 1e-6
        simFreq = self.sub.getMessageRate()
        self.label.text = 'Sim freq: %d hz  |  Sim time: %.2f' % (simFreq,
                                                                  simTime)


def gripperOpen():
    pass


def gripperClose():
    pass


def setupToolbar():
    toolBar = applogic.findToolBar('Main Toolbar')
    app.app.addToolBarAction(
        toolBar, 'Gripper Open', icon='', callback=gripperOpen)
    app.app.addToolBarAction(
        toolBar, 'Gripper Close', icon='', callback=gripperClose)


# create a default mainwindow app
app = mainwindowapp.construct(globalsDict=globals())

# load a minimal robot system with ik planning
robotSystem = robotsystem.create(app.view, planningOnly=True)

# add the teleop and playback panels to the mainwindow
app.app.addWidgetToDock(robotSystem.teleopPanel.widget,
                        QtCore.Qt.RightDockWidgetArea)
app.app.addWidgetToDock(robotSystem.playbackPanel.widget,
                        QtCore.Qt.BottomDockWidgetArea)

setupToolbar()

# show sim time in the status bar
infoLabel = KinovaSimInfoLabel(app.mainWindow.statusBar())

# use pydrake ik backend
ikPlanner = robotSystem.ikPlanner
if ikPlanner.planningMode == 'pydrake':
    ikPlanner.plannerPub._setupLocalServer()

# change the default animation mode of the playback panel
robotSystem.playbackPanel.animateOnExecute = True

# disable pointwise ik by default
ikPlanner.getIkOptions().setProperty('Use pointwise', False)
ikPlanner.getIkOptions().setProperty('Max joint degrees/s', 20)

# initialize the listener for the pose gui
ikPlanner.addPostureGoalListener(robotSystem.robotStateJointController)

# set the default camera view
applogic.resetCamera(viewDirection=[-1, 0, 0], view=app.view)

# start!
app.app.start()
