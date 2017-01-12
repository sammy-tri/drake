import math

from director import robotstate
from director import drcargs
from director import transformUtils
from director import ikplanner
from director import ikconstraints
from director import visualization as vis
from director import objectmodel as om
from director.debugVis import DebugData
from director import vtkAll as vtk
from director import vtkNumpy as vnp
import numpy as np


def planNominalPosture():

    #ikPlanner.computePostureGoal(startPose, 'General', 'q_nom')

    startPose = robotSystem.planningUtils.getPlanningStartPose()
    endPose = robotSystem.robotStateJointController.getPose('q_nom')
    ikPlanner.computePostureGoal(startPose, endPose)


def getGraspToHandLink():
    config = drcargs.getDirectorConfig()['endEffectorConfig']
    return transformUtils.frameFromPositionAndRPY(
                          config['graspOffsetFrame'][0],
                          np.degrees(config['graspOffsetFrame'][1]))

_callbackId = None

def planReachGoal(goalFrameName='reach goal', interactive=False):

    goalFrame = om.findObjectByName(goalFrameName).transform
    startPoseName = 'reach_start'
    endPoseName = 'reach_end'

    endEffectorLinkName = 'iiwa_link_ee'
    graspOffsetFrame = getGraspToHandLink()


    startPose = robotSystem.planningUtils.getPlanningStartPose()
    ikPlanner.addPose(startPose, startPoseName)

    constraints = []
    constraints.append(ikPlanner.createPostureConstraint(startPoseName, robotstate.matchJoints('base_')))
    p, q = ikPlanner.createPositionOrientationConstraint(endEffectorLinkName, goalFrame, graspOffsetFrame, positionTolerance=0.0, angleToleranceInDegrees=0.0)
    p.tspan = [1.0, 1.0]
    q.tspan = [1.0, 1.0]


    g = ikconstraints.WorldGazeDirConstraint()
    g.linkName = endEffectorLinkName
    g.targetFrame = goalFrame
    g.targetAxis = [0,1,0]
    g.bodyAxis = list(graspOffsetFrame.TransformVector([0,1,0]))
    g.coneThreshold = 0.0
    g.tspan = [1.0, 1.0]


    constraints.append(p)
    constraints.append(g)

    constraintSet = ikplanner.ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)

    global _callbackId
    #if _callbackId:
    #    om.findObjectByName(goalFrameName).disconnectFrameModified(_callbackId)

    if interactive:
        def update(frame):
            endPose, info = constraintSet.runIk()
            robotSystem.teleopPanel.showPose(endPose)

        _callbackId = om.findObjectByName(goalFrameName).connectFrameModified(update)
        update(None)

    else:

        robotSystem.teleopPanel.hideTeleopModel()
        constraintSet.runIk()
        print constraintSet.runIkTraj()


def showDebugPoint(p, name='debug point', update=False, visible=True):
    d = DebugData()
    d.addSphere(p, radius=0.01, color=[1,0,0])
    if update:
        vis.updatePolyData(d.getPolyData(), name)
    else:
        vis.showPolyData(d.getPolyData(), name, colorByName='RGB255', visible=visible)


def makeCylinder():
    ''' has properties Radius and Length '''
    desc = dict(classname='CylinderAffordanceItem',
                Name='cylinder',
                pose=transformUtils.poseFromTransform(vtk.vtkTransform()))
    return newAffordanceFromDescription(desc)


def makeBox():
    ''' has property Dimensions '''
    desc = dict(classname='BoxAffordanceItem',
                Name='box',
                pose=transformUtils.poseFromTransform(vtk.vtkTransform()))
    box = newAffordanceFromDescription(desc)
    box.getChildFrame().setProperty('Scale', 0.1)
    return box


def getMaxZCoordinate(polyData):
    return float(np.nanmax(vnp.getNumpyFromVtk(polyData, 'Points')[:,2]))


def spawnBox():

    t = transformUtils.frameFromPositionAndRPY([0.5,0.0,0.5], [-20,30,0])

    om.removeFromObjectModel(om.findObjectByName('box'))
    obj = makeBox()
    obj.setProperty('Dimensions', [0.06, 0.04, 0.12])
    obj.getChildFrame().copyFrame(t)
    #obj.setProperty('Surface Mode', 'Wireframe')
    obj.setProperty('Color', [1,0,0])


def setGraspTarget(position, rpy, dimensions):

    # At this point, t is a 3-element position: [fwd from robot, left, up] and a quaternion pose describing the rotation of the box.  This works:
    # t = transformUtils.transformFromPose(boxCenter, transformUtils.rollPitchYawToQuaternion([0, 0, 0]))
    # where the quaternion is actually [1, 0, 0, 0]
    rot_quat = transformUtils.rollPitchYawToQuaternion(
        [math.radians(x) for x in rpy])

    #t = transformUtils.transformFromPose([0.81, -0.03, 0.0], [1, 0, 0, 0])
    t = transformUtils.transformFromPose(position, rot_quat)

    om.removeFromObjectModel(om.findObjectByName('box'))
    obj = makeBox()
    obj.setProperty('Dimensions', dimensions)
    obj.getChildFrame().copyFrame(t)
    #obj.getChildFrame().setProperty('Visible', True)
    obj.setProperty('Surface Mode', 'Wireframe')
    obj.setProperty('Color', [1,0,0])



def addGraspFrames():

    obj = om.findObjectByName('box')
    om.removeFromObjectModel(obj.findChild('grasp to world'))
    om.removeFromObjectModel(obj.findChild('pregrasp to world'))

    dims = obj.getProperty('Dimensions')

    objectToWorld = obj.getChildFrame().transform

    graspToObject = transformUtils.frameFromPositionAndRPY([0.0,0.0,dims[2]/2.0 - 0.025], [0,50,0])
    preGraspToGrasp = transformUtils.frameFromPositionAndRPY([-0.08, 0.0, 0.0], [0,0,0])

    graspToWorld = transformUtils.concatenateTransforms([graspToObject, objectToWorld])
    preGraspToWorld = transformUtils.concatenateTransforms([preGraspToGrasp, graspToWorld])

    graspFrame = vis.updateFrame(graspToWorld, 'grasp to world', scale=0.1, parent=obj, visible=False)
    obj.getChildFrame().getFrameSync().addFrame(graspFrame, ignoreIncoming=True)

    preGraspFrame = vis.updateFrame(preGraspToWorld, 'pregrasp to world', scale=0.1, parent=obj, visible=False)
    graspFrame.getFrameSync().addFrame(preGraspFrame, ignoreIncoming=True)


def init(robotSystem_):
    global robotSystem, affordanceManager, ikPlanner, newAffordanceFromDescription

    robotSystem = robotSystem_
    affordanceManager = robotSystem.affordanceManager
    ikPlanner = robotSystem.ikPlanner
    newAffordanceFromDescription = robotSystem.affordanceManager.newAffordanceFromDescription
