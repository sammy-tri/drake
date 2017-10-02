from director import consoleapp
from director import lcmUtils
from director import robotstate

import drake as lcmdrake

# Kinova's JACO urdf's have a 0-114 degree (0-2 radian) range for the
# finger joints, but the SDK's position values range from 0-6800
# degrees (0-118.68 radian).  Convert these as appropriate here.  I
# (sam.creasey) still don't think we wind up with the correct
# simulated position vs. where the actual fingers are at the same
# commanded value, but it's a start.
kFingerSdkToUrdf = 2. / 118.68;


def onJacoStatus(msg):
    q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + list(msg.joint_position) + [x * kFingerSdkToUrdf for x in list(msg.finger_position)]
    stateMsg = robotstate.drakePoseToRobotState(q)
    stateMsg.utime = msg.utime
    lcmUtils.publish('EST_ROBOT_STATE', stateMsg)


subscriber = lcmUtils.addSubscriber(
    'KINOVA_JACO_STATUS', lcmdrake.lcmt_jaco_status, onJacoStatus)
consoleapp.ConsoleApp.start()
