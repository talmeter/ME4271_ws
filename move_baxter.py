#! /usr/bin/env python

import struct
import sys
import rospy
import tf
import numpy

import baxter_interface

from tf import transformations
from baxter_interface import Limb

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from std_msgs.msg import (
    Float64,
)

from sensor_msgs.msg import (
    JointState
)

from baxter_core_msgs.msg import(
    JointCommand,
)

def send_to_joint_vals(q):
    	# Send Baxter to the home configuration
	# create joint commands for home pose
	pub_joint_cmd=rospy.Publisher('/robot/limb/right/joint_command',JointCommand)   # setup the publisher
	command_msg=JointCommand()
	#command_msg.names=['right_s0', 'right_s1', 'right_e0', 'right_e1',  'right_w0', 'right_w1', 'right_w2']
        command_msg.names=['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
	command_msg.command=q
	command_msg.mode=JointCommand.POSITION_MODE
	control_rate = rospy.Rate(100) # sending commands at 100HZ

	# acquire the current joint positions
    	joint_positions=rospy.wait_for_message("/robot/joint_states",JointState)
	qc = joint_positions.position[9:16]
	qc = ([qc[2], qc[3], qc[0], qc[1], qc[4], qc[5], qc[6]]) # reorder due to the odd order that q is read in from the arm

	while not rospy.is_shutdown() and numpy.linalg.norm(numpy.subtract(q,qc))>0.01: # move until the desired joint variable
		pub_joint_cmd.publish(command_msg)	# sends the commanded joint values to Baxter
		control_rate.sleep()                    # sending commands at 100HZ
		joint_positions=rospy.wait_for_message("/robot/joint_states",JointState)
    		qc = (joint_positions.position[9:16])
		#qc = (qc[2], qc[3], qc[0], qc[1], qc[4], qc[5], qc[6])
		#print "joint error = ", numpy.linalg.norm(numpy.subtract(q,qc))
	print("In home pose")
    	return (qc[2], qc[3], qc[0], qc[1], qc[4], qc[5], qc[6]) # reorder due to the odd order that q is read in from the arm


def get_joint_angles(limb,Px,Py,Pz,Qx,Qy,Qz,Qw):
    #rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
    'left': PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x = Px,    #original val:  0.657579481614,
                y = Py,    #original val:  0.851981417433,
                z = Pz,    #original val:  0.0388352386502,
            ),
            orientation=Quaternion(
                x = Qx, #original val: -0.366894936773,
                y = Qy, #original val:  0.885980397775,
                z = Qz, #original val:  0.108155782462,
                w = Qw, #original val:  0.262162481772,
            ),
        ),
    ),
    'right': PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x=Px, #original val:  0.656982770038,
                y=Py, #original val: -0.852598021641,
                z=Pz, #original val:  0.0388609422173,
            ),
            orientation=Quaternion(
                x=Qx, #original val:  0.367048116303,
                y=Qy, #original val:  0.885911751787,
                z=Qz, #original val: -0.108908281936,
                w=Qw, #original val:  0.261868353356,
            ),
        ),
    ),
    }
    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        limb_joints_array = resp.joints[0].position
        print limb_joints
        return limb_joints_array
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    return 0


def makeT(R,t):
	return numpy.vstack((numpy.column_stack((R, numpy.transpose(numpy.array(t)))),[0,0,0,1]))

def main(): 
	print("Initializing node... ")
	rospy.init_node("examples")   # the node's name is examples
        
        first_pose = Limb('right')
        print first_pose.endpoint_pose()
        limb_joints = get_joint_angles('right',0.60979,-0.52214,0.17674,-0.068284,0.48343,-0.13865,0.86163)

        #joint_vars = [limb_joints[5],limb_joints[6],limb_joints[0],limb_joints[1],limb_joints[2],limb_joints[3],limb_joints[4]]
        #send_to_joint_vals(joint_vars)
        
	send_to_joint_vals([0.607839886505127, 0.14994662184448243, 0.1461116698791504, -0.11696603494262696, -0.6036214393432617, -1.5700293346069336, 3.0]) # 
        send_to_joint_vals([0.607839886505127, 0.14994662184448243, 0.1461116698791504, -0.11696603494262696, -0.6036214393432617, -1.5700293346069336, 0.0]) 
	pub_joint_cmd=rospy.Publisher('/robot/limb/right/joint_command',JointCommand)	


if __name__ == '__main__':
    main()



