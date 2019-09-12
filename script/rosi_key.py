#!/usr/bin/env python
import rospy
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints
from rosi_defy.msg import KeyArray


class RosiNodeClass():

    #TODO: Verificar estes valores
    max_translational_speed = 8 # in [m/s]
    max_rotational_speed = 10 # in [rad/s]
    max_arms_rotational_speed = 0.52 # in [rad/s]


    var_lambda = 0.965
    wheel_radius = 0.1324
    ycir = 0.531

    # construtor
    def __init__(self):

	    # inicializando atributos
        self.omega_left = 0
        self.omega_right = 0
        self.arm_front_rotSpeed = 0
        self.arm_rear_rotSpeed = 0

	    # computing the kinematic A matrix
        self.kin_matrix_A = self.compute_kinematicAMatrix(self.var_lambda, self.wheel_radius, self.ycir)

	    # sends a message to the user
        rospy.loginfo('Rosi_key node started')

	    # registering to publishers
        self.pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
        self.pub_arm = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size=1)

	    # registering to subscribers
        self.sub_key = rospy.Subscriber('/keyboard', KeyArray, self.callback_Key)

	    # defining the eternal loop frequency
        node_sleep_rate = rospy.Rate(10)

	    # eternal loop (until second order)
        while not rospy.is_shutdown():

            arm_command_list = RosiMovementArray()
            traction_command_list = RosiMovementArray()

            # mounting the lists
            for i in range(4):

			    # ----- treating the traction commands
                traction_command = RosiMovement()

                # mount traction command list
                traction_command.nodeID = i+1

                # separates each traction side command
                if i < 2:
                    traction_command.joint_var = self.omega_right
                else:
                    traction_command.joint_var = self.omega_left

                # appending the command to the list
                traction_command_list.movement_array.append(traction_command)

                # ----- treating the arms commands		
                arm_command = RosiMovement()

                # mounting arm command list
                arm_command.nodeID = i+1

                # separates each arm side command
                if i == 0 or i == 2:
                    arm_command.joint_var = self.arm_front_rotSpeed
                else:
                    arm_command.joint_var = self.arm_rear_rotSpeed

                # appending the command to the list
                arm_command_list.movement_array.append(arm_command)

		    # publishing
            self.pub_arm.publish(arm_command_list)		
            self.pub_traction.publish(traction_command_list)

            # sleeps for a while
            node_sleep_rate.sleep()

	    # infinite loop
	    #while not rospy.is_shutdown():
		    # pass

	    # enter in rospy spin
	    #rospy.spin()

    # joystick callback function
    def callback_Key(self, msg):

        comando_lin = msg.keyboardArray[0]
        comando_ang = msg.keyboardArray[1]
        comando_esteiras_f = msg.keyboardArray[2]
        comando_esteiras_t = msg.keyboardArray[3]
        

        # computing desired linear and angular of the robot
        vel_linear_x = self.max_translational_speed * comando_lin
        vel_angular_z = self.max_rotational_speed * comando_ang 

	    # -- computes traction command - kinematic math

	    # b matrix
        b = np.array([[vel_linear_x],[vel_angular_z]])

        # finds the joints control
        x = np.linalg.lstsq(self.kin_matrix_A, b, rcond=-1)[0]

        # query the sides velocities
        self.omega_right = np.deg2rad(x[0][0])
        self.omega_left = np.deg2rad(x[1][0])

        #arms
        self.arm_front_rotSpeed = self.max_arms_rotational_speed * comando_esteiras_f
        self.arm_rear_rotSpeed = self.max_arms_rotational_speed * comando_esteiras_t
            
    # ---- Support Methods --------

    # -- Method for compute the skid-steer A kinematic matrix
    @staticmethod
    def compute_kinematicAMatrix(var_lambda, wheel_radius, ycir):

	    # kinematic A matrix 
        matrix_A = np.array([[var_lambda*wheel_radius/2, var_lambda*wheel_radius/2], [(var_lambda*wheel_radius)/(2*ycir), -(var_lambda*wheel_radius)/(2*ycir)]])

        return matrix_A

# instaciate the node
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('rosi_example_node', anonymous=True)

	# instantiate the class
	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException: pass
