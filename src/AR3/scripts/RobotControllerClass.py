#!/usr/bin/env python

class RobotController(object):
    def __init__(self):
        # ROS stuff
        self.AR3Control = AR3_Control()
        self.AR3Angles = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.eStop

        self.AR3ControlPub = rospy.Publisher('/AR3/Control', AR3_Control, queue_size = 1)
        self.AR3FeedbackSub = rospy.Subscriber('/AR3/Feedback', AR3_Feedback, self.AR3FeedbackCallback)

        # Motion parameters
        self.joint_idx = 0
        self.ka = .005

        # Joystick axes and buttons
        self.left_stick_val = 0
        self.left_stick_press = 0
        self.B = 0
        self.A = 0
        self.Y = 0

        t = time.time()
        t_old = time.time()

    def AR3FeedbackCallback(self,data):
        self.AR3Angles = data.joint_angles
        self.eStop = data.eStop
    
    def set_controller_to_AR3(self):
        while (t-t_old) < 4:
            self.AR3Control.joint_angles = self.AR3Angles
            t = time.time()
        self.print_control()

    def home(self):
        self.AR3Control.joint_angles = [0.0,0.0,0.0,0.0,0.0,0.0]

    def joy_callback(self,data):
        self.A = data.buttons[0]
        self.B = data.buttons[1]
        self.Y = data.buttons[3]
        self.left_stick_val = -1.0 * data.axes[0]
        self.left_stick_press = data.buttons[9]

        # Increment the joint being moved based upon left stick press
        if self.A == 1:
            self.joint_idx += 1
            if self.joint_idx > 5:
                self.joint_idx = 0
            self.print_control()
        elif self.B == 1:
            self.joint_idx -= 1
            if self.joint_idx < 0:
                self.joint_idx = 5
            self.print_control()

        if self.Y == 1:
            self.home()


    def stick_move(self):
        # Modify one joint angle based upon
        self.AR3Control.joint_angles[self.joint_idx] = self.AR3Control.joint_angles[self.joint_idx] + (self.left_stick_val*self.ka)

        # Convert to global angle
        if self.AR3Control.joint_angles[self.joint_idx] < 0:
            self.AR3Control.joint_angles[self.joint_idx] = (2*pi) + self.AR3Control.joint_angles[self.joint_idx]
        elif self.AR3Control.joint_angles[self.joint_idx] >= 2*pi:
            self.AR3Control.joint_angles[self.joint_idx] = self.AR3Control.joint_angles[self.joint_idx] - 2*pi
    
    def send_joints(self):
        self.AR3ControlPub.publish(self.AR3Control)

    def clc(self):
        print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
        print('AR3@Controller:$ '),
    
    def print_control(self):
        self.clc()
        print('Moving joint: [{}]'.format(self.joint_idx+1))