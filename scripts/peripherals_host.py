#!/usr/bin/env python
import roslib
roslib.load_manifest('baxter_rr_bridge')
import rospy
import baxter_interface
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16
from std_msgs.msg import Empty
from baxter_core_msgs.msg import SEAJointState

import sys, argparse
import struct
import time
from collections import OrderedDict
import RobotRaconteur as RR
import thread
import threading
import numpy

baxter_servicedef="""
#Service to provide simple interface to Baxter
service BaxterPeripheral_interface

option version 0.4

struct NavigatorState
    field uint8 ok_button
    field uint8 cancel_button
    field uint8 show_button
    field uint8 scroll_wheel
    field uint8 inner_led
    field uint8 outer_led    
end struct

struct SonarPointCloud
    field single[] sensors
    field single[] distances
    field single[] points
end struct

object BaxterPeripherals

function void openGripper(string gripper)
function void closeGripper(string gripper)
function void calibrateGripper(string gripper)
function void setGripperPosition(string gripper, double position)
function void setGripperVelocity(string gripper, double velocity)
function void setGripperHoldForce(string gripper, double force)
function void setGripperMoveForce(string gripper, double force)
function void setGripperDeadband(string gripper, double deadband)
function single getGripperPosition(string gripper)
function single getGripperForce(string gripper)

function single getRangerValue(string arm)

function single getAccelerometerValue(string arm)

function void panHead(double angle)
function single getHeadPanAngle()
function void nodHead()

function void enableSonar()
function void disableSonar()
property SonarPointCloud sonar_pointcloud

function void suppressBodyAvoidance(string limb, uint8 suppress)
function void suppressCollisionAvoidance(string limb, uint8 suppress)
function void suppressContactSafety(string limb, uint8 suppress)
function void suppressCuffInteraction(string limb, uint8 suppress)
function void suppressGravityCompensation(string limb, uint8 suppress)

property double[] gravity_compensation_torques

function NavigatorState getNavigatorState(string navigator)
function void setNavigatorLEDs(string navigator, uint8 inner_led, uint8 outer_led)

end object

"""
class BaxterPeripherals_impl(object):
    def __init__(self):
        print "Initializing Node"
        rospy.init_node('baxter_peripherals')
        
        self._running = True
        self._valid_limb_names = {'left': 'left', 
                                    'l': 'left', 
                                    'right': 'right',
                                    'r': 'right'}
        
        # gripper initialization
        self._grippers = {'left': baxter_interface.Gripper('left'), 
                            'right': baxter_interface.Gripper('right')}
        # Set grippers to defaults
        self._grippers['left'].set_parameters( 
                                self._grippers['left'].valid_parameters())
        self._grippers['right'].set_parameters(
                                self._grippers['right'].valid_parameters())
        
        # ranger initialization
        self._rangers = {'left': baxter_interface.AnalogIO('left_hand_range'), 
                       'right': baxter_interface.AnalogIO('right_hand_range')}
                            
        # accelerometer initialization
        self._accelerometers = {'left': [0.0]*3, 'right': [0.0]*3}
        rospy.Subscriber("/robot/accelerometer/left_accelerometer/state", 
                                                                     Imu, 
                                                self.left_accel_callback)
        rospy.Subscriber("/robot/accelerometer/right_accelerometer/state", 
                                                                      Imu, 
                                                self.right_accel_callback)
        
        # head control initialization
        self._head = baxter_interface.Head()
        
        # sonar initialization
        self._sonar_pointcloud = RR.RobotRaconteurNode.s.NewStructure( 
                          'BaxterPeripheral_interface.SonarPointCloud' )
                                
        self._sonar_state_sub = rospy.Subscriber(
                                        "/robot/sonar/head_sonar/state", 
                                                             PointCloud, 
                                                    self.sonar_callback)
        self._sonar_enable_pub = rospy.Publisher(
                           "/robot/sonar/head_sonar/set_sonars_enabled", 
                                                                 UInt16, 
                                                             latch=True)
        # initially all sonar sensors on
        self._sonar_enabled = True
        self._sonar_enable_pub.publish(4095)
        
        # suppressions
        self._suppress_body_avoidance = {'left': False, 'right': False}
        self._supp_body_avoid_pubs = {'left': 
            rospy.Publisher("/robot/limb/left/suppress_body_avoidance", 
                                                                 Empty, 
                                                           latch=True), 
                                        'right': 
            rospy.Publisher("/robot/limb/right/suppress_body_avoidance", 
                                                                  Empty, 
                                                            latch=True)}
                                        
        self._suppress_collision_avoidance = {'left': False, 'right': False}
        self._supp_coll_avoid_pubs = {'left': 
            rospy.Publisher("/robot/limb/left/suppress_collision_avoidance", 
                                                                      Empty, 
                                                                latch=True), 
                                        'right': 
            rospy.Publisher("/robot/limb/right/suppress_collision_avoidance", 
                                                                       Empty, 
                                                                 latch=True)}
                                        
        self._suppress_contact_safety = {'left': False, 'right': False}
        self._supp_con_safety_pubs = {'left': 
            rospy.Publisher("/robot/limb/left/suppress_contact_safety", 
                                                                 Empty, 
                                                           latch=True), 
                                        'right': 
            rospy.Publisher("/robot/limb/right/suppress_contact_safety", 
                                                                  Empty, 
                                                            latch=True)}
                                        
        self._suppress_cuff_interaction = {'left': False, 'right': False}
        self._supp_cuff_int_pubs = {'left': 
            rospy.Publisher("/robot/limb/left/suppress_cuff_interaction", 
                                                                   Empty, 
                                                             latch=True), 
                                    'right': 
            rospy.Publisher("/robot/limb/right/suppress_cuff_interaction", 
                                                                    Empty, 
                                                              latch=True)}
                                        
        self._suppress_gravity_compensation = {'left': False, 'right': False}
        self._supp_grav_comp_pubs = {'left': 
            rospy.Publisher("/robot/limb/left/suppress_gravity_compensation", 
                                                                       Empty, 
                                                                 latch=True), 
                                     'right': 
            rospy.Publisher("/robot/limb/right/suppress_gravity_compensation", 
                                                                        Empty, 
                                                                  latch=True)}
        
        # start suppressions background thread
        self._t_suppressions = threading.Thread(
                                    target=self.suppressions_worker)
        self._t_suppressions.daemon = True
        self._t_suppressions.start()
        
        # gravity compensation subscription
        self._grav_comp_lock = threading.Lock()
        self._gravity_compensation_torques = OrderedDict( 
                        zip(baxter_interface.Limb('left').joint_names() + \
                                baxter_interface.Limb('right').joint_names(), 
                                                                   [0.0]*14))
        rospy.Subscriber("/robot/limb/left/gravity_compensation_torques", 
                        SEAJointState, self.grav_comp_callback)
        rospy.Subscriber("/robot/limb/right/gravity_compensation_torques", 
                        SEAJointState, self.grav_comp_callback)
        
        # navigators
        self._navigators = {'left': baxter_interface.Navigator('left'), 
                            'right': baxter_interface.Navigator('right'), 
                            'torso_left': 
                                baxter_interface.Navigator('torso_left'), 
                            'torso_right': 
                                baxter_interface.Navigator('torso_right')}
   
    def close(self):
        self._running = False
        self._t_suppressions.join()
    
    # gripper functions
    def openGripper(self, gripper):
        gripper = gripper.lower()
        if gripper in self._valid_limb_names.keys():
            self._grippers[self._valid_limb_names[gripper]].open()
    
    def closeGripper(self, gripper):
        gripper = gripper.lower()
        if gripper in self._valid_limb_names.keys():
            self._grippers[self._valid_limb_names[gripper]].close()
    
    def calibrateGripper(self,gripper):
        gripper = gripper.lower()
        if gripper in self._valid_limb_names.keys():
            self._grippers[self._valid_limb_names[gripper]].calibrate()

    def setGripperPosition(self,gripper,position):
        gripper = gripper.lower()
        if gripper in self._valid_limb_names.keys():
            self._grippers[self._valid_limb_names[gripper]].command_position(
                                                                     position)

    def setGripperVelocity(self,gripper,velocity):
        gripper = gripper.lower()
        if gripper in self._valid_limb_names.keys():
            self._grippers[self._valid_limb_names[gripper]].set_velocity(
                                                                 velocity)

    def setGripperHoldForce(self,gripper,force):
        gripper = gripper.lower()
        if gripper in self._valid_limb_names.keys():
            self._grippers[self._valid_limb_names[gripper]].set_holding_force(
                                                                         force)

    def setGripperMoveForce(self,gripper,force):
        gripper = gripper.lower()
        if gripper in self._valid_limb_names.keys():
            self._grippers[self._valid_limb_names[gripper]].set_moving_force(
                                                                        force)

    def setGripperDeadband(self,gripper,deadband):
        gripper = gripper.lower()
        if gripper in self._valid_limb_names.keys():
            self._grippers[self._valid_limb_names[gripper]].set_dead_band( 
                                                                  deadband)
            
    def getGripperPosition(self, gripper):
        gripper = gripper.lower()
        if gripper in self._valid_limb_names.keys():
            return self._grippers[self._valid_limb_names[gripper]].position()

    def getGripperForce(self, gripper):
        gripper = gripper.lower()
        if gripper in self._valid_limb_names.keys():
            return self._grippers[self._valid_limb_names[gripper]].force()
            
    # Hand rangers
    def getRangerValue(self, arm):
        arm = arm.lower()
        if arm in self._valid_limb_names.keys():
            return self._rangers[self._valid_limb_names[arm]].state()
    
    # Accelerometers
    def getAccelerometerValue(self, arm):
        arm = arm.lower()
        if arm in self._valid_limb_names.keys():
            return self._accelerometers[self._valid_limb_names[arm]]
        
    def left_accel_callback(self, data):
        if (data.linear_acceleration):
            self._accelerometers['left'][0] = data.linear_acceleration.x
            self._accelerometers['left'][1] = data.linear_acceleration.y
            self._accelerometers['left'][2] = data.linear_acceleration.z
        
    def right_accel_callback(self, data):
        if (data.linear_acceleration):
            self._accelerometers['right'][0] = data.linear_acceleration.x
            self._accelerometers['right'][1] = data.linear_acceleration.y
            self._accelerometers['right'][2] = data.linear_acceleration.z
        
    
    # head control functions
    def panHead(self, angle):
        self._head.set_pan(angle)
    
    def getHeadPanAngle(self):
        return self._head.pan()
    
    def nodHead(self):
        self._head.command_nod()
    
    # sonar functions
    @property
    def sonar_pointcloud(self):
        return self._sonar_pointcloud
    
    def sonar_callback(self, data):
            
        if data.points:
            # fill array
            pCloud = []
            for p in data.points:
                pCloud.append(p.x)
                pCloud.append(p.y)
                pCloud.append(p.z)
            
            self._sonar_pointcloud.sensors = tuple(data.channels[0].values)
            self._sonar_pointcloud.distances = tuple(data.channels[1].values)
            self._sonar_pointcloud.points = tuple(pCloud)
        else:
            self._sonar_pointcloud.sensors = None
            self._sonar_pointcloud.distances = None
            self._sonar_pointcloud.points = None

    def enableSonar(self):
        if not self._sonar_enabled:
            self._sonar_enabled = True
            self._sonar_enable_pub.publish(4095)
            self._sonar_state_sub = \
                    rospy.Subscriber("/robot/sonar/head_sonar/state", 
                                                          PointCloud, 
                                                 self.sonar_callback)
    
    def disableSonar(self):
        if self._sonar_enabled:
            self._sonar_enabled = False
            self._sonar_enable_pub.publish(0)
            self._sonar_state_sub.unregister()
    
    # Suppression functions
    def suppressBodyAvoidance(self, limb, suppress):
        limb = limb.lower()
        if limb in self._valid_limb_names.keys():
            if self._suppress_body_avoidance[self._valid_limb_names[limb]] == \
                                                                (suppress > 0):
                return
            self._suppress_body_avoidance[self._valid_limb_names[limb]] = \
                                                            (suppress > 0)
            if self._suppress_body_avoidance[self._valid_limb_names[limb]]:
                print 'Suppressing Body Avoidance for limb ', limb
            else:
                print 'Enabling Body Avoidance for limb ', limb
    
    def suppressCollisionAvoidance(self, limb, suppress):
        limb = limb.lower()
        if limb in self._valid_limb_names.keys():
            if self._suppress_collision_avoidance[ \
                                            self._valid_limb_names[limb]] == \
                                                                (suppress > 0):
                return
            self._suppress_collision_avoidance[ \
                            self._valid_limb_names[limb]] = (suppress > 0)
            if self._suppress_collision_avoidance[ \
                                            self._valid_limb_names[limb]]:
                print 'Suppressing Collision Avoidance for limb ', limb
            else:
                print 'Enabling Collision Avoidance for limb ', limb
    
    def suppressContactSafety(self, limb, suppress):
        limb = limb.lower()
        if limb in self._valid_limb_names.keys():
            if self._suppress_contact_safety[ \
                                            self._valid_limb_names[limb]] == \
                                                                (suppress > 0):
                return
            self._suppress_contact_safety[ \
                            self._valid_limb_names[limb]] = (suppress > 0)
            if self._suppress_contact_safety[self._valid_limb_names[limb]]:
                print 'Suppressing Contact Safety for limb ', limb
            else:
                print 'Enabling Contact Safety for limb ', limb
    
    def suppressCuffInteraction(self, limb, suppress):
        limb = limb.lower()
        if limb in self._valid_limb_names.keys():
            if self._suppress_cuff_interaction[\
                                            self._valid_limb_names[limb]] == \
                                                                (suppress > 0):
                return
            self._suppress_cuff_interaction[self._valid_limb_names[limb]] = \
                                                                (suppress > 0)
            if self._suppress_cuff_interaction[self._valid_limb_names[limb]]:
                print 'Suppressing Cuff Interaction for limb ', limb
            else:
                print 'Enabling Cuff Interaction for limb ', limb
    
    def suppressGravityCompensation(self, limb, suppress):
        limb = limb.lower()
        if limb in self._valid_limb_names.keys():
            if self._suppress_gravity_compensation[\
                                            self._valid_limb_names[limb]] == \
                                                                (suppress > 0):
                return
            self._suppress_gravity_compensation[ \
                                            self._valid_limb_names[limb]] = \
                                                                (suppress > 0)
            if self._suppress_gravity_compensation[ \
                            self._valid_limb_names[limb]]:
                print 'Suppressing Gravity Compensation for limb ', limb
            else:
                print 'Enabling Gravity Compensation for limb ', limb
    
    def publishSuppressions(self, limb):
        if self._suppress_body_avoidance[limb]:
            self._supp_body_avoid_pubs[limb].publish()
            
        if self._suppress_collision_avoidance[limb]:
            self._supp_coll_avoid_pubs[limb].publish()
        
        if self._suppress_contact_safety[limb]:
            self._supp_con_safety_pubs[limb].publish()
        
        if self._suppress_cuff_interaction[limb]:
            self._supp_cuff_int_pubs[limb].publish()
        
        if self._suppress_gravity_compensation[limb]:
            self._supp_grav_comp_pubs[limb].publish()
    
    # worker function to continuously publish suppression commands at >5Hz
    def suppressions_worker(self):
        while self._running:
            time.sleep(0.05)
            self.publishSuppressions('left')
            self.publishSuppressions('right')
    
    # gravity compensation info functions
    @property
    def gravity_compensation_torques(self):
        return self._gravity_compensation_torques.values()
    
    def grav_comp_callback(self, data):
        with self._grav_comp_lock:
            if data.gravity_model_effort:
                for n in xrange(0,len(data.name)):
                    self._gravity_compensation_torques[data.name[n]] = \
                                                data.gravity_model_effort[n]
    
    # navigator functions
    def getNavigatorState(self, navigator):
        if (navigator in self._navigators.keys()):
            navigator_state = RR.RobotRaconteurNode.s.NewStructure(
                                'BaxterPeripheral_interface.NavigatorState')
            navigator_state.ok_button = self._navigators[navigator].button0
            navigator_state.cancel_button = self._navigators[navigator].button1
            navigator_state.show_button = self._navigators[navigator].button2
            navigator_state.scroll_wheel = self._navigators[navigator].wheel
            navigator_state.inner_led = self._navigators[navigator].inner_led
            navigator_state.outer_led = self._navigators[navigator].outer_led
            return navigator_state
        else:
            return None
        
    def setNavigatorLEDs(self, navigator, inner_led, outer_led):
        if (navigator in self._navigators.keys()):
            self._navigators[navigator].inner_led = (inner_led > 0)
            self._navigators[navigator].outer_led = (outer_led > 0)
        
                
    

def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(
                        description='Initialize Baxter Peripherals.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on ' + \
                            '(will auto-generate if not specified)')
    args = parser.parse_args(argv)
    
    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the Node name
    RR.RobotRaconteurNode.s.NodeName="BaxterPeripheralServer"

    
    #Create transport, register it, and start the server
    print "Registering Transport"
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
                         RR.IPNodeDiscoveryFlags_LINK_LOCAL | 
                         RR.IPNodeDiscoveryFlags_SITE_LOCAL)
    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(args.port)
    port = args.port
    if (port == 0):
        port = t.GetListenPort()
    
    #Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(baxter_servicedef)
    
    #Initialize object
    baxter_obj = BaxterPeripherals_impl()    
    
    RR.RobotRaconteurNode.s.RegisterService("BaxterPeripherals",
                 "BaxterPeripheral_interface.BaxterPeripherals",
                                                     baxter_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + \
                    "/BaxterPeripheralServer/BaxterPeripherals"
    raw_input("press enter to quit...\r\n")
    
    baxter_obj.close()

    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
