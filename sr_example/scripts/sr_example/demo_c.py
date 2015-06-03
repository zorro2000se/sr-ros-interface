#!/usr/bin/env python

import rospy
import random
from sr_robot_commander.sr_hand_commander import SrHandCommander
import time

rospy.init_node("basic_hand_examples", anonymous=True)

hand_commander = SrHandCommander(name="right_hand", prefix="rh")


max_random_interpolation_time = 4.0

##########
# RANGES #
##########

# Minimum alllowed range for the joints in this particular script
min_range = {"THJ2": -40, "THJ3": -12, "THJ4": 0, "THJ5": -55,
             "FFJ0": 20, "FFJ3": 0, "FFJ4": -20,
             "MFJ0": 20, "MFJ3": 0, "MFJ4": -10,
             "RFJ0": 20, "RFJ3": 0, "RFJ4": -10,
             "LFJ0": 20, "LFJ3": 0, "LFJ4": -20, "LFJ5": 0,
             "WRJ1": -20, "WRJ2": -10}

# Maximum alllowed range for the joints in this particular script
max_range = {"THJ2": 20, "THJ3": 12, "THJ4": 70, "THJ5": 0,
             "FFJ0": 110, "FFJ3": 90, "FFJ4": 0,
             "MFJ0": 110, "MFJ3": 90, "MFJ4": 0,
             "RFJ0": 110, "RFJ3": 90, "RFJ4": 0,
             "LFJ0": 110, "LFJ3": 90, "LFJ4": 0, "LFJ5": 1,
             "WRJ1": 10, "WRJ2": 5}

####################
# POSE DEFINITIONS #
####################

# starting position for the hand
start_pos = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0,
             "FFJ0": 0, "FFJ3": 0, "FFJ4": 0,
             "MFJ0": 0, "MFJ3": 0, "MFJ4": 0,
             "RFJ0": 0, "RFJ3": 0, "RFJ4": 0,
             "LFJ0": 0, "LFJ3": 0, "LFJ4": 0, "LFJ5": 0,
             "WRJ1": 0, "WRJ2": 0}
# Start position for the Hand
pregrasp_pos = {"THJ2": 12, "THJ3": 15, "THJ4": 69, "THJ5": -23,
                "FFJ0": 40, "FFJ3": 21, "FFJ4": -15,
                "MFJ0": 40, "MFJ3": 21, "MFJ4": 0,
                "RFJ0": 40, "RFJ3": 21, "RFJ4": -7,
                "LFJ0": 40, "LFJ3": 21, "LFJ4": -10, "LFJ5": 0,
                "WRJ1": 0, "WRJ2": 0}
# Close position for the Hand
grasp_pos = {"THJ2": 30, "THJ3": 15, "THJ4": 69, "THJ5": -3,
             "FFJ0": 77, "FFJ3": 67, "FFJ4": -19,
             "MFJ0": 82, "MFJ3": 62, "MFJ4": 0,
             "RFJ0": 89, "RFJ3": 64, "RFJ4": -18,
             "LFJ0": 97, "LFJ3": 64, "LFJ4": -19, "LFJ5": 0,
             "WRJ1": 0, "WRJ2": 0}
# Random position for the Hand (initialied at 0)
rand_pos = {"THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0,
            "FFJ0": 0, "FFJ3": 0, "FFJ4": 0,
            "MFJ0": 0, "MFJ3": 0, "MFJ4": 0,
            "RFJ0": 0, "RFJ3": 0, "RFJ4": 0,
            "LFJ0": 0, "LFJ3": 0, "LFJ4": 0, "LFJ5": 0,
            "WRJ1": 0, "WRJ2": 0}
# flex first finger
flex_ff = {"FFJ0": 180, "FFJ3": 90, "FFJ4": 0}
# extend first finger
ext_ff = {"FFJ0": 0, "FFJ3": 0, "FFJ4": 0}
# flex middle finger
flex_mf = {"MFJ0": 180, "MFJ3": 90, "MFJ4": 0}
# extend middle finger
ext_mf = {"MFJ0": 0, "MFJ3": 0, "MFJ4": 0}
# flex ring finger
flex_rf = {"RFJ0": 180, "RFJ3": 90, "RFJ4": 0}
# extend ring finger
ext_rf = {"RFJ0": 0, "RFJ3": 0, "RFJ4": 0}
# flex little finger
flex_lf = {"LFJ0": 180, "LFJ3": 90, "LFJ4": 0}
# extend middle finger
ext_lf = {"LFJ0": 0, "LFJ3": 0, "LFJ4": 0}
# flex thumb step 1
flex_th_1 = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 70, "THJ5": 0}
# flex thumb step 2
flex_th_2 = {"THJ1": 35, "THJ2": 40, "THJ3": 10, "THJ4": 70, "THJ5": 60}
# extend thumb step 1
ext_th_1 = {"THJ1": 10, "THJ2": 20, "THJ3": 5, "THJ4": 35, "THJ5": 25}
# extend thumb step 2
ext_th_2 = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0}
# zero thumb
zero_th = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0}
# Pre O.K. with first finger
pre_ff_ok = {"THJ4": 70}
# O.K. with first finger
ff_ok = {"THJ1": 15, "THJ2": 4, "THJ3": 0, "THJ4": 56, "THJ5": 20,
         "FFJ0": 85, "FFJ3": 45, "FFJ4": -0.2,
         "MFJ0": 42, "MFJ3": 33, "MFJ4": -3,
         "RFJ0": 50, "RFJ3": 18, "RFJ4": 0.5,
         "LFJ0": 30, "LFJ3": 0, "LFJ4": -6, "LFJ5": 7}
# O.K. transition from first finger to middle finger
ff2mf_ok = {"THJ1": 5, "THJ2": -5, "THJ3": 4, "THJ4": 60, "THJ5": 2,
            "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
            "MFJ0": 42, "MFJ3": 33, "MFJ4": -3,
            "RFJ0": 50, "RFJ3": 18, "RFJ4": 0.5,
            "LFJ0": 30, "LFJ3": 0, "LFJ4": -6, "LFJ5": 7}
# O.K. with middle finger
mf_ok = {"THJ1": 15, "THJ2": 0, "THJ3": 7, "THJ4": 66, "THJ5": 30,
         "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
         "MFJ0": 80, "MFJ3": 54, "MFJ4": 11,
         "RFJ0": 50, "RFJ3": 18, "RFJ4": -10,
         "LFJ0": 30, "LFJ3": 0, "LFJ4": -6, "LFJ5": 7}
# O.K. transition from middle finger to ring finger
mf2rf_ok = {"THJ1": 5, "THJ2": -5, "THJ3": 15, "THJ4": 70, "THJ5": 19,
            "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
            "MFJ0": 45, "MFJ3": 4, "MFJ4": -1,
            "RFJ0": 50, "RFJ3": 18, "RFJ4": -19,
            "LFJ0": 30, "LFJ3": 0, "LFJ4": -12, "LFJ5": 7}
# O.K. with ring finger
rf_ok = {"THJ1": 6, "THJ2": -6, "THJ3": 12, "THJ4": 70, "THJ5": 45,
         "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
         "MFJ0": 45, "MFJ3": 4, "MFJ4": -1,
         "RFJ0": 101, "RFJ3": 40, "RFJ4": -19,
         "LFJ0": 30, "LFJ3": 0, "LFJ4": -12, "LFJ5": 7}
# O.K. transition from ring finger to little finger
rf2lf_ok = {"THJ1": 5, "THJ2": 4.5, "THJ3": 8, "THJ4": 60, "THJ5": 21,
            "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
            "MFJ0": 45, "MFJ3": 4, "MFJ4": -1,
            "RFJ0": 30, "RFJ3": 6, "RFJ4": 0.5,
            "LFJ0": 30, "LFJ3": 0, "LFJ4": -10, "LFJ5": 7}
# O.K. with little finger
lf_ok = {"THJ1": 21, "THJ2": -10, "THJ3": 10, "THJ4": 69, "THJ5": 29,
         "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
         "MFJ0": 15, "MFJ3": 4, "MFJ4": -1,
         "RFJ0": 15, "RFJ3": 6, "RFJ4": 0.5,
         "LFJ0": 77, "LFJ3": 35, "LFJ4": 6, "LFJ5": 35}
# zero wrist
zero_wr = {"WRJ1": 0, "WRJ2": 0}
# north wrist
n_wr = {"WRJ1": 15, "WRJ2": 0}
# south wrist
s_wr = {"WRJ1": -20, "WRJ2": 0}
# east wrist
e_wr = {"WRJ1": 0, "WRJ2": 8}
# west wrist
w_wr = {"WRJ1": 0, "WRJ2": -14}
# northeast wrist
ne_wr = {"WRJ1": 15, "WRJ2": 8}
# northwest wrist
nw_wr = {"WRJ1": 15, "WRJ2": -14}
# southweast wrist
sw_wr = {"WRJ1": -20, "WRJ2": -14}
# southeast wrist
se_wr = {"WRJ1": -20, "WRJ2": 8}
# lateral lf ext side
l_ext_lf = {"LFJ4": -15}
# lateral rf ext side
l_ext_rf = {"RFJ4": -15}
# lateral mf ext side
l_ext_mf = {"MFJ4": 15}
# lateral ff ext side
l_ext_ff = {"FFJ4": 15}
# lateral all int side
l_int_all = {"FFJ4": -15, "MFJ4": -15, "RFJ4": 15, "LFJ4": 15}
# lateral all ext side
l_ext_all = {"FFJ4": 15, "MFJ4": 15, "RFJ4": -15, "LFJ4": -15}
# lateral ff int side
l_int_ff = {"FFJ4": -15}
# lateral mf int side
l_int_mf = {"MFJ4": -15}
# lateral rf int side
l_int_rf = {"RFJ4": 15}
# lateral lf int side
l_int_lf = {"LFJ4": 15}
# all zero
l_zero_all = {"FFJ4": 0, "MFJ4": 0, "RFJ4": 0, "LFJ4": 0}
# spock
l_spock = {"FFJ4": -20, "MFJ4": -20, "RFJ4": -20, "LFJ4": -20}
# grasp for shaking hands step 1
shake_grasp_1 = {"THJ1": 0, "THJ2": 6, "THJ3": 10, "THJ4": 37, "THJ5": 9,
                 "FFJ0": 21, "FFJ3": 26, "FFJ4": 0,
                 "MFJ0": 18, "MFJ3": 24, "MFJ4": 0,
                 "RFJ0": 30, "RFJ3": 16, "RFJ4": 0,
                 "LFJ0": 30, "LFJ3": 0, "LFJ4": -10, "LFJ5": 10}
# grasp for shaking hands step 2
shake_grasp_2 = {"THJ1": 21, "THJ2": 21, "THJ3": 10, "THJ4": 42, "THJ5": 21,
                 "FFJ0": 75, "FFJ3": 29, "FFJ4": 0,
                 "MFJ0": 75, "MFJ3": 41, "MFJ4": 0,
                 "RFJ0": 75, "RFJ3": 41, "RFJ4": 0,
                 "LFJ0": 100, "LFJ3": 41, "LFJ4": 0, "LFJ5": 0}
# store step 1 PST
store_1_PST = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 60, "THJ5": 0,
               "FFJ0": 180, "FFJ3": 90, "FFJ4": 0,
               "MFJ0": 180, "MFJ3": 90, "MFJ4": 0,
               "RFJ0": 180, "RFJ3": 90, "RFJ4": 0,
               "LFJ0": 180, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
               "WRJ1": 0, "WRJ2": 0}
# store step 2 PST
store_2_PST = {"THJ1": 50, "THJ2": 12, "THJ3": 0, "THJ4": 60, "THJ5": 27,
               "FFJ0": 180, "FFJ3": 90, "FFJ4": 0,
               "MFJ0": 180, "MFJ3": 90, "MFJ4": 0,
               "RFJ0": 180, "RFJ3": 90, "RFJ4": 0,
               "LFJ0": 180, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
               "WRJ1": 0, "WRJ2": 0}
# store step 1 Bio_Tac
store_1_BioTac = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 30, "THJ5": 0,
                  "FFJ0": 180, "FFJ3": 90, "FFJ4": 0,
                  "MFJ0": 180, "MFJ3": 90, "MFJ4": 0,
                  "RFJ0": 180, "RFJ3": 90, "RFJ4": 0,
                  "LFJ0": 180, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
                  "WRJ1": 0, "WRJ2": 0}
# store step 2 Bio_Tac
store_2_BioTac = {"THJ1": 20, "THJ2": 36, "THJ3": 0, "THJ4": 30, "THJ5": -3,
                  "FFJ0": 180, "FFJ3": 90, "FFJ4": 0,
                  "MFJ0": 180, "MFJ3": 90, "MFJ4": 0,
                  "RFJ0": 180, "RFJ3": 90, "RFJ4": 0,
                  "LFJ0": 180, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
                  "WRJ1": 0, "WRJ2": 0}
# store step 3
store_3 = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 65, "THJ5": 0}
# business card pre-zero position
bc_pre_zero = {"THJ1": 15, "THJ2": 7, "THJ3": -4, "THJ4": 50, "THJ5": -17,
               "FFJ0": 14, "FFJ3": 7, "FFJ4": -1,
               "MFJ0": 51, "MFJ3": 33, "MFJ4": 20,
               "RFJ0": 50, "RFJ3": 18, "RFJ4": -20,
               "LFJ0": 30, "LFJ3": 0, "LFJ4": -20, "LFJ5": 7}
# business card zero position
bc_zero = {"THJ1": 23, "THJ2": 6, "THJ3": -1, "THJ4": 43, "THJ5": -11,
           "MFJ0": 63, "MFJ3": 24, "MFJ4": 20}
# business card position 1
bc_1 = {"FFJ0": 137, "FFJ3": 7}
# business card position 2
bc_2 = {"FFJ0": 137, "FFJ3": 58}
# business card position 3
bc_3 = {"FFJ0": 72, "FFJ3": 62}
# business card position 4
bc_4 = {"FFJ0": 180, "FFJ3": 58}
# business card position 5
bc_5 = {"FFJ0": 180, "FFJ3": 0}
# business card position 6
bc_6 = {"FFJ0": 0, "FFJ3": 0}
# business card position 7
bc_7 = {"FFJ0": 137, "FFJ3": 15}
# business card position 8
bc_8 = {"FFJ0": 137, "FFJ3": 58}
# business card position 9
bc_9 = {"FFJ0": 80, "FFJ3": 58}
# business card position 10
bc_10 = {"MFJ3": 64, "FFJ4": 20}
# business card position 11
bc_11 = {"FFJ0": 81, "FFJ3": 50, "FFJ4": 20,
         "THJ4": 57, "THJ5": 25}
# business card position 12
bc_12 = {"MFJ0": 20, "MFJ3": 10, "MFJ4": 0}


########################
# FUNCTION DEFINITIONS #
########################

def sequence_ff():
    # Start secuence 1
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(store_3, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, wait=False, use_prefix=True)
    rospy.sleep(1.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(flex_th_1, wait=False, use_prefix=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(flex_th_2, wait=False, use_prefix=True)
#   tmp = flex_th_2.copy()
#   tmp.update({'interpolation_time': 2.0})
#   hand_commander.move_to_joint_value_target_unsafe(tmp)

#   while True:
#      # Check  the state of the tactile senors
#      read_tactile_values()
#      # Record current joint positions
#      hand_pos = c.get_hand_position()
#      # If the tacticle sensor is triggered stop movement
#      if ( tactile_values['TH'] > force_zero['TH'] ):
#         hand_commander.move_to_joint_value_target_unsafe(hand_pos)
#         print 'Thumb contact'
#         break

    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ext_th_2, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_lf, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_rf, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_mf, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_ff, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_all, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_all, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_ff, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_mf, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_rf, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_lf, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_zero_all, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_spock, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_zero_all, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(pre_ff_ok, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ff_ok, wait=False, use_prefix=True)
#   tmp = ff_ok.copy()
#   tmp.update({'interpolation_time': 2.0})
#   hand_commander.move_to_joint_value_target_unsafe(tmp)

#   while True:
#      # Check  the state of the tactile senors
#      read_tactile_values()
#      # Record current joint positions
#      hand_pos = c.get_hand_position()
#      # If the tacticle sensor is triggered stop movement
#      if ( tactile_values['TH'] > force_zero['TH'] or tactile_values['FF'] > force_zero['FF'] ):
#         hand_commander.move_to_joint_value_target_unsafe(hand_pos)
#         print 'First finger contact'
#         break

    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ff2mf_ok, wait=False, use_prefix=True)
    rospy.sleep(0.8)
    hand_commander.move_to_joint_value_target_unsafe(mf_ok, wait=False, use_prefix=True)
#   tmp = mf_ok.copy()
#   tmp.update({'interpolation_time': 2.0})
#   hand_commander.move_to_joint_value_target_unsafe(tmp)

#   while True:
#      # Check  the state of the tactile senors
#      read_tactile_values()
#      # Record current joint positions
#      hand_pos = c.get_hand_position()
#      # If the tacticle sensor is triggered stop movement
#      if ( tactile_values['TH'] > force_zero['TH'] or tactile_values['MF'] > force_zero['MF'] ):
#         hand_commander.move_to_joint_value_target_unsafe(hand_pos)
#         print 'Middle finger contact'
#         break

    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(mf2rf_ok, wait=False, use_prefix=True)
    rospy.sleep(0.8)
    hand_commander.move_to_joint_value_target_unsafe(rf_ok, wait=False, use_prefix=True)
#   tmp = rf_ok.copy()
#   tmp.update({'interpolation_time': 2.0})
#   hand_commander.move_to_joint_value_target_unsafe(tmp)

#   while True:
#      # Check  the state of the tactile senors
#      read_tactile_values()
#      # Record current joint positions
#      hand_pos = c.get_hand_position()
#      # If the tacticle sensor is triggered stop movement
#      if ( tactile_values['TH'] > force_zero['TH'] or tactile_values['RF'] > force_zero['RF'] ):
#         hand_commander.move_to_joint_value_target_unsafe(hand_pos)
#         print 'Ring finger contact'
#         break

    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(rf2lf_ok, wait=False, use_prefix=True)
    rospy.sleep(0.8)
    hand_commander.move_to_joint_value_target_unsafe(lf_ok, wait=False, use_prefix=True)
#   tmp = lf_ok.copy()
#   tmp.update({'interpolation_time': 2.0})
#   hand_commander.move_to_joint_value_target_unsafe(tmp)

#   while True:
#      # Check  the state of the tactile senors
#      read_tactile_values()
#      # Record current joint positions
#      hand_pos = c.get_hand_position()
#      # If the tacticle sensor is triggered stop movement
#      if ( tactile_values['TH'] > force_zero['TH'] or tactile_values['LF'] > force_zero['LF'] ):
#         hand_commander.move_to_joint_value_target_unsafe(hand_pos)
#         print 'Little finger contact'
#         break

    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, wait=False, use_prefix=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(pre_ff_ok, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ff_ok, wait=False, use_prefix=True)
#   tmp = ff_ok.copy()
#   tmp.update({'interpolation_time': 3.0})
#   hand_commander.move_to_joint_value_target_unsafe(tmp)

#   while True:
#      # Check  the state of the tactile senors
#      read_tactile_values()
#      # Record current joint positions
#      hand_pos = c.get_hand_position()
#      # If the tacticle sensor is triggered stop movement
#      if ( tactile_values['TH'] > force_zero['TH'] or tactile_values['FF'] > force_zero['FF'] ):
#         hand_commander.move_to_joint_value_target_unsafe(hand_pos)
#         print 'First finger contact'
#         break

    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ne_wr, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(nw_wr, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(sw_wr, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(se_wr, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ne_wr, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(nw_wr, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(sw_wr, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(se_wr, wait=False, use_prefix=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(zero_wr, wait=False, use_prefix=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, wait=False, use_prefix=True)
    rospy.sleep(1)
    return


def sequence_mf():
    # Start the secuence 2
    rospy.sleep(2.0)
    # Initialize wake time
    wake_time = time.time()

    while True:
        # Check if any of the tactile senors have been triggered
        # If so, send the Hand to its start position
        read_tactile_values()
        if (tactile_values['FF'] > force_zero['FF'] or
           tactile_values['MF'] > force_zero['MF'] or
           tactile_values['RF'] > force_zero['RF'] or
           tactile_values['LF'] > force_zero['LF'] or
           tactile_values['TH'] > force_zero['TH']):

            hand_commander.move_to_joint_value_target_unsafe(start_pos)
            print 'HAND TOUCHED!'
            rospy.sleep(2.0)

            if tactile_values['TH'] > force_zero['TH']:
                break

        # If the tactile sensors have not been triggered and the Hand
        # is not in the middle of a movement, generate a random position
        # and interpolation time
        else:
            if time.time() > wake_time:
                for i in rand_pos:
                    rand_pos[i] = random.randrange(min_range[i], max_range[i])

                rand_pos['FFJ4'] = random.randrange(min_range['FFJ4'], rand_pos['MFJ4'])
                rand_pos['LFJ4'] = random.randrange(min_range['LFJ4'], rand_pos['RFJ4'])
                time_to_position = max_random_interpolation_time * random.random()

                hand_commander.move_to_joint_value_target_unsafe(rand_pos, time=time_to_position, wait=False, use_prefix=True)
                wake_time = time.time() + time_to_position * 0.9
    return


def sequence_rf():
    # Start the secuence 3
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, wait=False, use_prefix=True)
    rospy.sleep(1.5)
    hand_commander.move_to_joint_value_target_unsafe(shake_grasp_1, wait=False, use_prefix=True)
    rospy.sleep(2.5)
    hand_commander.move_to_joint_value_target_unsafe(shake_grasp_2, wait=False, use_prefix=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(e_wr, wait=False, use_prefix=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(w_wr, wait=False, use_prefix=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(zero_wr, wait=False, use_prefix=True)
    rospy.sleep(0.8)
    hand_commander.move_to_joint_value_target_unsafe(shake_grasp_1, wait=False, use_prefix=True)
    rospy.sleep(1.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, wait=False, use_prefix=True)
    rospy.sleep(1.5)

#   rospy.sleep(0.5)
#   hand_commander.move_to_joint_value_target_unsafe(start_pos)
#   rospy.sleep(1)
#   hand_commander.move_to_joint_value_target_unsafe(bc_pre_zero)
#   rospy.sleep(2)
#   hand_commander.move_to_joint_value_target_unsafe(bc_zero)
#   rospy.sleep(4)
#   hand_commander.move_to_joint_value_target_unsafe(bc_1)
#   rospy.sleep(1)
#   hand_commander.move_to_joint_value_target_unsafe(bc_2)
#   rospy.sleep(1)
#   hand_commander.move_to_joint_value_target_unsafe(bc_3)
#   rospy.sleep(1)
#   hand_commander.move_to_joint_value_target_unsafe(bc_4)
#   rospy.sleep(1)
#   hand_commander.move_to_joint_value_target_unsafe(bc_5)
#   rospy.sleep(1)
#   hand_commander.move_to_joint_value_target_unsafe(bc_6)
#   rospy.sleep(1)
#   hand_commander.move_to_joint_value_target_unsafe(bc_7)
#   rospy.sleep(1)
#   hand_commander.move_to_joint_value_target_unsafe(bc_8)
#   rospy.sleep(1)
#   hand_commander.move_to_joint_value_target_unsafe(bc_9)
#   rospy.sleep(1)
#   hand_commander.move_to_joint_value_target_unsafe(bc_11)
#   rospy.sleep(1)
#   hand_commander.move_to_joint_value_target_unsafe(bc_12)
#   rospy.sleep(3)
#   hand_commander.move_to_joint_value_target_unsafe(start_pos)
#   rospy.sleep(1.5)

    return


def sequence_lf():
    # Start the sequence 4
    # Trigger flag array
    trigger = [0, 0, 0, 0, 0]

    # Move Hand to zero position
    hand_commander.move_to_joint_value_target_unsafe(start_pos, wait=False, use_prefix=True)
    rospy.sleep(2.0)

    # Move Hand to starting position
    hand_commander.move_to_joint_value_target_unsafe(pregrasp_pos, wait=False, use_prefix=True)
    rospy.sleep(2.0)

    # Move Hand to close position
    grasp_duration = 10.0
    hand_commander.move_to_joint_value_target_unsafe(grasp_pos, time=grasp_duration, wait=False, use_prefix=True)

    curr_tgt = dict(grasp_pos)
    offset1 = 3

    # Initialize end time
    target_time = rospy.Time.now() + rospy.Duration.from_sec(grasp_duration)
    end_time = time.time() + grasp_duration + 1

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()

        # Record current joint positions
        hand_pos = hand_commander.get_joints_position()

        # If any tacticle sensor has been triggered, send
        # the corresponding digit to its current position
        if tactile_values['FF'] > force_zero['FF'] and trigger[0] == 0:
            curr_tgt.update({jname: pos + offset1 for (jname, pos) in hand_pos.items() if 'FFJ0' in jname or 'FFJ3' in jname})
            hand_commander.move_to_joint_value_target_unsafe(curr_tgt, time=(target_time - rospy.Time.now()).to_sec(), wait=False)
            print 'First finger contact'
            trigger[0] = 1

        if tactile_values['MF'] > force_zero['MF'] and trigger[1] == 0:
            curr_tgt.update({jname: pos + offset1 for (jname, pos) in hand_pos.items() if 'MFJ0' in jname or 'MFJ3' in jname})
            hand_commander.move_to_joint_value_target_unsafe(curr_tgt, time=(target_time - rospy.Time.now()).to_sec(), wait=False)
            print 'Middle finger contact'
            trigger[1] = 1

        if tactile_values['RF'] > force_zero['RF'] and trigger[2] == 0:
            curr_tgt.update({jname: pos + offset1 for (jname, pos) in hand_pos.items() if 'RFJ0' in jname or 'RFJ3' in jname})
            hand_commander.move_to_joint_value_target_unsafe(curr_tgt, time=(target_time - rospy.Time.now()).to_sec(), wait=False)
            print 'Ring finger contact'
            trigger[2] = 1

        if tactile_values['LF'] > force_zero['LF'] and trigger[3] == 0:
            curr_tgt.update({jname: pos + offset1 for (jname, pos) in hand_pos.items() if 'LFJ0' in jname or 'LFJ3' in jname})
            hand_commander.move_to_joint_value_target_unsafe(curr_tgt, time=(target_time - rospy.Time.now()).to_sec(), wait=False)
            print 'Little finger contact'
            trigger[3] = 1

        if tactile_values['TH'] > force_zero['TH'] and trigger[4] == 0:
            curr_tgt.update({jname: pos + offset1 for (jname, pos) in hand_pos.items() if 'THJ2' in jname or 'THJ5' in jname})
            hand_commander.move_to_joint_value_target_unsafe(curr_tgt, time=(target_time - rospy.Time.now()).to_sec(), wait=False)
            print 'Thumb contact'
            trigger[4] = 1

        if (trigger[0] == 1 and
           trigger[1] == 1 and
           trigger[2] == 1 and
           trigger[3] == 1 and
           trigger[4] == 1):
            break

        if time.time() > end_time:
            break

    # Send all joints to current position to compensate
    # for minor offsets created in the previous loop
    hand_pos = hand_commander.get_joints_position()
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, wait=False, use_prefix=False)
    rospy.sleep(2.0)

    # Generate new values to squeeze object slightly
    offset2 = 3
    squeeze = hand_pos.copy()
    squeeze.update({jname: pos + offset2 for (jname, pos) in hand_pos.items()
                    if 'FFJ0' in jname or 'FFJ3' in jname or
                    'MFJ0' in jname or 'MFJ3' in jname or
                    'RFJ0' in jname or 'RFJ3' in jname or
                    'LFJ0' in jname or 'LFJ3' in jname or
                    'THJ2' in jname or 'THJ5' in jname})

    # Squeeze object gently
    hand_commander.move_to_joint_value_target_unsafe(squeeze, wait=False, use_prefix=False)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, wait=False, use_prefix=False)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(squeeze, wait=False, use_prefix=False)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, wait=False, use_prefix=False)
    rospy.sleep(2.0)
    hand_commander.move_to_joint_value_target_unsafe(pregrasp_pos, wait=False, use_prefix=True)
    rospy.sleep(2.0)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, wait=False, use_prefix=True)
    rospy.sleep(2.0)

    return


def sequence_th():
    # Start the sequence 5
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, wait=False, use_prefix=True)
    rospy.sleep(1.5)
    return


def zero_tactile_sensors():
    # Move Hand to zero position
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, wait=False, use_prefix=True)

    print '\n\nPLEASE ENSURE THAT THE TACTILE SENSORS ARE NOT PRESSED\n'
    # raw_input ('Press ENTER to continue')
    rospy.sleep(1.0)

    for x in xrange(1, 1000):
        # Read current state of tactile sensors to zero them
        read_tactile_values()

        if tactile_values['FF'] > force_zero['FF']:
            force_zero['FF'] = tactile_values['FF']
        if tactile_values['MF'] > force_zero['MF']:
            force_zero['MF'] = tactile_values['MF']
        if tactile_values['RF'] > force_zero['RF']:
            force_zero['RF'] = tactile_values['RF']
        if tactile_values['LF'] > force_zero['LF']:
            force_zero['LF'] = tactile_values['LF']
        if tactile_values['TH'] > force_zero['TH']:
            force_zero['TH'] = tactile_values['TH']

    force_zero['FF'] += 3
    force_zero['MF'] += 3
    force_zero['RF'] += 3
    force_zero['LF'] += 3
    force_zero['TH'] += 3

    print 'Force Zero', force_zero

    rospy.loginfo("\n\nOK, ready for the demo")

    print "\nPRESS ONE OF THE TACTILES TO START A DEMO"
    print "   FF: Standard Demo"
    print "   MF: Shy Hand Demo"
    print "   RF: Shake Hand Demo"
    print "   LF: Grasp Demo"
    print "   TH: Open Hand"

    return


def read_tactile_values():
    # Read tactile type
    tactile_type = hand_commander.get_tactile_type()
    # Read current state of tactile sensors
    tactile_state = hand_commander.get_tactile_state()

    if tactile_type == "biotac":
        tactile_values['FF'] = tactile_state.tactiles[0].pdc
        tactile_values['MF'] = tactile_state.tactiles[1].pdc
        tactile_values['RF'] = tactile_state.tactiles[2].pdc
        tactile_values['LF'] = tactile_state.tactiles[3].pdc
        tactile_values['TH'] = tactile_state.tactiles[4].pdc

    elif tactile_type == "PST":
        tactile_values['FF'] = tactile_state.pressure[0]
        tactile_values['MF'] = tactile_state.pressure[1]
        tactile_values['RF'] = tactile_state.pressure[2]
        tactile_values['LF'] = tactile_state.pressure[3]
        tactile_values['TH'] = tactile_state.pressure[4]

    elif tactile_type is None:
        print "You don't have tactile sensors. Talk to your Shadow representative to purchase some"

    return

########
# MAIN #
########

# Zero values in dictionary for tactile sensors (initialized at 0)
force_zero = {"FF": 0, "MF": 0, "RF": 0, "LF": 0, "TH": 0}
# Initialize values for current tactile values
tactile_values = {"FF": 0, "MF": 0, "RF": 0, "LF": 0, "TH": 0}
# Zero tactile sensors
zero_tactile_sensors()

while not rospy.is_shutdown():
    # Check the state of the tactile senors
    read_tactile_values()

    # If the tactile is touched, trigger the corresponding function
    if tactile_values['FF'] > force_zero['FF']:
        print 'First finger contact'
        sequence_ff()
        print 'FF demo completed'
        zero_tactile_sensors()
    if tactile_values['MF'] > force_zero['MF']:
        print 'Middle finger contact'
        sequence_mf()
        print 'MF demo completed'
        zero_tactile_sensors()
    if tactile_values['RF'] > force_zero['RF']:
        print 'Ring finger contact'
        sequence_rf()
        print 'RF demo completed'
        zero_tactile_sensors()
    if tactile_values['LF'] > force_zero['LF']:
        print 'Little finger contact'
        sequence_lf()
        print 'LF demo completed'
        zero_tactile_sensors()
    if tactile_values['TH'] > force_zero['TH']:
        print 'Thumb finger contact'
        sequence_th()
        print 'TH demo completed'
        zero_tactile_sensors()
