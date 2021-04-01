
import PyKinectV2
from PyKinectV2 import *
import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys

import csv
import time
import datetime
import os

import pandas as pd

import logging
import threading
import time

import zmq
import json

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread


debug_no_csv = False # debug if testrun should not produce csv files


# colors for drawing different bodies 
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]

class Recorder(object):
    """
    This class can be used to record skeleton points and positioned points-of-interest in csv-files.

    After creating the recorder, start by calling run.
    """
    def __init__(self):
        """
        Create the Recorder and lists for collecting data
        """
        pygame.init()

        # manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect for Windows v2 Body Game")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data 
        self._bodies = None

        self.kin_counter = 0 # count through the samples
        self.hand_samples = [] # all samples of hand position of kinect
        self.full_samples = [] # all samples of all joints of kinect
        self.events = [] # all events from user input (mouse and key button down and up, mouse movement, window switching, window resizing)
        self.events_keys = [] # all events with keyboard (only button down), with unicode (key itself, 1-9, a-z, usw)

        # list of one single activity from key start to key end
        # is filled while recording activity from key_start to key_end
        # and after key_end directly saved to csv with activity_letter and timestamp in name
        self.activity = [] # list of samples

        self.finger_points = [] # 3d points and color points of fingertips with timestamp (for positioning of POI)

        # info: columns in self.positions_curr = ['key', 'point_x', 'point_y', 'pos_x', 'pos_y', 'pos_z', 'counter', 'timestamp']
        self.positions_curr = [] # latest positions of POI, if "tire 2" is reset, old position will be overwritten
        
        self.positions_all = [] # all positions of all POI through time, no position overwritten when reset
        self.tires_curr = [] # like positions_curr but only for keys 1-2 = 2 tires
        self.tires_all = [] # like positions_all but only for keys 1-2 = 2 tires
        self.fields_curr = [] # like tires_curr but only for keys 3-5 = 3 tires
        self.fields_all = [] # like positions_all but only for keys 3-5 = 3 tires

        self.distances = [] # distanz between wrist and POIs
        self.closest = [] # list of bodies and POI they each are closest to - will not be needed/saved

    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # at least one is good 
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        try:
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except: # need to catch it due to possible invalid positions (with inf)
            pass
        
    def draw_body(self, joints, jointPoints, color):
        bool_everything = True # True to draw whole skeleton, False to only draw hands

        # Torso
        if bool_everything:
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);
        
    
        # Right Arm 
        if bool_everything:   
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight); 
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight);

        # Left Arm
        if bool_everything:
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);

        # Right Leg
        if bool_everything: 
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight);

        # Left Leg
        if bool_everything: 
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft);
            self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft);

    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                # Event Types:
                # 768 = KEYDOWN - Taste drücken
                # 769 = KEYUP - Taste loslassen

                # save event into CSV with timestamp
                events_row = (event.type, self.kin_counter, int(time.time()*1000))
                self.events.append(events_row)
                
                if event.type == 768: # key log - button DOWN
                    # get last position of fingertips
                    last_position = self.finger_points[-1:][0]
                    # pos_row
                    pos_row = (event.unicode, last_position[0], last_position[1], last_position[2], last_position[3], last_position[4], last_position[5], last_position[6])
                    
                    # save keys to csv
                    keys_row = (event.type, event.unicode, event.scancode, self.kin_counter, int(time.time()*1000))
                    self.events_keys.append(keys_row)

                    if event.unicode == '1' or event.unicode == '2': # tires
                        # vorhandenes Tuple in tires_curr überschreiben
                        self.tires_curr = [i for i in self.tires_curr if i[0] != event.unicode] # delete current tire with event.unicode
                        self.tires_curr.append(pos_row) # set it with new values

                        self.tires_all.append(pos_row) # save tire position to list of all tires
                        
                        # vorhandenes Tuple in positions_curr überschreiben
                        self.positions_curr = [i for i in self.positions_curr if i[0] != event.unicode]
                        self.positions_curr.append(pos_row) 
                        print('set tire ', event.unicode)
                        self.positions_all.append(pos_row)
                    
                    elif event.unicode == '3' or event.unicode == '4': # fields
                        
                        #statt allen tires: fields_curr, in denen nur no-tires drinnen sind? --> darum dann Rechtecke zeichnen?
                        self.fields_curr = [i for i in self.fields_curr if i[0] != event.unicode] # delete current position with event.unicode
                        self.fields_curr.append(pos_row) # set it with new values
                        self.fields_all.append(pos_row) # save tire position to list of all tires

                        # vorhandenes Tuple in positions_curr überschreiben
                        self.positions_curr = [i for i in self.positions_curr if i[0] != event.unicode]
                        self.positions_curr.append(pos_row) 
                        print('set field ', event.unicode)
                        self.positions_all.append(pos_row)

                    else:
                        print('activity start of ', event.unicode)
                        # ab hier samples in self.activity speichern und wenn Ende die Liste als csv speichern und leeren (für nächste Aktivität)
                        # während normaler Aufnahme wird self.activity immer befüllt - weil hier drin landet man nur bei Tastendruck, nicht pro Sample
                        # hier wird bei passendem Tastendruck entweder List speichern oder leeren
                        
                if event.type == 769: # key log - button UP
                    # save keys to csv
                    keys_down_row = (event.type, "none", event.scancode, self.kin_counter, int(time.time()*1000))
                    self.events_keys.append(keys_down_row)
                    print('activity end')
                    
            # --- filling out back buffer surface with frame's data 
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # --- getting skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None: 
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked: 
                        continue 
                    
                    joints = body.joints 
                    # convert joint coordinates to color space 
                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i]) #draw body
                    self.drawOver() # draw own stuff

                    
            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            pygame.display.update()

            # --- update the screen with what was drawn
            pygame.display.flip()

            self.processHandPos() # hand processing

            # --- Limit to 60 frames per second
            self._clock.tick(60)

        # Close Kinect sensor, close the window and quit.
        self._kinect.close()

        if not debug_no_csv:
            print('saving into csv')
            self.saveIntoCSV()

        pygame.quit()

    ######################################################################
    #                               DRAW OVER
    ######################################################################
    def drawOver(self):
        """
        draws over points-of-interest = tires and fields
        draws tires as circles, fields as rectangles
        """
        offset = 25 # = radius for circles

        # drawing circles for tires
        for tire in self.tires_curr:
            tire_point = (int(tire[1]), int(tire[2]))
            pygame.draw.circle(self._frame_surface, 2, tire_point, offset, 8)
        
        # drawing rectangles for fields
        for field in self.fields_curr:
            field_point = (int(field[1]), int(field[2]))

            # calculate points around rectangle
            a = (field[1] - offset, field[2] + offset)
            b = (field[1] + offset, field[2] + offset)
            c = (field[1] + offset, field[2] - offset)
            d = (field[1] - offset, field[2] - offset)
            rect_pnt_list = []
            rect_pnt_list = [a,b,c,d]
            pygame.draw.polygon(self._frame_surface, 2, rect_pnt_list, 8)

    ######################################################################
    #                    PROCESS HAND POSITION
    ######################################################################
    def processHandPos(self):
        """
        takes all joints and joints of right hand and collects them, 
        and calls method to calculate and save extra data from distance between wrist and POIs
        """
        if self._bodies is not None:
            for i in range(0, self._kinect.max_body_count):
                body = self._bodies.bodies[i]
                if not body.is_tracked: continue 
                joints = body.joints
                # convert joint coordinates to color space
                joint_points = self._kinect.body_joints_to_color_space(joints)
                
                # full body
                joint_type = ['SpineBase','SpineMid','Neck','Head', 
                              'ShoulderLeft','ElbowLeft','WristLeft','HandLeft', 
                              'ShoulderRight','ElbowRight','WristRight','HandRight', 
                              'HipLeft','KneeLeft','AnkleLeft','FootLeft', 
                              'HipRight','KneeRight','AnkleRight','FootRight', 
                              'SpineShoulder','HandTipLeft','ThumbLeft','HandTipRight','ThumbRight','Count'] #names of those joints
                    
                for x in range(0,25): # joint-numbers that are interesting for us
                    pos = joints[x].Position
                    typ = joint_type[x]
                    
                    if x == 10: # = if joint == WristRight
                        csv_row = (pos.x, pos.y, pos.z, typ, self.kin_counter, int(time.time()*1000))
                        self.hand_samples.append(csv_row)
                
                    # add sample to log file
                    csv_row = (pos.x, pos.y, pos.z, typ, self.kin_counter, int(time.time()*1000))
                    self.full_samples.append(csv_row)

                fingers = PyKinectV2.JointType_HandTipRight
                csv_row = (joint_points[fingers].x, joint_points[fingers].y, joints[fingers].Position.x, joints[fingers].Position.y, joints[fingers].Position.z, self.kin_counter, int(time.time()*1000))
                self.finger_points.append(csv_row)
                
                self.kin_counter += 1 # increment counter for samples

                # calculate distance between right wrist and POIs, and save into self.distances
                self.calc_distances(i, joints)

    def calc_distances(self, body, joints):
        """
        calculates and saves the distance between wrist joint and the collected POIs
        :param body: index of current body - not used here, because only one current
        :param joints: all joints from collected body
        """
        wrist = PyKinectV2.JointType_WristRight
        # print('wrist xyz = ', joints[wrist].Position.x, joints[wrist].Position.y, joints[wrist].Position.z)
        
        diff = []
        row_with_all = ()

        for position in self.positions_curr: # iterate over all POIs
            # info: columns in self.positions_curr = ['key', 'point_x', 'point_y', 'pos_x', 'pos_y', 'pos_z', 'counter', 'timestamp']

            # calculate distances in x,y,z
            diff_x = (joints[wrist].Position.x - position[3]) ** 2
            diff_y = (joints[wrist].Position.y - position[4]) ** 2
            diff_z = (joints[wrist].Position.z - position[5]) ** 2
            
            diff_nmb = diff_x + diff_y + diff_z
            diff_row = (position[0], diff_nmb)

            # add here into df with diffs in columns
            row_with_all = row_with_all + (position[0], diff_nmb, int(time.time()*1000))
            
            diff.append(diff_row)

        self.distances.append(row_with_all) # append distances


    ######################################################################
    #               SAVE INTO CSV
    ######################################################################
    def saveIntoCSV(self):
        """
        save all collected lists into individual csv-files, each with specific column headers
        custom_dir includes timestamp to track samples
        """
        hand_dataFrame = pd.DataFrame(self.hand_samples, columns = ['pos_x', 'pos_y', 'pos_z', 'typ', 'counter', 'timestamp'])
        with open("%s/kin-sample-hand.csv" % custom_dir, "w") as fh_hand:
            hand_dataFrame.to_csv(fh_hand)

        full_dataFrame = pd.DataFrame(self.full_samples, columns = ['pos_x', 'pos_y', 'pos_z', 'typ', 'counter', 'timestamp'])
        with open("%s/kin-sample-full.csv" % custom_dir, "w") as fh_full:
            full_dataFrame.to_csv(fh_full)

        events_dataFrame = pd.DataFrame(self.events, columns = ['event_type', 'counter', 'timestamp'])
        with open("%s/kin-sample-events.csv" % custom_dir, "w") as fh_events:
            events_dataFrame.to_csv(fh_events)

        events_keys_dataFrame = pd.DataFrame(self.events_keys, columns = ['event_type', 'unicode', 'scancode', 'counter', 'timestamp'])
        with open("%s/kin-sample-events-keys.csv" % custom_dir, "w") as fh_events_keys:
            events_keys_dataFrame.to_csv(fh_events_keys)

        hand_poi_dataFrame = pd.DataFrame(self.finger_points, columns = ['point_x', 'point_y', 'pos_x', 'pos_y', 'pos_z', 'counter', 'timestamp'])
        with open("%s/kin-sample-hand-points.csv" % custom_dir, "w") as fh_hand_poi:
            hand_poi_dataFrame.to_csv(fh_hand_poi)

        pos_curr_dataFrame = pd.DataFrame(self.positions_curr, columns = ['key', 'point_x', 'point_y', 'pos_x', 'pos_y', 'pos_z', 'counter', 'timestamp'])
        with open("%s/kin-sample-positions-current.csv" % custom_dir, "w") as fh_pos_curr:
            pos_curr_dataFrame.to_csv(fh_pos_curr)

        pos_all_dataFrame = pd.DataFrame(self.positions_all, columns = ['key', 'point_x', 'point_y', 'pos_x', 'pos_y', 'pos_z', 'counter', 'timestamp'])
        with open("%s/kin-sample-positions_all.csv" % custom_dir, "w") as fh_pos_all:
            pos_all_dataFrame.to_csv(fh_pos_all)

        tires_curr_dataFrame = pd.DataFrame(self.tires_curr, columns = ['key', 'point_x', 'point_y', 'pos_x', 'pos_y', 'pos_z', 'counter', 'timestamp'])
        with open("%s/kin-sample-tires-current.csv" % custom_dir, "w") as fh_tires_curr:
            tires_curr_dataFrame.to_csv(fh_tires_curr)

        tires_all_dataFrame = pd.DataFrame(self.tires_all, columns = ['key', 'point_x', 'point_y', 'pos_x', 'pos_y', 'pos_z', 'counter', 'timestamp'])
        with open("%s/kin-sample-tires_all.csv" % custom_dir, "w") as fh_tires_all:
            tires_all_dataFrame.to_csv(fh_tires_all)

        fields_curr_dataFrame = pd.DataFrame(self.fields_curr, columns = ['key', 'point_x', 'point_y', 'pos_x', 'pos_y', 'pos_z', 'counter', 'timestamp'])
        with open("%s/kin-sample-fields-current.csv" % custom_dir, "w") as fh_fields_curr:
            fields_curr_dataFrame.to_csv(fh_fields_curr)

        fields_all_dataFrame = pd.DataFrame(self.fields_all, columns = ['key', 'point_x', 'point_y', 'pos_x', 'pos_y', 'pos_z', 'counter', 'timestamp'])
        with open("%s/kin-sample-fields_all.csv" % custom_dir, "w") as fh_fields_all:
            fields_all_dataFrame.to_csv(fh_fields_all)

        distances_all_dataFrame = pd.DataFrame(self.distances, columns = ['tire1', 'distance1', 'timestamp1', 'tire2', 'distance2', 'timestamp2', 'field1', 'distance_field1', 'timestamp_field1', 'field2', 'distance_field2', 'timestamp_field2'])#, 'field3', 'distance_field3', 'timestamp_field3', 'field4', 'distance_field4', 'timestamp_field4', 'field5', 'distance_field5', 'timestamp_field5'])#, '5', '6', '7', '8'])
        with open("%s/kin-sample-distances_all.csv" % custom_dir, "w") as fh_distances_all:
            distances_all_dataFrame.to_csv(fh_distances_all)
          
        #closest_dataFrame = pd.DataFrame(self.closest, columns = ['key', 'point_x', 'point_y', 'pos_x', 'pos_y', 'pos_z', 'counter', 'timestamp'])
        #with open("%s/kin-sample-closest.csv" % custom_dir, "w") as fh_closest:
        #    closest_dataFrame.to_csv(fh_closest)

        # make csv-file with targets
        self.targets = []
        # targets = labels for motions that subject would do inbetween keydown and keyup, preset here to document order of motions for later analysis
        csv_row = ('050', '020', '110', '121', '010', '030', '020', '130', '111', '010', '040', '020', '210', '220', '230', '240', '310', '320', '330', '340', '410')
        self.targets.append(csv_row)

        targets_dataFrame = pd.DataFrame(self.targets, columns = ['01', '02', '03', '04', '05', '06', '07', '08', '09', '10', '11', '12', '13', '14', '15', '16', '17', '18', '19', '20', '21'])
        with open("%s/targets.csv" % custom_dir, "w") as fh_targets:
            targets_dataFrame.to_csv(fh_targets)


        ## open session_list_csv and add custom dir
        #with open("session_names.csv", "a") as fh_session_name:
        #    fh_session_name.write(custom_dir)

        # make csv-file with samples
        with open("recordings/session_names.csv", "a") as fh_samples:
            csv_row = ',' + dir_name
            fh_samples.write(csv_row)

        # make csv-file with samples
        with open("recordings/notes.csv", "a", newline='') as fh_notes:
            csv_row = '\n' + dir_name + ': '
            fh_notes.write(csv_row)
            
# -----------------------------------------------------------------------------------------------------------------------------------------------------------

__main__ = "Kinect v2 Recorder"

if not debug_no_csv: # for saving into csv
    dir_name = "sample-{}".format(datetime.datetime.now().strftime("%Y%m%d-%H%M%S")) # set name for directory
    custom_dir = "recordings/" + dir_name # set path for directory
    os.makedirs(custom_dir) # create directory

    plot_dir = (custom_dir + "/plots")      
    os.makedirs(plot_dir) # create directory

game = Recorder()
game.run()

