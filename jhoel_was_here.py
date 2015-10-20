#!/usr/bin/env python

import numpy as np
import cv2

import video

from math import atan2, degrees, pi, cos, sin, radians

import nxt.locator
from nxt.motor import *
from nxt.bluesock import *
import time


#USER INPUTS
NUMBER_PLAYERS = 3

#SYSTEM VARS
NUMBER_OBJECTS =  NUMBER_PLAYERS*2 + 2

kp = 0.7
kd = 0.7
MIN_FRZ = 55
MAX_SCL = 35
MAX_SCA = 35
PONDER_SCA = 1.3
PONDER_SCL = 1.3
MAX_WHEEL = 127
LIMITE_ARQUERO_DELANTERO = 180
RADIO_BALL = 40

def get_angle(p1, p2): 
    (dx, dy) = (p1[0]-p2[0], p2[1]-p1[1])
    angle = degrees(atan2(float(dy), float(dx))) % 360
    if angle < 0:
        angle += 180
    return angle

def midpoint(p1, p2):
    return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)

def set_position(p1):
    print p1[0]
    return np.array([p1[0]-7 , 480+p1[1]])

def set_position_jean(p1):
    return np.array([p1[0] , 480-p1[1]])

def draw_arrow(image, p, q, color, arrow_magnitude=1, thickness=3, line_type=8, shift=0):
    # draw arrow tail
    cv2.line(image, p, q, color, thickness, line_type, shift)
    # calc angle of the arrow 
    angle = np.arctan2(p[1]-q[1], p[0]-q[0])
    # starting point of first line of arrow head 
    p = (int(q[0] + arrow_magnitude * np.cos(angle + np.pi/4)),
    int(q[1] + arrow_magnitude * np.sin(angle + np.pi/4)))
    # draw first half of arrow head
    cv2.line(image, p, q, color, thickness, line_type, shift)
    # starting point of second line of arrow head 
    p = (int(q[0] + arrow_magnitude * np.cos(angle - np.pi/4)),
    int(q[1] + arrow_magnitude * np.sin(angle - np.pi/4)))
    # draw second half of arrow head
    cv2.line(image, p, q, color, thickness, line_type, shift)

def draw_arrow_angle(image, p, angle, distance, color, arrow_magnitude=1, thickness=3, line_type=8, shift=0):
    new_angle = angle - 45
    print "new_angle", new_angle
    print "angle", angle
    print "cos", cos( radians(angle)) * distance
    print "sin", sin(radians(angle)) * distance
    print "p", p
    q = (int(p[0] + cos(radians(new_angle))*distance), int(p[1] - sin(radians(new_angle))*distance))
    print "q", q

    # draw arrow tail
    cv2.line(image, p, q, color, thickness, line_type, shift)
    # calc angle of the arrow 
    angle = np.arctan2(p[1]-q[1], p[0]-q[0])
    # starting point of first line of arrow head 
    p = (int(q[0] + arrow_magnitude * np.cos(angle + np.pi/4)),
    int(q[1] + arrow_magnitude * np.sin(angle + np.pi/4)))
    # draw first half of arrow head
    cv2.line(image, p, q, color, thickness, line_type, shift)
    # starting point of second line of arrow head 
    p = (int(q[0] + arrow_magnitude * np.cos(angle - np.pi/4)),
    int(q[1] + arrow_magnitude * np.sin(angle - np.pi/4)))
    # draw second half of arrow head
    cv2.line(image, p, q, color, thickness, line_type, shift)

class App(object):
    def __init__(self, video_src):
        self.cam = video.create_capture(video_src)
        ret, self.frame = self.cam.read()
        cv2.namedWindow('camshift')
        cv2.setMouseCallback('camshift', self.onmouse)

        self.drag_start = None
        self.tracking_state = 0
        self.show_backproj = False

        self.objects = [None] * (NUMBER_OBJECTS + 1)
        self.objects[NUMBER_OBJECTS] = 1
        self.windows = []
        self.histograms = []
        self.actual_object = 0
        self.actual_window = 0
        self.actual_histogram = 0

        self.variables = [None] * (NUMBER_OBJECTS)
        self.last_el = self.last_ea = 0

        self.gogo = False
        self.reset = False


    def onmouse(self, event, x, y, flags, param):
        x, y = np.int16([x, y]) # BUG
        if (self.actual_object == NUMBER_OBJECTS):
            self.tracking_state = 1
        else:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drag_start = (x, y)
                self.tracking_state = 0
                return
            if self.drag_start:
                if flags & cv2.EVENT_FLAG_LBUTTON:
                    h, w = self.frame.shape[:2]
                    xo, yo = self.drag_start
                    x0, y0 = np.maximum(0, np.minimum([xo, yo], [x, y]))
                    x1, y1 = np.minimum([w, h], np.maximum([xo, yo], [x, y]))
                    self.objects[self.actual_object] = None
                    if x1-x0 > 0 and y1-y0 > 0:
                        self.objects[self.actual_object] = (x0, y0, x1, y1)
                else:
                    self.drag_start = None
                    if self.objects[self.actual_object] is not None:
                        self.actual_object = self.actual_object + 1
                        if (self.actual_object == NUMBER_OBJECTS):
                            self.tracking_state = 1

    def show_hist(self):
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('hist', img)

    def is_complete(self):
        is_it = True
        for each in self.objects:
                if not each: is_it = False
        return is_it

    def run(self, m_right , m_left):
        while True:
            ret, self.frame = self.cam.read()
            vis = self.frame.copy()
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array((0., 60., 32.)), np.array((180., 255., 255.)))


            if self.is_complete():
                for each in self.objects[:-1]:
                    if each:
                        x0, y0, x1, y1 = each
                        self.windows.append((x0, y0, x1-x0, y1-y0))
                        hsv_roi = hsv[y0:y1, x0:x1]
                        mask_roi = mask[y0:y1, x0:x1]
                        hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
                        cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX);
                        self.histograms.append(hist.reshape(-1))
                        # self.show_hist()
                        vis_roi = vis[y0:y1, x0:x1]
                        cv2.bitwise_not(vis_roi, vis_roi)
                        vis[mask == 0] = 0
                self.objects = [None] * NUMBER_OBJECTS

            if self.tracking_state == 1:
                for idx, each in enumerate(self.histograms[:-1]):
                    prob = cv2.calcBackProject([hsv], [0], each, [0, 180], 1)
                    prob &= mask
                    term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
                    track_box, self.windows[idx] = cv2.meanShift(prob, self.windows[idx], term_crit)

                    if self.show_backproj:
                        vis[:] = prob[...,np.newaxis]
                    try:
                        x,y,w,h = self.windows[idx]
                        x2,y2,w2,h2 = self.aux_box
                        pointa1 = (x,y+h)
                        pointa2 = (x+w,y)
                        pointb1 = (x2,y2+h2)
                        pointb2 = (x2+w2,y2)
                        cv2.rectangle(vis, (x,y), (x+w,y+h), 255,2)
                        cv2.rectangle(vis, (x2,y2), (x2+w2,y2+h2), 255,2)
                        # cv2.ellipse(vis, self.windows[idx], (0, 0, 255), 2)
                        # q = (int(track_box[0][0]) , int(track_box[0][1]))
                        # p = (int(self.aux_box[0][0]) , int(self.aux_box[0][1]))
                        p = midpoint( pointb1, pointb2 )
                        q = midpoint( pointa1, pointa2 )

                        angle = get_angle(q, p)
                        center = midpoint(q, p)
                        center_array = set_position_jean(center)
                        print p , q, center, angle
                        if idx != 0:
                            if idx < len(self.histograms)/2:
                                print "menor"
                                draw_arrow_angle(vis, center, angle, 30 , (0, 255, 0), 15)
                            else:
                                print "mayor"
                                draw_arrow_angle(vis, center, angle, 30 , (0, 0, 255), 15)

                    except Exception as e:
                        print e

                    if idx % 2 == 0:
                        self.aux_box = self.windows[idx]
                    else:
                        self.variables[(idx - 1)/2] = (angle-45, center_array)
                        self.aux_box = None
                    if idx == (NUMBER_OBJECTS-2):
                        self.variables[idx/2] = set_position_jean( (x + w/2, y + h/2))

            cv2.imshow('camshift', vis)

            # print self.variables


            for idx, variable in enumerate(self.variables):
            	print idx, ".-", variable

            if self.variables[0] != None:
                self.variables[0][1][1] = self.variables[0][1][1]
                print "car_position" , self.variables[0][1]

                self.definir_estrategia()

                RI,RD = (0,0)

                print 'estrategia: ' , self.estrategia

                if self.estrategia == 0: #delantero
                    print 'ES DELANTERO'
                    # error = np.linalg.norm(self.variables[0][1] - ball_position[1])

                    
                    car_position = np.array(self.variables[0][1])
                    obj_position = np.array(self.variables[NUMBER_PLAYERS])

                    error = np.linalg.norm(car_position - obj_position)
                    print 'pre error:', error

                    if(error < RADIO_BALL): # lo toca
                        print 'go to the goal'
                        goal_pos = np.array((600,300))
                        angle = 0
                        RI,RD = self.move_pdi_to(goal_pos , angle) #va al arco
                    else: # si no
                        print 'search the ball'

                        # if car_position[0] > objective[0]: objective_angle = 180
                        # else: objective_angle = 0

                        angle = 0

                        RI,RD = self.move_pdi_to(np.array(self.variables[NUMBER_PLAYERS]) , angle) #sigue la pelota

                else: #arquero
                    print 'ES ARQUERO'

                    print self.variables[0][1]

                    if self.variables[0][1][1] < 300:
                        angle = 90
                    else:
                        angle = 270

                    print angle

                    own = np.array((40,250))
                    RI , RD = self.move_pdi_to(own , angle) #va a su arco

                print self.gogo
                if self.gogo:
                    m_right.run(RD)
                    m_left.run(RI)

                if not self.gogo:
                    m_left.brake()
                    m_right.brake()

                if self.reset:
                    RI , RD = self.move_pdi_to(np.array((320,240)) , 180) #va a su arco
                    m_right.run(RD)
                    m_left.run(RI)

                time.sleep(0.02)

                # m_left.brake()
                # m_right.brake()


    
            ch = 0xFF & cv2.waitKey(5)
            if ch == 27:
                m_left.brake()
                m_right.brake()

                break
                
            if ch == ord('b'):
                self.show_backproj = not self.show_backproj

            if ch == ord('g'):
                self.gogo = not self.gogo

            if ch == ord('r'):
                self.reset = not self.reset


        cv2.destroyAllWindows()

    def go_center(self):
        self.move_pdi_to(320,240)

    def definir_estrategia(self):

        if self.variables[0][1][0] < LIMITE_ARQUERO_DELANTERO :
            self.estrategia = 1 # arequero
        else:
            self.estrategia = 0 # delantero

    
    def move_pdi_to(self , objective , angle):

        car_angle = self.variables[0][0]

        # ball_position = self.variables[1]
        car_position = self.variables[0][1]

        print 'car_position', car_position
        print 'car angle: ', car_angle
        print 'objective angle:', angle

        error = np.linalg.norm(car_position - objective)

        Pl = kp*error

        print 'objective ' , objective
        print  'error:', error

        Dl = kd*(error - self.last_el)

        scl = Pl + Dl
        scl = scl/PONDER_SCL

        if(scl > MAX_SCL): scl = MAX_SCL

        ## 
        sub_error = angle - car_angle
        if sub_error < -180:
            sub_error = 360 + sub_error

        sub_error2 = (atan2(objective[1] - car_position[1], objective[0]-car_position[0]))/pi*180

        if sub_error2 < 90: sub_error2 = 180 + sub_error2
        if sub_error2 > 90: sub_error2 = sub_error2 - 180

        error_a = (sub_error + sub_error2)
        Pa = kp*error_a
        Da = kd*(error_a - self.last_ea)
        sca = Pa + Da

        sca = sca/PONDER_SCA

        if abs(sca) > MAX_SCA: 
            sca = sca/abs(sca)*MAX_SCA
            self.last_el = error
            self.last_ea = error_a

        RI = MIN_FRZ + scl - sca
        RD = MIN_FRZ + scl + sca
        print 'RI , RD: ', RI, RD

        
        # return 0,0
        return RI , RD   



if __name__ == '__main__':
    import sys
    try:
        video_src = sys.argv[1]
    except:
        video_src = 0
    print __doc__

    m_right = m_left = 1

    rokusho = BlueSock('00:16:53:1A:EA:61').connect()

    m_right = Motor(rokusho, PORT_B)
    m_left  = Motor(rokusho, PORT_C)


    App(video_src).run(m_right , m_left)

