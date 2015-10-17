#!/usr/bin/env python

import numpy as np
import cv2

import video

from math import atan2, degrees  , pi

NUMBER_OBJECTS = 3


kp = 0.3
kd = 0.3
MIN_FRZ = 50
MAX_SCL = 35
MAX_SCA = 35
PONDER_SCA = 3
PONDER_SCL = 2

import nxt.locator
from nxt.motor import *
from nxt.bluesock import *
import time



def control_remoto(m_right, m_left):

    while(1):
        f = input("plz")
        if f == 1:
            print 'up'
            m_left.run(80)
            m_right.run(80)
        elif f == 2:
            print 'down'
            m_left.run(-80)
            m_right.run(-80)
        elif f == 4:
            print 'left'
            m_left.run(-90)
            m_right.run(90)
        elif f == 5:
            print 'right'
            m_left.run(90)
            m_right.run(-90)
        elif f == 8:
            print 'stop'
            m_left.brake()
            m_right.brake()

        else:
            continue


if __name__ == '__main__':
    mac_rokusho = '00:16:53:1A:EA:61'

    rokusho = BlueSock(mac_rokusho).connect()

    m_right = Motor(rokusho, PORT_B)
    m_left  = Motor(rokusho, PORT_C)

    control_remoto(m_right , m_left)
