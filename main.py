#!/usr/bin/python
# -*- coding: utf8 -*-
'''
Created on 26. des. 2012

@author: Håvard
'''

from __future__ import print_function
import LinRS
import time


def test(port='COM11'):
    # configure the serial connection
    ser = LinRS.Line('COM11').connect()
    #connect to one drive, (serial connection, drive ID)
    drive1 = LinRS.Drive(ser, '01')

    print('Status telegram from drive: ' + drive1.get_status())

    print('RX = ' + drive1.move_home())
    time.sleep(0.5)


    print('RX = ' + drive1.move_to_pos(10))
    time.sleep(0.5)
    print('RX = ' + drive1.move_to_pos(0))


    ## Loop for random movement. Commented out by default.
    # import random
    # i=50
    # while i>0:
    #     i-=1
    #     drive1.move_to_pos(random.randint(0, 120))
    #     time.sleep(random.uniform(0.1,0.5))

    ser.close()


if __name__ == '__main__':
    test()
