#!/usr/bin/env python

from math import tan


class Detector():

    def __init__(self, angle):
        '''
        ' Field of View :
        '   radius = height * tangent( angle )
        '''
        self.a = angle

    
    '''
    ' tracker   : [ x, y, z ]
    ' target    : { id => [x, y, z] }
    '''
    def get_detections(self, tracker, targets, get_all=False, pr=False):
        z_rec = [True for i in range(len(targets))]
        if get_all:
            return (targets, z_rec)

        zeta = {}

        xc = tracker[0]
        yc = tracker[1]
        r2 = (tracker[2] * tan(self.a))**2
        
        for k, v in targets.items():
            xp = v[0]
            yp = v[1]

            d = (xp - xc)**2 + (yp - yc)**2
            if d < r2:
                if pr:
                    print("detection", d, r2)
                zeta[k] = v
            else:
                if pr:
                    print("no detection", d, r2)
                z_rec[k] = False
                zeta[k] = [0., 0., 0., 0.]

        return (zeta, z_rec)
