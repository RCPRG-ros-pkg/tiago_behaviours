#!/usr/bin/env python

import PyKDL

def transformPoint(pt, tf):
    fr = PyKDL.Frame( PyKDL.Rotation.RotZ(tf[1]), PyKDL.Vector(tf[0][0], tf[0][1], 0) )
    tf_pt = fr * PyKDL.Vector(pt[0], pt[1], 0)
    return (tf_pt.x(), tf_pt.y())

def transformVector(vec, tf):
    rotM = PyKDL.Rotation.RotZ(tf[1])
    tf_n = rotM * PyKDL.Vector(vec[0], vec[1], 0)
    return (tf_n.x(), tf_n.y())
