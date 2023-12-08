#!/usr/bin/env python
# license removed for brevity
import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from sti_msgs.msg import PathInfo, PointRequestMove, ListPointRequestMove, LineRequestMove

from math import sin, cos, asin, tan, atan, degrees, radians, sqrt, fabs, acos, atan2
from math import pi as PI

def calAngleThreePoint(x1, y1, x2, y2, x3, y3):
    dx1 = x1 - x2
    dy1 = y1 - y2
    dx2 = x3 - x2
    dy2 = y3 - y2
    c_goc = (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-12)
    goc = acos(c_goc)
    
    return goc

def fnCalcDistPoints(x1, x2, y1, y2):
    return sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

def funcalptduongthang(X_s, Y_s, X_f, Y_f):
    _a = Y_s - Y_f
    _b = X_f - X_s
    _c = -X_s*_a -Y_s*_b
    return _a, _b, _c

def ptduongthangvuonggoc(a, b, _x, _y):
    return a, b, (-1)*a*_x - b*_y

def funcalPointTT(a_qd, b_qd, c_qd, _d, _x, _y, _x_s, _y_s, _dis):
    X_c = Y_c =  X_c1 = Y_c1 = X_c2 = Y_c2 = 0.0
    if b_qd == 0.0:
        X_c1 = X_c2 = -c_qd/a_qd
        Y_c1 = -sqrt(_d*_d - (X_c1 - _x)*(X_c1 - _x)) + _y
        Y_c2 = sqrt(_d*_d - (X_c2 - _x)*(X_c2 - _x)) + _y
        
    else:
        la = (1.0 + (a_qd/b_qd)*(a_qd/b_qd))
        lb = -2.0*(_x - (a_qd/b_qd)*((c_qd/b_qd) + _y))
        lc = _x*_x + ((c_qd/b_qd) + _y)*((c_qd/b_qd) + _y) - _d*_d
        denlta = lb*lb - 4.0*la*lc
        # print(la,lb,lc,denlta)

        X_c1 = (-lb + sqrt(denlta))/(2.0*la)
        X_c2 = (-lb - sqrt(denlta))/(2.0*la)

        Y_c1 = (-c_qd - a_qd*X_c1)/b_qd
        Y_c2 = (-c_qd - a_qd*X_c2)/b_qd
        
    disC1 = fnCalcDistPoints(_x_s, X_c1, _y_s, Y_c1)
    # disC2 = self.fnCalcDistPoints(_x_s, X_c2, _y_s, Y_c2)
    
    if disC1 < _dis:
        X_c = X_c1
        Y_c = Y_c1
        
    else:
        X_c = X_c2
        Y_c = Y_c2
    
    return X_c, Y_c

def funcalCircle(x1, y1, x2, y2, x3, y3):
        X_c = 0
        Y_c = 0
        R = 1.
        # tim pt duong thang 1:
        a1, b1, c1 = funcalptduongthang(x1, y1, x2, y2)
        a2, b2, c2 = funcalptduongthang(x2, y2, x3, y3)
        
        # tim ban kinh duong tron:
        _theta = calAngleThreePoint(x1, y1, x2, y2, x3, y3)
        d1 = fnCalcDistPoints(x1, x2, y1, y2)
        d2 = fnCalcDistPoints(x2, x3, y2, y3)
        dmin = min(d1, d2)
        rospy.logwarn("d1 = %s |d2 = %s |dmin = %s", d1, d2 , dmin)
        
        r = (dmin/2)*tan(_theta/2)
        # R = constrain(r, MinRadiusCircle, MaxRadiusCircle)
        
        # tim 2 diem tiep tuyen:
        _d = R/tan(_theta/2)
        Xc1, Yc1 = funcalPointTT(a1, b1, c1, _d, x2, y2, x1, y1, d1)
        Xc2, Yc2 = funcalPointTT(a2, b2, c2, _d, x2, y2, x3, y3, d2)
        
        # tim tam duong tron:
        avg1, bvg1, cvg1 = ptduongthangvuonggoc(b1, (-1)*a1, Xc1, Yc1)
        avg2, bvg2, cvg2 = ptduongthangvuonggoc(b2, (-1)*a2, Xc2, Yc2)
        
        if bvg1 == 0.0:
            X_c = -cvg1/avg1
            Y_c = (-cvg2 - avg2*X_c)/bvg2
            
        elif bvg2 == 0.0:
            X_c = -cvg2/avg2
            Y_c = (-cvg1 - avg1*X_c)/bvg1
            
        else:
            X_c = ((cvg1/bvg1)-(cvg2/bvg2))/((avg2/bvg2)-(avg1/bvg1))
            Y_c = (-cvg1 - avg1*X_c)/bvg1
        
        return X_c, Y_c, R, Xc1, Yc1, Xc2, Yc2, _d

def talker():
    pub = rospy.Publisher('/move_request', LineRequestMove, queue_size=10)
    rospy.init_node('pub_requestMove', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # print(funcalCircle(7.57,5.778,7.57,-2.201,9.989,-2.201))

    msg = LineRequestMove()
    msg.enable = 1
    msg.target_x = 0.667
    msg.target_y = -1.496
    msg.target_z = 3.12

    # ------------------------------------------------------------------------------
    # path 1
    # path1 = PathInfo()
    # path1.pathID = 1
    # path1.typePath = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -0.878
    # path1.pointOne.position.y = -7.681
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -3.413
    # path1.pointSecond.position.y = -7.681
    # path1.velocity = 0.3
    # msg.pathInfo.append(path1)


    # path 1
    # path1 = PathInfo()
    # path1.pathID = 1
    # path1.typePath = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -0.085
    # path1.pointOne.position.y = -10.740
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -0.085
    # path1.pointSecond.position.y = -3.576
    # path1.velocity = 0.3
    # msg.pathInfo.append(path1)

    # path 2 Benze 
    # path2 = PathInfo()
    # path2.pathID = 2
    # path2.typePath = 3
    # path2.pointOne = Pose()
    # path2.pointOne.position.x = -3.413
    # path2.pointOne.position.y = -7.681
    # path2.pointSecond = Pose()
    # path2.pointSecond.position.x = -4.413
    # path2.pointSecond.position.y = -8.681
    # path2.velocity = 0.2
    # path2.numberPts = 150
    # path2.pointMid = Pose()
    # path2.pointMid.position.x = -4.413
    # path2.pointMid.position.y = -7.681
    # msg.pathInfo.append(path2)

    # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 3
    # path3.typePath = 3
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = -4.413
    # path3.pointOne.position.y = -8.681
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = -3.413
    # path3.pointSecond.position.y = -10.073
    # path3.velocity = 0.2
    # path3.numberPts = 150
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = -4.413
    # path3.pointMid.position.y = -10.073
    # msg.pathInfo.append(path3)

    # path 5
    # path5 = PathInfo()
    # path5.pathID = 5
    # path5.typePath = 1
    # path5.pointOne = Pose()
    # path5.pointOne.position.x = -3.413
    # path5.pointOne.position.y = -10.073
    # path5.pointSecond = Pose()
    # path5.pointSecond.position.x = -0.878
    # path5.pointSecond.position.y = -10.073
    # path5.velocity = 0.3
    # msg.pathInfo.append(path5)

    # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 6
    # path3.typePath = 3
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = -0.878
    # path3.pointOne.position.y = -10.073
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = 0.122
    # path3.pointSecond.position.y = -9.073
    # path3.velocity = 0.2
    # path3.numberPts = 150
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = 0.122
    # path3.pointMid.position.y = -10.073
    # msg.pathInfo.append(path3)

    # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 7
    # path3.typePath = 3
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = 0.122
    # path3.pointOne.position.y = -9.073
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = -0.878
    # path3.pointSecond.position.y = -7.681
    # path3.velocity = 0.2
    # path3.numberPts = 150
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = 0.122
    # path3.pointMid.position.y = -7.681
    # msg.pathInfo.append(path3)

    # ------------------------------------------------------------------------------

    # path 1
    # path1 = PathInfo()
    # path1.pathID = 1
    # path1.typePath = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -5.698
    # path1.pointOne.position.y = -7.792
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -1.585
    # path1.pointSecond.position.y = -7.792
    # path1.velocity = 0.3
    # msg.pathInfo.append(path1)

    # # # path 2 Benze 
    # path2 = PathInfo()
    # path2.pathID = 2
    # path2.typePath = 3
    # path2.pointOne = Pose()
    # path2.pointOne.position.x = -1.585
    # path2.pointOne.position.y = -7.792
    # path2.pointSecond = Pose()
    # path2.pointSecond.position.x = -0.085
    # path2.pointSecond.position.y = -6.292
    # path2.velocity = 0.18
    # path2.numberPts = 150
    # path2.pointMid = Pose()
    # path2.pointMid.position.x = -0.085
    # path2.pointMid.position.y = -7.792
    # msg.pathInfo.append(path2)

    # # path 1
    # path1 = PathInfo()
    # path1.pathID = 3
    # path1.typePath = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -0.085
    # path1.pointOne.position.y = -6.292
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -0.085
    # path1.pointSecond.position.y = -3.576
    # path1.velocity = 0.3
    # msg.pathInfo.append(path1)


    # -----------------------------------------------------------------------------------
    # path 1
    # path1 = PathInfo()
    # path1.pathID = 1
    # path1.typePath = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -20.149
    # path1.pointOne.position.y = -2.873
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -23.229
    # path1.pointSecond.position.y = -2.873
    # path1.velocity = 0.35
    # msg.pathInfo.append(path1)

    # # path 2 Benze 
    # path2 = PathInfo()
    # path2.pathID = 2
    # path2.typePath = 3
    # path2.pointOne = Pose()
    # path2.pointOne.position.x = -23.229
    # path2.pointOne.position.y = -2.873
    # path2.pointSecond = Pose()
    # path2.pointSecond.position.x = -24.729
    # path2.pointSecond.position.y = -4.598
    # path2.velocity = 0.25
    # path2.numberPts = 150
    # path2.pointMid = Pose()
    # path2.pointMid.position.x = -24.729
    # path2.pointMid.position.y = -2.873
    # msg.pathInfo.append(path2)

    # # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 3
    # path3.typePath = 3
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = -24.729
    # path3.pointOne.position.y = -4.598
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = -23.229
    # path3.pointSecond.position.y = -6.098
    # path3.velocity = 0.25
    # path3.numberPts = 150
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = -24.729
    # path3.pointMid.position.y = -6.098
    # msg.pathInfo.append(path3)

    # # path 5
    # path5 = PathInfo()
    # path5.pathID = 5
    # path5.typePath = 1
    # path5.pointOne = Pose()
    # path5.pointOne.position.x = -23.229
    # path5.pointOne.position.y = -6.098
    # path5.pointSecond = Pose()
    # path5.pointSecond.position.x = -20.149
    # path5.pointSecond.position.y = -6.098
    # path5.velocity = 0.35
    # msg.pathInfo.append(path5)

    # # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 6
    # path3.typePath = 3
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = -20.149
    # path3.pointOne.position.y = -6.098
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = -18.649
    # path3.pointSecond.position.y = -4.373
    # path3.velocity = 0.25
    # path3.numberPts = 150
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = -18.649
    # path3.pointMid.position.y = -6.098
    # msg.pathInfo.append(path3)

    # # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 7
    # path3.typePath = 3
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = -18.649
    # path3.pointOne.position.y = -4.373
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = -20.149
    # path3.pointSecond.position.y = -2.873
    # path3.velocity = 0.25
    # path3.numberPts = 150
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = -18.649
    # path3.pointMid.position.y = -2.873
    # msg.pathInfo.append(path3)


    #---------------------------------------------------------------------------------------
    # path 1
    # path1 = PathInfo()
    # path1.pathID = 1
    # path1.typePath = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -4.866
    # path1.pointOne.position.y = -10.184
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -3.724
    # path1.pointSecond.position.y = -7.747
    # path1.velocity = 0.3
    # path1.movableZone = 1.2
    # msg.pathInfo.append(path1)

    # # path 1
    # path1 = PathInfo()
    # path1.pathID = 2
    # path1.typePath = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -3.724
    # path1.pointOne.position.y = -7.747
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -0.724
    # path1.pointSecond.position.y = -7.747
    # path1.velocity = 0.3
    # path1.movableZone = 1.2
    # msg.pathInfo.append(path1)

    # # path 1
    # path1 = PathInfo()
    # path1.pathID = 3
    # path1.typePath = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -0.724
    # path1.pointOne.position.y = -7.747
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -0.724
    # path1.pointSecond.position.y = -3.523
    # path1.velocity = 0.3
    # path1.movableZone = 1.2
    # msg.pathInfo.append(path1)

    #---------------------------------------------------------------------------------------

    # # path 1
    # path1 = PathInfo()
    # path1.pathID = 6
    # path1.typePath = 1
    # path1.direction = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -6.832
    # path1.pointOne.position.y = -6.84
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -6.332
    # path1.pointSecond.position.y = -6.84
    # path1.velocity = 0.3
    # path1.movableZone = 1.2
    # msg.pathInfo.append(path1)

    # # path 1
    # path1 = PathInfo()
    # path1.pathID = 7
    # path1.typePath = 1
    # path1.direction = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -6.332
    # path1.pointOne.position.y = -6.84
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -2.973
    # path1.pointSecond.position.y = -6.84
    # path1.velocity = 0.3
    # path1.movableZone = 1.2
    # msg.pathInfo.append(path1)


    # # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 2
    # path3.typePath = 3
    # path3.direction = 1
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = -2.973
    # path3.pointOne.position.y = -6.84
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = -0.956
    # path3.pointSecond.position.y = -5.26
    # path3.velocity = 0.2
    # path3.numberPts = 150
    # path3.movableZone = 1.2
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = -0.956
    # path3.pointMid.position.y = -6.84
    # msg.pathInfo.append(path3)

    # # path 1
    # path1 = PathInfo()
    # path1.pathID = 3
    # path1.typePath = 1
    # path1.direction = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -0.956
    # path1.pointOne.position.y = -5.26
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -0.956
    # path1.pointSecond.position.y = -2.951
    # path1.velocity = 0.3
    # path1.movableZone = 1.2
    # msg.pathInfo.append(path1)

    # # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 4
    # path3.typePath = 3
    # path3.direction = 1
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = -0.956
    # path3.pointOne.position.y = -2.951
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = -2.556
    # path3.pointSecond.position.y = -1.496
    # path3.velocity = 0.2
    # path3.numberPts = 150
    # path3.movableZone = 1.2
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = -0.956
    # path3.pointMid.position.y = -1.496
    # msg.pathInfo.append(path3)

    # # path 1
    # path1 = PathInfo()
    # path1.pathID = 5
    # path1.typePath = 1
    # path1.direction = 2
    # path1.pointOne = Pose() 
    # path1.pointOne.position.x = -2.556
    # path1.pointOne.position.y = -1.496
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = 0.667
    # path1.pointSecond.position.y = -1.496
    # path1.velocity = 0.3
    # path1.movableZone = 1.2
    # msg.pathInfo.append(path1)

    # ----------------------------------------------------------------

    # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 1
    # path3.typePath = 3
    # path3.direction = 1
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = 0.667
    # path3.pointOne.position.y = -1.496
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = -0.956
    # path3.pointSecond.position.y = -2.951
    # path3.velocity = 0.2
    # path3.numberPts = 150
    # path3.movableZone = 1.2
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = -0.956
    # path3.pointMid.position.y = -1.496
    # msg.pathInfo.append(path3)

    # # path 1
    # path1 = PathInfo()
    # path1.pathID = 2
    # path1.typePath = 1
    # path1.direction = 1
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -0.956
    # path1.pointOne.position.y = -2.951
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -0.956
    # path1.pointSecond.position.y = -5.26
    # path1.velocity = 0.3
    # path1.movableZone = 1.2
    # msg.pathInfo.append(path1)

    # # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 3
    # path3.typePath = 3
    # path3.direction = 1
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = -0.956
    # path3.pointOne.position.y = -5.26
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = -2.973
    # path3.pointSecond.position.y = -6.84
    # path3.velocity = 0.2
    # path3.numberPts = 150
    # path3.movableZone = 1.2
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = -0.956
    # path3.pointMid.position.y = -6.84
    # msg.pathInfo.append(path3)

    # # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 4
    # path3.typePath = 3
    # path3.direction = 1
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = -2.973
    # path3.pointOne.position.y = -6.84
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = -4.832
    # path3.pointSecond.position.y = -8.497
    # path3.velocity = 0.2
    # path3.numberPts = 150
    # path3.movableZone = 1.2
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = -4.832
    # path3.pointMid.position.y = -6.84
    # msg.pathInfo.append(path3)

    # # path 3 Benze 
    # path3 = PathInfo()
    # path3.pathID = 5
    # path3.typePath = 3
    # path3.direction = 2
    # path3.pointOne = Pose()
    # path3.pointOne.position.x = -4.832
    # path3.pointOne.position.y = -8.497
    # path3.pointSecond = Pose()
    # path3.pointSecond.position.x = -6.332
    # path3.pointSecond.position.y = -6.84
    # path3.velocity = 0.15
    # path3.numberPts = 150
    # path3.movableZone = 1.2
    # path3.pointMid = Pose()
    # path3.pointMid.position.x = -4.832
    # path3.pointMid.position.y = -6.84
    # msg.pathInfo.append(path3)

    # # path 1
    # path1 = PathInfo()
    # path1.pathID = 6
    # path1.typePath = 1
    # path1.direction = 2
    # path1.pointOne = Pose()
    # path1.pointOne.position.x = -6.332
    # path1.pointOne.position.y = -6.84
    # path1.pointSecond = Pose()
    # path1.pointSecond.position.x = -6.832
    # path1.pointSecond.position.y = -6.84
    # path1.velocity = 0.15
    # path1.movableZone = 1.2
    # msg.pathInfo.append(path1)

    # # path 1
    path1 = PathInfo()
    path1.pathID = 3
    path1.typePath = 1
    path1.direction = 1
    path1.pointOne = Pose()
    path1.pointOne.position.x = -0.524
    path1.pointOne.position.y = -9.728
    path1.pointSecond = Pose()
    path1.pointSecond.position.x = -0.483
    path1.pointSecond.position.y = -2.480
    path1.velocity = 0.3
    path1.movableZone = 1.2
    msg.pathInfo.append(path1)



    while not rospy.is_shutdown():


        rospy.loginfo("in procress PUB")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass