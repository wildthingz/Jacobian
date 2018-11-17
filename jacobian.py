#!/usr/bin/env python

####################################################################################
## -o   ############################################################################
##  |   ############################################################################
##  o   ############################ Jacobian Construction #########################
##  |   ############################################################################
##  o   ################################ Hatef Khadivi #############################
##  M   ############################################################################
##      ############################################################################
####################################################################################
####################################################################################
## [-] O  :  Origin (O)                                                            #
##   [-] origin is set on the shoulder joint, with x in direction of the shoulder, #
##       y perpendicular to x in horizontal plane towards the screen               #
##       and z in vertical downwards, direction.                                   #
##   [-] notice the drawing above for approximate representation of the arm.       #
##   [-] note: henceforth, local coordinates will be formed in this manner.        #
##                                                                                 #
## [-] I  :  Shoulder flexion (SF)                                                 #
##  [-] rotation along x axis of the O                                             #
##                                                                                 #
## [-] II :  Shoulder abduction (SA)                                               #
##  [-] rotation along y axis of SF                                                #
##                                                                                 #
## [-] III:  Shoulder rotation (SR)                                                #
##  [-] rotation along z axis of SA                                                #
##  [-] translation along z axis of SA for L1                                      #
##                                                                                 #
## [-] IV :  Elbow flexion (EF)                                                    #
##  [-] rotation along x axis of SR                                                #
##                                                                                 #
## [-] V  :  Elbow rotation (ER)                                                   #
##  [-] rotation along z axis of EF                                                #
##  [-] translation along z axis of EF for L2                                      #
##                                                                                 #
## [-] VI :  Wrist abduction (WF)                                                    #
##  [-] rotation y axis of ER                                                      #
##                                                                                 #
####################################################################################

import numpy as np


class ARM_FTM:
    '''
    For forward transformation matrices
    '''
    def __init__(self, L1, L2, L3):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

        self.SF_ROT     = lambda θ : np.array([[1, 0, 0, 0], [0, np.cos(θ), np.sin(θ), 0], [0, -np.sin(θ), np.cos(θ), 0], [0, 0, 0, 1]])
        self.SA_ROT     = lambda θ : np.array([[np.cos(θ), 0, np.sin(θ), 0], [0, 1, 0, 0], [-np.sin(θ), 0, np.cos(θ), 0], [0, 0, 0, 1]])
        self.SR_ROT     = lambda θ : np.array([[np.cos(θ), -np.sin(θ), 0, 0], [np.sin(θ), np.cos(θ), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.SR_TRANS   = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self.L1], [0, 0, 0, 1]])
        self.EF_ROT     = lambda θ : np.array([[1, 0, 0, 0], [0, np.cos(θ), np.sin(θ), 0], [0, -np.sin(θ), np.cos(θ), 0], [0, 0, 0, 1]])
        self.ER_ROT     = lambda θ : np.array([[np.cos(θ), -np.sin(θ), 0, 0], [np.sin(θ), np.cos(θ), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.ER_TRANS   = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self.L2], [0, 0, 0, 1]])
        self.WA_ROT     = lambda θ : np.array([[np.cos(θ), 0, -np.sin(θ), 0], [0, 1, 0, 0], [np.sin(θ), 0, np.cos(θ), 0], [0, 0, 0, 1]])
        self.WA_TRANS   = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self.L3], [0, 0, 0, 1]])


def main():
    L1 = 100
    L2 = 100
    L3 = 50

    point = np.array([[0, 0, 0, 1]])

    theta_SF_ROT = 30 * np.pi / 180
    theta_SA_ROT = 20 * np.pi / 180
    theta_SR_ROT = 5 * np.pi / 180
    theta_EF_ROT = 10 * np.pi / 180
    theta_ER_ROT = 5 * np.pi / 180
    theta_WA_ROT = 10 * np.pi / 180

    test = ARM_FTM(L1, L2, L3)

    transform_SF_ROT = test.SF_ROT(theta_SF_ROT)
    transform_SA_ROT = test.SA_ROT(theta_SA_ROT)
    transform_SR_ROT = test.SR_ROT(theta_SR_ROT)
    transform_SR_TRANS = test.SR_TRANS
    transform_EF_ROT = test.EF_ROT(theta_EF_ROT)
    transform_ER_ROT = test.ER_ROT(theta_ER_ROT)
    transform_ER_TRANS = test.ER_TRANS
    transform_WA_ROT = test.WA_ROT(theta_WA_ROT)
    transform_WA_TRANS = test.WA_TRANS

    #shoulder joint
    transform_SF2SA = np.matmul(transform_SF_ROT, transform_SA_ROT)
    transform_SA2SR = np.matmul(transform_SF2SA, transform_SR_ROT)
    transform_S = np.matmul(transform_SA2SR, transform_SR_TRANS)

    #elbow joint
    transform_EF2ER = np.matmul(transform_EF_ROT, transform_ER_ROT)
    transform_E = np.matmul(transform_EF2ER, transform_ER_TRANS)

    #wrist joint
    transform_W = np.matmul(transform_WA_ROT, transform_WA_TRANS)

    #combination
    transform_S2E = np.matmul(transform_S, transform_E)
    end_transform = np.matmul(transform_S2E, transform_W)


    elbowpoint = np.matmul(transform_S, point.T)
    wristpoint = np.matmul(transform_S2E, point.T)
    endpoint = np.matmul(end_transform, point.T)

    print("elbow point:\n{}\n".format(elbowpoint))
    print("wrist point:\n{}\n".format(wristpoint))
    print("end point:\n{}\n".format(endpoint))

if __name__ == '__main__':
    main()
