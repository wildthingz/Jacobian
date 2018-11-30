#!/usr/bin/env python

'''
####################################################################################
## =o   ############################################################################
##  |   ############################################################################
##  =   ############################ Jacobian Construction #########################
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
## [-] VI :  Wrist abduction (WF)                                                  #
##  [-] rotation y axis of ER                                                      #
##                                                                                 #
####################################################################################
'''
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

    def linearTransform(self, angles):

        point = np.array([[0, 0, 0, 1]])

        theta_SF_ROT = angles[0]
        theta_SA_ROT = angles[1]
        theta_SR_ROT = angles[2]
        theta_EF_ROT = angles[3]
        theta_ER_ROT = angles[4]
        theta_WA_ROT = angles[5]

        transform_SF_ROT = self.SF_ROT(theta_SF_ROT)
        transform_SA_ROT = self.SA_ROT(theta_SA_ROT)
        transform_SR_ROT = self.SR_ROT(theta_SR_ROT)
        transform_SR_TRANS = self.SR_TRANS
        transform_EF_ROT = self.EF_ROT(theta_EF_ROT)
        transform_ER_ROT = self.ER_ROT(theta_ER_ROT)
        transform_ER_TRANS = self.ER_TRANS
        transform_WA_ROT = self.WA_ROT(theta_WA_ROT)
        transform_WA_TRANS = self.WA_TRANS

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

        elbowpointPosition = np.matmul(transform_S, point.T)
        wristpointPosition = np.matmul(transform_S2E, point.T)
        endpointPosition = np.matmul(end_transform, point.T)

        return(endpointPosition[:-1][:, 0])

        #test = nd.Jacobian(endpoint)(point.T)
        #print(test)
        #return(elbowpoint, wristpoint, endpoint)

    def angularTransform(self, angles):

        theta_SF_ROT = angles[0]
        theta_SA_ROT = angles[1]
        theta_SR_ROT = angles[2]
        theta_EF_ROT = angles[3]
        theta_ER_ROT = angles[4]
        theta_WA_ROT = angles[5]

        endpointOrientation = np.array([theta_SF_ROT + theta_EF_ROT, theta_SA_ROT - theta_WA_ROT, theta_SR_ROT + theta_ER_ROT])

        return endpointOrientation

    def jacobian(self, angles, eps = 1e-8):

        endpointPosition = self.linearTransform(angles)
        anglesPerturb = np.copy(angles)
        jacobianLinearVelocity = np.zeros((3, len(angles)))

        for i, _ in enumerate(angles):
            anglesPerturb[i] += eps
            jacobianLinearVelocity[:, i] = (self.linearTransform(anglesPerturb) - endpointPosition) / eps
            anglesPerturb[i] = angles[i]

        endpointOrientation = self.angularTransform(angles)
        anglesPerturb = np.copy(angles)
        jacobianAngularVelocity = np.zeros((3, len(angles)))

        for i, _ in enumerate(angles):
            anglesPerturb[i] += eps
            jacobianAngularVelocity[:, i] = (self.angularTransform(anglesPerturb) - endpointOrientation) / eps
            anglesPerturb[i] = angles[i]

        res = np.concatenate((jacobianLinearVelocity, jacobianAngularVelocity), axis = 0)

        return res

def main():
    L1 = 100
    L2 = 100
    L3 = 50

    angles = np.array([30 * np.pi / 180,
                        20 * np.pi / 180,
                        5 * np.pi / 180,
                        10 * np.pi / 180,
                        5 * np.pi / 180,
                        10 * np.pi / 180])

    test = ARM_FTM(L1, L2, L3)

    #elbowpoint, wristpoint, endpoint = test.transformMatrix(angles)
    #print("elbow point:\n{}\n".format(elbowpoint))
    #print("wrist point:\n{}\n".format(wristpoint))

    endpoint = test.linearTransform(angles)
    print("end point:\n{}\n".format(endpoint))

    jc = test.jacobian(angles)
    print(jc)

if __name__ == '__main__':
    main()
