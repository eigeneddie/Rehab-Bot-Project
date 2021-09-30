# Custom Functions
import numpy as np
import pandas as pd

def ATransformMatrix (theta): #A
    ATransform = np.array([[np.cos(theta), -np.sin(theta)], 
                            [np.sin(theta), np.cos(theta)]])
    return ATransform 

def ATransformMatrixTHETA (theta): #A_theta
    ATransformTHETA = np.array([[-np.sin(theta), -np.cos(theta)], 
                                [np.cos(theta), -np.sin(theta)]])
    return ATransformTHETA

def prettyMatVect(matVect):
    prettyMatVect = pd.DataFrame(matVect, columns =
                                ['R1x', 'R1y', 'theta1',
                                'R2x','R2y','theta2',
                                'R3x','R3y','theta3',
                                'R4x','R4y','theta4'])
    return prettyMatVect

def calcGenCoor(qi, u_bar_2O, u_bar_2A, u_bar_3A, u_bar_3B, u_bar_4B):
    # To calculate Point of Interest positions
    # GLOBAL COORDINATES
    R1 = np.array([qi[0], qi[1]], dtype = float)
    R2 = np.array([qi[3], qi[4]], dtype = float)

    # Point O at link 1
    r1O = R1
    # Point O at link 2
    r2O = R2 + np.matmul(ATransformMatrix(float(qi[5])), u_bar_2O)

    # Point A at link 2
    r2A = R2 + np.matmul(ATransformMatrix(float(qi[5])), u_bar_2A)
    # Point A at link 3
    R3 = np.array([qi[6], qi[7]], dtype = float) 
    r3A = R3 + np.matmul(ATransformMatrix(float(qi[8])), u_bar_3A)

    # Point B at link 3
    r3B = R3 + np.matmul(ATransformMatrix(float(qi[8])),u_bar_3B)
    # Point B at link 4
    R4 = np.array([qi[9], qi[10]], dtype = float)
    r4B = R4 + np.matmul(ATransformMatrix(float(qi[11])),u_bar_4B)

    return r1O, r2O, r2A, r3A, r3B, r4B

def constraintEquation(constraintVector, qi, r1O, r2O, r2A, 
                    r3A, r3B, r4B, sliderHeight, theta2Initial,
                    omega2, timeNow):
    # Ground constraint
    for i in range(3):
        constraintVector[i] = qi[i]

    # Pin joint O
    constraintPinO = r2O # r1O -
    for i in range(np.size(constraintPinO)):
    # Equation 4-5
        constraintVector[i+3] = constraintPinO[i]

    # Pin joint A
    constraintPinA = r2A - r3A
    for i in range(np.size(constraintPinA)):
    # Equation 6-7
        constraintVector[i+5] = constraintPinA[i]

    # Pin joint B
    constraintPinB = r3B - r4B
    for i in range(np.size(constraintPinB)):
    # Equation 8-9
        constraintVector[i+7] = constraintPinB[i]

    # Equation 10 - 12
    constraintVector[9] = qi[10]-sliderHeight
    constraintVector[10] = qi[11] - 0
    constraintVector[11] = qi[5] - theta2Initial - omega2*timeNow 

    return constraintVector

def jacobianMatrix(qi, u_bar_2O, u_bar_2A, u_bar_3A, u_bar_3B, u_bar_4B):
    n = np.size(qi)
    jacobianMatrixCq = np.zeros((n,n), dtype = float)
    identity3x3 = np.identity(3)
    identity2x2 = np.identity(2)

    # row 1-3
    jacobianMatrixCq[0:3,0:3] = identity3x3

    # row 4-5
    Cq45 = np.matmul(ATransformMatrixTHETA(float(qi[5])), u_bar_2O)
    jacobianMatrixCq[3:5,3:5] = identity2x2
    jacobianMatrixCq[3:5,5:6] = Cq45

    # row 6-7
    Cq67_link2 = np.matmul(ATransformMatrixTHETA(float(qi[5])), u_bar_2A)
    Cq67_link3 = np.matmul(ATransformMatrixTHETA(float(qi[8])), u_bar_3A)
    jacobianMatrixCq[5:7,3:5] = identity2x2
    jacobianMatrixCq[5:7,5:6] = Cq67_link2
    jacobianMatrixCq[5:7,6:8] = -identity2x2
    jacobianMatrixCq[5:7,8:9] = -Cq67_link3

    # row 8-9
    Cq89_link3 = np.matmul(ATransformMatrixTHETA(float(qi[8])), u_bar_3B)
    Cq89_link4 = np.matmul(ATransformMatrixTHETA(float(qi[11])), u_bar_4B)
    jacobianMatrixCq[7:9,6:8] = identity2x2
    jacobianMatrixCq[7:9,8:9] = Cq89_link3
    jacobianMatrixCq[7:9,9:11] = -identity2x2
    jacobianMatrixCq[7:9,11:12] = -Cq89_link4

    # row 10-11
    jacobianMatrixCq[9:11,10:12] = identity2x2

    # row 12
    jacobianMatrixCq[11][5] = 1

    return jacobianMatrixCq

def constrEqDot(constraintVectorVel, omega):

    constraintVectorVel[11] = -omega

    return constraintVectorVel

def positionAnalysis(constraintVector, jacobianMatrix, qi):
    inverse_jacobian = np.linalg.inv(jacobianMatrix)
    delta_qi = - np.matmul(inverse_jacobian, constraintVector)
    delta_qi_norm = np.linalg.norm(delta_qi)
    qi = qi + delta_qi

    return qi, delta_qi_norm

def velocityAnalysis(constraintVectorVel, jacobianMatrix, qiDot):
    inverse_jacobian = np.linalg.inv(jacobianMatrix)
    qiDot= - np.matmul(inverse_jacobian, constraintVectorVel)

    return qiDot

def accelerationAnalysis(jacobianMatrix, qiDotDot, qiDot, qi,
                        u_bar_2O, u_bar_2A, u_bar_3A, u_bar_3B, u_bar_4B):
    # Qd line 3-4                    
    Qd34 = np.square(float(qiDot[5]))*np.matmul(ATransformMatrix(float(qi[5])), u_bar_2O)
    
    # Qd line 5-6
    Qd56a = np.square(float(qiDot[5]))*np.matmul(ATransformMatrix(float(qi[5])), u_bar_2A)
    Qd56b = np.square(float(qiDot[8]))*np.matmul(ATransformMatrix(float(qi[8])), u_bar_3A)
    Qd56 = Qd56a-Qd56b

    # Qd line 7-8
    Qd78a = np.square(float(qi[8]))*np.matmul(ATransformMatrix(float(qi[8])), u_bar_3B)
    Qd78b = np.square(float(qi[11]))*np.matmul(ATransformMatrix(float(qi[11])), u_bar_4B)
    Qd78 = Qd78a-Qd78b

    Qd = np.array([[0],[0],[0], Qd34[0], Qd34[1],
                    Qd56[0], Qd56[1], Qd78[0], Qd78[1], 
                    [0], [0], [0]])
    
    inverse_jacobian = np.linalg.inv(jacobianMatrix)
    qiDotDot= np.matmul(inverse_jacobian, Qd)

    return qiDotDot