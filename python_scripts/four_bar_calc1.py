#Multibody DYNAMICS CALCULATION

#import necessary packages
import numpy as np  
import matplotlib.pyplot as plt
import functionScriptMB as fc # Important custom functions
from functionScriptMB import ATransformMatrix as ATrans #Transformation matrix A_i

# SLIDER MECHANISM
# 1. KNOWN PARAMETERS 
# May also be data fetched from somewhere, I dunno
lengthLink2 = 20 # [cm]
lengthLink3 = 40 # [cm]
sliderHeight = 7 # [cm]
omega2 = 20 # [rad/s]
theta2Initial = np.pi/3 # [rad]
stepTime = 0.001 # [s] time increment
timeStart = 0 # [s] start time
timeEnd = 2 # [s] end time

# Create simulation time and reserve space for vector
simulTime = np.arange(timeStart, timeEnd, stepTime).T

# Calculate:
# a. Slider

sliderRx4 = np.zeros([np.size(simulTime)], dtype = float).T #Slider position
sliderVelocityRx4 = np.zeros([np.size(simulTime)], dtype = float).T #Slider velocity
sliderAccelerationRx4 = np.zeros([np.size(simulTime)], dtype = float).T #Slider acceleration


# b. Link 3

omegaLink3 = np.zeros([np.size(simulTime)], dtype = float).T #Angular Velocity Link 3

alphaLink3 = np.zeros([np.size(simulTime)], dtype = float).T #Angular Acceleration Link 3



# c. Point A

trajPointA = np.zeros([2, np.size(simulTime)], dtype = float).T #Point A trajectory





# 2. DEFINING GENERALIZED COORDINATES AND POINTS OF INTEREST

# LOCAL COORDINATES V

# Point O

u_bar_1O = np.array([[0], [0]], dtype = float)

u_bar_2O = np.array([[-lengthLink2/2], [0]], dtype = float)



# Point A

u_bar_2A = np.array([[lengthLink2/2], [0]], dtype = float) 

u_bar_3A = np.array([[-lengthLink3/2], [0]], dtype = float)



# Point B

u_bar_3B = np.array([[lengthLink3/2], [0]], dtype = float)

u_bar_4B = np.array([[0],[0]], dtype = float) # --> SLIDER LOCAL



# Generalized Coordinate vector!!

qi = np.array([np.zeros(12)], dtype = float).T #position

qiDot = np.array([np.zeros(12)], dtype = float).T # velocity

qiDotDot = np.array([np.zeros(12)], dtype = float).T # acceleration



# 3. CONSTRAINT EQUATION

constraintVector = np.array([np.zeros(np.size(qi))], dtype = float).T

constraintVectorDot = np.array([np.zeros(np.size(qi))], dtype = float).T

epsilon = 0.0000000000000000001

timeNow = timeStart



for timeID in range(np.size(simulTime)):

    max_iteration = 30

    count = 0

    delta_qi_norm = 1



    # FOR EVERY TIME STEPP!!

    while delta_qi_norm > epsilon:

        # Calculate Point of Interests

        r1O, r2O, r2A, r3A, r3B, r4B = fc.calcGenCoor(qi, u_bar_2O, u_bar_2A, 

                                                u_bar_3A, u_bar_3B, u_bar_4B)

        

        # 4. CONSTRAINT EQUATION C

        constraintVector = fc.constraintEquation(constraintVector, qi, r1O, r2O, r2A, 

                                            r3A, r3B, r4B, sliderHeight, theta2Initial,

                                            omega2, timeNow)



        constraintVectorDot = fc.constrEqDot(constraintVectorDot, omega2)



        # 5. JACOBIAN MATRIX Cq

        jacobianMatrixCq = fc.jacobianMatrix(qi, u_bar_2O, u_bar_2A, u_bar_3A, 

                                                u_bar_3B, u_bar_4B)

        

        # 6. POSITION ANALYSIS

        qi, delta_qi_norm = fc.positionAnalysis(constraintVector,

                                                jacobianMatrixCq, qi)



        # 7. Velocity ANLAYSIS

        qiDot = fc.velocityAnalysis(constraintVectorDot, jacobianMatrixCq, qiDot)



        # 8. ACCELERATION ANALYSIS

        qiDotDot = fc.accelerationAnalysis(jacobianMatrixCq, qiDotDot, qiDot, qi,

                                u_bar_2O, u_bar_2A, u_bar_3A, u_bar_3B, u_bar_4B)

                                

        count = count + 1

        

        if (delta_qi_norm<epsilon) or (count>max_iteration):

            break



    sliderRx4[timeID] = qi[9]

    sliderVelocityRx4[timeID] = qiDot[9]

    sliderAccelerationRx4[timeID] = qiDotDot[9]

    omegaLink3[timeID] = qiDot[8]

    alphaLink3[timeID] = qiDotDot[8]



    R2 = np.array([qi[3], qi[4]], dtype = float).T

    r2A = R2 + np.matmul(ATrans(float(qi[5])), u_bar_2A).T

    trajPointA[timeID] = r2A



    timeNow = timeNow + stepTime



plt.figure(1)

plt.plot(simulTime, sliderRx4)

plt.title('SLIDER POSITION')

plt.ylabel('Position [cm]')

plt.xlabel('time [s]')

plt.grid(True)



plt.figure(2)

plt.plot(simulTime, sliderVelocityRx4)

plt.title('SLIDER VELOCITY')

plt.ylabel('Velocity [cm/s]')

plt.xlabel('time [s]')

plt.grid(True)




plt.figure(3)

plt.plot(simulTime, sliderAccelerationRx4)

plt.title('SLIDER ACCELERATION')

plt.ylabel('Slider Acceleration [cm/s^2]')

plt.xlabel('time [s]')

plt.grid(True)




plt.figure(4)

plt.plot(simulTime, omegaLink3)

plt.title('LINK 3 ANGULAR VELOCITY')

plt.ylabel('omega3 [rad/s]')

plt.xlabel('time [s]')

plt.grid(True)





plt.figure(5)

plt.plot(simulTime, alphaLink3)

plt.title('LINK 3 ANGULAR ACCELERATION')

plt.ylabel('Alpha3 [rad/s^2]')

plt.xlabel('time [s]')

plt.grid(True)





plt.figure(6)

plt.plot(trajPointA[:,0:1], trajPointA[:,1:2])

plt.title('Trajectory Point A')

plt.ylabel('Position y [cm]')

plt.xlabel('Position X [cm]')

plt.grid(True)

plt.show()



generalizezCoordinates_q = fc.prettyMatVect(np.transpose(qi))

print("Generalized Coordinates vector = ")

print(generalizezCoordinates_q)

print(" ")



constraintVector_C = fc.prettyMatVect(np.transpose(constraintVector))

print("Constraint Vector = ")

print(constraintVector_C)

print(" ")



constraintVectorDot_Ct = fc.prettyMatVect(np.transpose(constraintVectorDot))

print("Constraint Vector (velocity) = ")

print(constraintVectorDot_Ct)

print(" ")



jacobianMatrix_Cq = fc.prettyMatVect(jacobianMatrixCq)

print("Jacobian Matrix = ")

print(jacobianMatrix_Cq)

print(" ")



inverseJacob = fc.prettyMatVect(np.linalg.inv(jacobianMatrixCq))

print("Inverse Jacobian Matrix = ")

print(inverseJacob)

print(" ")



print("delta qi norm = ")

print(delta_qi_norm)

print(" ")



print("iteration count = ")

print(count)

print(" ")