import numpy as np
import matplotlib.pyplot as plt

# Here we simulate just the motor

# The goal is to turn the motor to a specified position
# Input:            motor torque
# State variables:  shaft angular position, shaft angular velocity
# Output:           shaft position

# Plan:
# 1. Numerically simulate the system
# 2. Add a controller
# 3. Simulate the system with the controller
# -----------------------------------------------------------------------------
   
# 1. Numerically simulate the system
# Differential equation for system:
# T = mmI*d^2(theta)/dt^2
# T = torque
# mmI = mass moment of inertia
# theta = angular position (radians)

# Writing as two first order ODEs
# T = mmI*d(x2)/dt
# x2 = d(x1)/dt
# where x1 = theta

# Approximating derivatives using Euler's methods, and solving for x1 (see Maple file)
# x2 = np.append(x2,(T[i]*delT+mmI*x2[i-1])/mmI) # angular velocity
# x1 = np.append(x1,delT*x2[i]+x1[i-1]) 

# Simulate the response of the motor to one torque input
def simOneInteraction(x1, x2, T, mmI, delT):
    i = len(x1)
    x2 = np.append(x2,(T*delT+mmI*x2[i-1])/mmI) # angular velocity
    x1 = np.append(x1,delT*x2[i]+x1[i-1])
    return [x1, x2]
   
# Plot results of motor simulation
def plotSim(x1,x2,Tseq,numIter,delT):
    t = np.arange(0, delT*numIter, delT);        
    theta = x1
    plt.plot(t,theta,'b',label = 'Angular Position')
    plt.plot(t,x2,'r',label = 'Angular Velocity')
    plt.plot(t,Tseq,'g',label = 'Torque')
    plt.legend(loc = 4)
    plt.title('Motor Shaft Simulation')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular position (rad), Angular velocity (rad/s)')
    plt.show()     

# Demo usage
def demoSim():    
    # System parameters
    mmI = 1 # kg*m^2

    # Length and precision of simulation
    numIter = 2000
    delT = 0.1 # Time interval between steps, in sconds  
    
    # Simulate
    x1 = np.array([0]); # Initial position
    x2 = np.array([0]); # Initial velocity
    xr = 10; # Desired position
    
    Tseq = np.array([0]);
    for i in range(numIter-1):
        # Control policy (proportional control)
        kp = 0.01      
        eNow = xr - x1[i]
        if (i > 0): # PD control           
            ePrev = xr - x1[i-1]
            kd = 1
            T = kp*eNow + kd*(eNow - ePrev )
        else: # P control
            T = kp*eNow
            
        # Keep track of torque used
        Tseq = np.append(Tseq,T)
        (x1,x2) = simOneInteraction(x1, x2, T, mmI, delT)      
    
    plotSim(x1,x2,Tseq,numIter,delT)

    
demoSim()
   
  




