#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
from AgentTrajectories import *
import uav_trajectory

#define the final position in one big list or several smaller arrays
#define our total time
time = 8.0 #total time in seconds
z = 1.0
#for now, static final positions for agents in order
#consider a swarm of size 3 with zero initial velocity and arbitrary final pos.
#2345
pf = np.array([ [0.0, 0.6, 1.0],
                [-0.6, 0.0, 1.3],
                [0.6, 0.0, 0.5],
                [0.0, -0.6, 1.3] ] )

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    i = 0;20
    for cf in allcfs.crazyflies:
	    #get the initial position from cf.initialPosition
	    #get the final position from the variables at the top of the file

        #for the crazyflies, p0i and pfi are relative, so we want to go from (0,0) to pfi - p0i
        print(cf.id)
        p0i = np.array(cf.initialPosition) + np.array([0.00, 0.00, z])  #p0

        pfi = pf[i,:] - p0i  #pf converted to the crazyflie coordinates
        p0i = p0i - p0i #p0 converted rto crazyflie coordinates (0, 0)

        v0i = np.array([0.00, 0.00, 0.00])  #v0
        vfi = np.array([0.00, 0.00, 0.00])  #vf
        t0 = 0.0  #t0
        tf = 10.0  #tf
	    #TODO use the controller to get a polynomial. One for x, one for y. z and yaw can be zero
        # cx   = np.array([0.83, -0.28, -0.38, 0.18])
        # cy   = np.array([-1.36, 0.69, 0.59, -0.33])
        cx, cy, cz = AgentTrajectories(p0i, pfi, v0i, vfi, t0, tf)
        #cx = cx[::-1]  #for the refersal
        #cy = cy[::-1]
        cx[0] = cx[0]/6.0
        cx[1] = cx[1]/2
        cy[0] = cy[0]/6
        cy[1] = cy[1]/2
        #cz   = np.array([0, 0, 0, 0])
        cz[0] = cz[0]/6
        cz[1] = cz[1]/2
        cyaw = np.array([0, 0, 0, 0])

        cx = np.flip(cx)
        cy = np.flip(cy)
        cz = np.flip(cz)
        cyaw = np.flip(cyaw)



	    #combine the coefficients into one big array that matches the format of figure8.csv
        #pad the left (t^7 - t^4 coefficients) with zeros
        coefs = np.array([time]) #duration is first
        coefs = np.append(coefs, cx)
        coefs = np.append(coefs, np.zeros(4))
        coefs = np.append(coefs, cy)
        coefs = np.append(coefs, np.zeros(4))
        coefs = np.append(coefs, cz)
        coefs = np.append(coefs, np.zeros(4))
        coefs = np.append(coefs, np.zeros(8)) #z and yaw
        #stack an empty trajectory on top of ours with the same length
        poly = np.stack((np.zeros(33), coefs))

        #create and upload the polynomial trajectory
    	traj = uav_trajectory.Trajectory() #creates an empty trajectory object
        traj.buildTrajectory(poly)
        cf.uploadTrajectory(0, 0, traj)
        print(cf.position())


        #increment Crazyflie index
        i += 1


    allcfs.takeoff(targetHeight=z, duration=2.0)
    timeHelper.sleep(2.5)
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, z])
        cf.goTo(pos, 0, 1.0)
    timeHelper.sleep(3.0)
    allcfs.startTrajectory(0, timescale=1.0)
    timeHelper.sleep(14)
    allcfs.land(targetHeight=0.06, duration=3.0)
    timeHelper.sleep(4.0)
