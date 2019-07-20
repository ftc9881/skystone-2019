import java.io.*;
import java.util.*;

public class ChadDrive {
	/*
	 * Sort of a stretch but we'll see
	 * Assumes Mecanum drive
	 * robotDir: 0 is RIGHT, FORWARDS is PI/2
	 * absStrafeDir: 0 is RIGHT, FORWARDS is PI/2
	 */

	//we can probably use mochel's nifty config for this
	//PID settings for direction and translation
	final double DIR_P = 0.01, DIR_I = 0.1, DIR_D = 1;
	final double VEL_P = 0.01, VEL_I = 0.1, VEL_D = 1;
	final double TOLERANCE = 0.5;
	
	boolean hasPoint = false;

	//absolute direction settings for next waypoint
	//we can probably read from a file for this
	double dirSet = 0, velSet = 0, xSet = 0, ySet = 0;

	//output from PID, moveDir is relative to robot
	double relStrafeDir, PIDVel, PIDRotation;

	//0 is in positive x direction
	double robotDir, absStrafeDir, robotVel, robotX, robotY, prevX, prevY, prevDir, prevVel;

	//output to wheels
	double RFDrive, LFDrive, RBDrive, LBDrive; 

	//Info PID uses
	double lastUpdate, absStrafeDir, dTime, intDir, intVel, dDir, dVel;

	double time;

	public ChadDrive () {
		//have mapping stuff later	
	}
	//returns an array of doubles representing motor values based on:
	//desired velocity (inches/s)?
	//direction relative to robot heading (will need conversion from absolute)
	//desired rotational velocity (radians/s)?
	public void updatePID(double t, double rDir, double rX, double rY) {
		time = t;
		dTime = t - lastUpdate;
		robotDir = rDir;
		robotX = rX; 
		robotY = rY;

		//Determine robotVel, combined with absStrafeDir represents movement
		robotVel = Math.sqrt(Math.pow(robotX-prevX, 2) + Math.pow(robotY-prevY, 2));

		//update integral of directional error
		intDir += dTime * (robotDir - dirSet);
		//update derivative of directional error
		dDir = (robotDir - prevDir) / dTime;
		//update integral of velocity error
		intVel += dTime * (robotVel - velSet);
		//update derivative of velocity error
		dVel = (robotVel - prevVel) / dTime;

		//calculate absolute direction to strafe towards w/ diff between robot pos and set pos
		absStrafeDir = Math.atan((ySet - robotY)/(xSet - robotX));
		//atan range stuff (account for atan (-1/-1) returns the same as atan(1/1))
		if (ySet - robotY < 0) absStrafeDir += Math.PI;
		

		//actual pid part
		PIDVel = VEL_P * ((robotVel - velSet) + (VEL_I * intVel) - (VEL_D * dVel));
		PIDRotation = DIR_P * ((robotDir - dirSet) + (DIR_I * intDir) - (DIR_D * dDir));
		relStrafeDir = strafeDir - robotDir;

		//logic here
		hasPoint = !atPoint();
		if (!hasPoint) {
			//does trig stuff, dm for more info ._. 
			RFDrive = PIDVel * Math.sin(relStrafeDir + (Math.PI/4)) + PIDRotation;
			LFDrive = PIDVel * Math.sin(relStrafeDir - (Math.PI/4)) - PIDRotation;
			RBDrive = PIDVel * Math.cos(relStrafeDir - (Math.PI/4)) + PIDRotation;
			LBDrive = PIDVel * Math.cos(relStrafeDir + (Math.PI/4)) - PIDRotation;
		} else {
			RFDrive = 0;
			LFDrive = 0;
			RBDrive = 0;
			LBDrive = 0;
		}
			
		prevDir = robotDir;
		prevX = robotX;
		prevY = robotY;
		prevVel = robotVel;
		lastUpdate = t;
	}

	public void setNextPoint(double dirS, double velS, double xS, double yS) {
		dirSet = dirS;
		velSet = velS;
		xSet = xS;
		ySet = yS;
		hasPoint = true;
	}

	public boolean atPoint() {
		return Math.sqrt(Math.pow(robotX-setX, 2) + Math.pow(robotY-setY, 2)) < TOLERANCE;
	} 
	
	public void dump() {
		System.out.printf("Time: %f \n", time);
		System.out.printf("Time since last update: %f \n", dTime);
	}
}
