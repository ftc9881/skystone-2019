import java.io.*;
import java.util.*;

public class PIDDrive {
	/*
	 * Sort of a stretch but we'll see
	 * Assumes Mecanum drive
	 */

	//we can probably use mochel's nifty config for this
	//PID settings for direction and translation
	final double DIR_P = 0.01, DIR_I = 0.1, DIR_D = 1;
	final double VEL_P = 0.01, VEL_I = 0.1, VEL_D = 1;
	

	//absolute direction settings for next waypoint
	//we can probably read from a file for this
	double dirSet, velSet, xSet, ySet;

	//output from PID, moveDir is relative to robot
	double moveDir, PIDVel, PIDRotation;

	//0 is in positive x direction
	double robotDir, robotVel, robotX, robotY, prevX, prevY, prevDir, prevVel;

	//directions of wheels if sent positive input
	double RFDrive, LFDrive, RBDrive, LBDrive; 

	//Info PID uses
	double timeSinceLastPoint, dTime, intDir, intVel, dDir, dVel;

	public static void main(String[] args) {
		//calculation code should all be in functions/class
	}

	//returns an array of doubles representing motor values based on:
	//desired velocity (inches/s)?
	//direction relative to robot heading (will need conversion from absolute)
	//desired rotational velocity (radians/s)?
	public void updatePID() {
		robotVel = Math.sqrt(Math.pow(robotX-prevX, 2) + Math.pow(robotY-prevY, 2));

		intDir += dTime * (robotDir - dirSet);
		dDir = (robotDir - prevDir) / dTime;

		intVel += dTime * (robotVel - velSet);
		dVel = (robotVel - prevVel) / dTime;
			
		prevDir = robotDir;
		prevX = robotX;
		prevY = robotY;
		prevVel = robotVel;
	}
	public void setPIDVel() {
		PIDVel = VEL_P * ((robotVel - velSet) + (VEL_I * intVel) - (VEL_D * dVel));
	}
	public void setPIDRotation() {
		PIDRotation = DIR_P * ((robotDir - dirSet) + (DIR_I * intDir) - (DIR_D * dDir));
	}
	public void setMoveDir() {
		moveDir = Math.atan((ySet - robotY)/(xSet - robotX));
		if (ySet - robotY < 0) {
			moveDir += Math.PI/2;
		}
	}

	public void setDrives() {
		//does trig stuff, dm for more info ._. 
		RFDrive = PIDVel * Math.sin(moveDir + (Math.PI/4)) + PIDRotation;
		LFDrive = PIDVel * Math.sin(moveDir - (Math.PI/4)) - PIDRotation;
		RBDrive = PIDVel * Math.cos(moveDir - (Math.PI/4)) + PIDRotation;
		LBDrive = PIDVel * Math.cos(moveDir + (Math.PI/4)) - PIDRotation;
	}
}
