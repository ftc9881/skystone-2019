import java.util.*;

//experiments with a possible odometry set up
public class OdometryExperiment {
	//we'll call 2 F/B wheels A and B, L/R wheel C
	
	public static void main(String[] args) {
		double x = 0;
		double y = 0;
		final double CLICKS_INCHES = 200;
		
		System.out.println("testing calcFB:");
		//quarter turn to left (right wheel stationary)
		double[] t1 = new double[]{Math.PI/4, 0, 0, 5};
		System.out.printf("calcFB(%f, %f, %f, %f)%n", t1[0], t1[1], t1[2], t1[3]);
		System.out.println("result: " + calcFB(t1[0], t1[1], t1[2], t1[3]));
		//quarter turn to right (left wheel stationary)
		double[] t2 = new double[]{-Math.PI/4, 0, 5, 0};
		System.out.printf("calcFB(%f, %f, %f, %f)%n", t2[0], t2[1], t2[2], t2[3]);
		System.out.println("result: " + calcFB(t2[0], t2[1], t2[2], t2[3]));
		//quarter rotation around center 
		double[] t3 = new double[]{-Math.PI/4, 0, 5, -5};
		System.out.printf("calcFB(%f, %f, %f, %f)%n", t3[0], t3[1], t3[2], t3[3]);
		System.out.println("result: " + calcFB(t3[0], t3[1], t3[2], t3[3]));

		System.out.println("testing calcLR:");
		//right 5 @ pi/4
		double[] t4 = new double[]{Math.PI/4, 5};
		System.out.printf("calcFB(%f, %f)%n", t4[0], t4[1]);
		System.out.println("result: " + calcLR(t4[0], t4[1]));
		//left 5 @ 3pi/4
		double[] t5 = new double[]{3*Math.PI/4, -5};
		System.out.printf("calcFB(%f, %f)%n", t5[0], t5[1]);
		System.out.println("result: " + calcLR(t5[0], t5[1]));
	}
	
	//we assume IMU readings are in radians, dA and dB in inches
	//FB is arbitary, this is just an algorithm for odometry with two wheels in an axis
	//in theory we could do weighted average if center of rotation not halfway between A and B
	//returns dX, dY
	public static Coord calcFB (double currIMU, double prevIMU, double dA, double dB) {
		//right is positive, left is negative
		double dDir = prevIMU - currIMU;
		double dX, dY;

		//case drive straight
		//0.01 to be adjusted as neccessary
		if (Math.abs(dDir) < 0.01) {
			dX = ((dA + dB) / 2.0) * Math.cos(prevIMU);
			dY = ((dA + dB) / 2.0) * Math.sin(currIMU);
		//case making a turn: may not be neccessary, error reduction is probably minimal
		} else {
			//turn radius
			double r = ((dA + dB) / 2.0) * (1.0 / dDir);
			
			dX = r*Math.cos(prevIMU) - r*Math.cos(currIMU);
			dY = r*Math.sin(prevIMU) - r*Math.sin(currIMU);
		}
		return new Coord(dX, dY);
	}

	//we assume IMU readings are in radians, dA and dB in inches
	//LR is arbitary, this is just an algorithm for odometry with 1 wheel in an axis
	//returns dX, dY
	public static Coord calcLR (double IMU, double dC) {
		//basically polar to rectangular conversion
		return new Coord(dC*Math.cos(IMU), dC*Math.sin(IMU));
	}
}

class Coord {
	public double x;
	public double y;

	public Coord (double x, double y) {
		this.x = x;
		this.y = y;
	}

	public Coord add(Coord other) {
		return new Coord(x + other.x, y + other.y);
	}

	public Coord subtract(Coord other) {
		return new Coord(x - other.x, y - other.y);
	}

	public String toString() {
		return (x + ", " + y);
	}
}
