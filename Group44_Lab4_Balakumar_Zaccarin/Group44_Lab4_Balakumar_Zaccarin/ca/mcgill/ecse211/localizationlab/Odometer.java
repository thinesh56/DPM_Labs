package ca.mcgill.ecse211.localizationlab;


import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
	private double x;
	private double y;
	private double theta;
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	public static double fakeTheta; // to avoid using theta outside of lock
	private int lastTachoL;
	private int lastTachoR;

	private static final long ODOMETER_PERIOD = 25; /*odometer update period, in ms*/

	private Object lock; /*lock object for mutual exclusion*/

	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0; // north has the value of 90°
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		lock = new Object();
		this.lastTachoL=0;
		this.lastTachoR=0;
		this.fakeTheta = 0.0;
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		//variables to update position
		double distL, distR, deltaD, deltaT, dX, dY;

		while (true) {
			updateStart = System.currentTimeMillis();
			//do calculations here
			int currentTachoL, currentTachoR;
			currentTachoL= leftMotor.getTachoCount();	 //update tachometer count
			currentTachoR = rightMotor.getTachoCount();  //update tachometer count
			distL = Math.PI *LocalizationLab.WHEEL_RADIUS*(currentTachoL - lastTachoL)/180; //convert tacho count to distance travelled
			distR= Math.PI*LocalizationLab.WHEEL_RADIUS*(currentTachoR - lastTachoR) / 180; //convert tacho count to distance travelled
			lastTachoL=currentTachoL; //update tachometer count
			lastTachoR=currentTachoR; //update tachometer count
			deltaD= 0.5*(distL + distR);
			deltaT = (distL - distR)/LocalizationLab.TRACK; // calculate variation in theta
			deltaT = Math.toDegrees(deltaT);
			dX= deltaD * Math.sin(Math.toRadians(fakeTheta)); // change in x and y according to robot orientation
			dY=deltaD * Math.cos(Math.toRadians(fakeTheta));
			synchronized (lock) {
				/**
				 * Don't use the variables x, y, or theta anywhere but here! Only update the values of x, y,
				 * and theta in this block. Do not perform complex math
				 * 
				 */
				// update values of X,Y,theta
				theta +=deltaT;
				//make sure theta is always between 0 and 360
				if(theta>=360.0){
					theta = theta-360.0;
				}
				else if(theta<0.0){
					theta = theta+360.0;
				}
				fakeTheta = theta;
				x = x+ dX;
				y = y+dY;

			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * @param leftMotorTachoCount the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;
		}
	}

	/**
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * @param rightMotorTachoCount the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;
		}
	}
}

