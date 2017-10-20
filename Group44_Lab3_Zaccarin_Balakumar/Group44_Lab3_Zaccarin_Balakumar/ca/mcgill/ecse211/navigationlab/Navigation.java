package ca.mcgill.ecse211.navigationlab;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicBoolean;

import ca.mcgill.ecse211.navigationlab.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Navigation extends Thread implements UltrasonicController{


	private Odometer odometer;
	private boolean isNavigating;
	private int distance; // distance returned by ultrasonic sensor;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor usMotor; //motor to change ultrasonic sensor's direction
	Queue<Double> positionX = new LinkedList<Double>(); //X values of way points
	Queue<Double> positionY = new LinkedList<Double>(); //Y values of way points
	private double [] location = new double[]{1.0,0.0,2.0,1.0,2.0,2.0,0.0,2.0,1.0,1.0}; // list of waypoints
	double finalX; // current x waypoint value in cm
	double finalY; // current y waypoint value in cm
	AtomicBoolean wallFollower = new AtomicBoolean(false); // obstacle avoidance flag
	AtomicBoolean doneFollowing = new AtomicBoolean(false); // end of obstacle avoidance flag
	private double thetaObstacle; // value of odometer theta when beginning obstacle avoidance
	int threshold = 15; //distance of obstacle from US sensor when wall follower is flagged
	
	//obstacle avoidance specific variables
	int bandCenter = 17;
	private static final int MAXCORRECTION=80; // max correction
	private static final double PROPCONST = 5; // proportionality constant
	private static int distError =0;
	private int diff; //motor speed variation
	private static final int ERRORTOL =1;

	
    //constructor
	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3MediumRegulatedMotor usMotor ){
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usMotor = usMotor;
	}



	public void navigate(){
		//fill queue with array
		for(int i=0; i<location.length; i=i+2){
			positionX.add(location[i]);
			positionY.add(location[i+1]);
		}
		//go to waypoints until queue is empty
		while(positionX.isEmpty()== false && positionY.isEmpty() == false){
			//only update if not navigating --> made it to waypoint or finished avoiding obstacle
			if(isNavigating == false){
				TravelTo(positionX.peek(),positionY.peek());
				//while the robot is travelling to destination
				while(atDestination() == false){
					//if avoid obstacles not enable --> wait
					//if avoiding obstacles --> look for and avoid obstacles
					if(NavigationLab.canAvoid == true){
						// when an obstacle is found, wallFollower is true
						if(wallFollower.get() == true){
							//turn to set  US sensor and robot position for avoiding obstacle
							usMotor.rotateTo(60); 
							isNavigating = false;
							leftMotor.setSpeed(NavigationLab.ROTATE_SPEED);
							rightMotor.setSpeed(NavigationLab.ROTATE_SPEED);
							leftMotor.rotate(convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, 90), true);
							rightMotor.rotate(-convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, 90), false);	
							thetaObstacle = odometer.getTheta();
							wallFollower.set(true); 
							break;
						}
					}
				}
				//avoid obstacle until done avoiding obstacle
				while(wallFollower.get()==true){
					avoid();
					//finished avoiding obstacle
					if(doneFollowing.get()==true ){
						wallFollower.set(false);
						doneFollowing.set(false);
					}

				}
				//if at destination, remove way point from queues
				if(atDestination() == true){
					Sound.beep();
					isNavigating = false;
					positionX.remove();
					positionY.remove();
				}
			}
		}
	}

	// basic method that determines angle to rotate and distance to travel and sets motors to travel that distance
	void TravelTo(double x, double y){
		isNavigating = true; // we are navigating
		finalX = x*NavigationLab.TILE_WIDTH; //point coordinates
		finalY = y*NavigationLab.TILE_WIDTH; //point coordinates
		double deltaX = finalX-odometer.getX();
		double deltaY = finalY-odometer.getY();
		double deltaD = Math.sqrt((deltaX*deltaX)+(deltaY*deltaY)); // find distance needing to be traveled
		double Theta = Math.toDegrees(Math.atan2(deltaY,deltaX)); //find angle that robot needs to turn to
		turnTo(Theta); // turn robot
		leftMotor.setSpeed(NavigationLab.FORWARD_SPEED); //start motors
		rightMotor.setSpeed(NavigationLab.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, deltaD), true);
		rightMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, deltaD), true);
	}

// method called to avoid obstacles
	void avoid(){
		// only follow wall until the value of theta has changed by 125° and makes sure it works with theta wrapping
		// from 359° to 0°
		if(wallFollower.get() == true && Math.abs(thetaObstacle-odometer.getTheta())<125 || Math.abs(thetaObstacle-odometer.getTheta())>=235){
			distError = bandCenter-distance;
			//robot acceptable distance from wall
			if(Math.abs(distError)<= ERRORTOL ){
				rightMotor.setSpeed(NavigationLab.FORWARD_SPEED);
				leftMotor.setSpeed(NavigationLab.FORWARD_SPEED);
				leftMotor.forward();
				rightMotor.forward();
			}

			//robot too close to wall
			else if(distError>0 ){
				diff = calcProp(distError);

				rightMotor.setSpeed(NavigationLab.FORWARD_SPEED-diff);
				leftMotor.setSpeed(NavigationLab.FORWARD_SPEED+diff);
				leftMotor.forward();
				rightMotor.forward();
			}

			// robot too far from wall
			else if (distError<0 ){
				diff =  (calcProp(distError));

				rightMotor.setSpeed(NavigationLab.FORWARD_SPEED );
				leftMotor.setSpeed(NavigationLab.FORWARD_SPEED-diff);
				leftMotor.forward();
				rightMotor.forward();
			}
		}
		//once we have been avoiding obstacle for 125°
		//trigger transition back to navigating
		else if(wallFollower.get() == true && Math.abs(thetaObstacle-odometer.getTheta())>=125&&Math.abs(thetaObstacle-odometer.getTheta())<=235){
			usMotor.rotateTo(0); // US sensor looks forward again
			doneFollowing.set(true);
		}
	}

	// determines smallest angle to turn to be in theta orientation
	//rotate to theta orientation
	void turnTo(double theta){
		double currT = odometer.getTheta();
		double deltaT = theta-currT;
		//makes sure smallest angle
		if(deltaT>180){
			deltaT=deltaT-360;
		}
		else if(deltaT <-180){
			deltaT= deltaT+360;
		}
		// set motors here 
		leftMotor.setSpeed(NavigationLab.ROTATE_SPEED);
		rightMotor.setSpeed(NavigationLab.ROTATE_SPEED);
		leftMotor.rotate(convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, deltaT), true);
		rightMotor.rotate(-convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, deltaT), false);

	}

	public void processUSData(int distance){
		this.distance = distance;
		if(distance<threshold){
			wallFollower.set(true);  // flag obstacle avoidance if close to obstacle
		}

	}

	boolean isNavigating(){
		return isNavigating;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	//checks to see if we are at targeted waypoint
	public boolean atDestination(){
		double errorX = Math.abs(finalX-odometer.getX());
		double errorY = Math.abs(finalY-odometer.getY());
		if(errorX<1 && errorY<1){
			return true;
		}
		return false;
	}



	@Override
	public int readUSDistance() {
		return this.distance;
	}

	//correction calculator for obstacle avoidance
	public int calcProp(int difference){
		int correction = 0;
		if(difference<0){
			difference = Math.abs(difference);
		}
		correction = (int)(PROPCONST * (double)difference);
		if(correction>= NavigationLab.FORWARD_SPEED) {
			correction = MAXCORRECTION;

		}
		return correction;

	}
}
