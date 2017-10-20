package ca.mcgill.ecse211.localizationlab;

import java.util.ArrayList;

import ca.mcgill.ecse211.localizationlab.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalization extends Thread{
	

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final Port lsPort = LocalEV3.get().getPort("S2");
	SensorModes lsSensor = new EV3ColorSensor(lsPort);
	SampleProvider lsIntensity = lsSensor.getMode("Red");
	float[] lsData = new float[lsIntensity.sampleSize()];
	
	private static final double TILE_WIDTH = 30.48;
	
	private double xStart; //value of X after light localization
	private double yStart; //value of Y after light localization
	private final double offset = 17; // distance of light sensor from center of rotation
	
	private boolean isLine = false;
	private double[] lines = new double[4]; 
	private int lineCounter =0;
	int intensity;
	int threshold = 20; //threshold for seeing a black line
	double deltaT ;
	double deltaY;
	double deltaYMinus;
	double newTheta; // value to add to heading after light localization

	public LightLocalization(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.leftMotor.setAcceleration(100);
		this.rightMotor.setAcceleration(100);
	}

	public void localize(){

		// move robot closer to origin, in order to be able to do the light localization
		turnTo(45.0);
		leftMotor.setSpeed(LocalizationLab.FORWARD_SPEED); //start motors
		rightMotor.setSpeed(LocalizationLab.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(LocalizationLab.WHEEL_RADIUS, 5), true);
		rightMotor.rotate(convertDistance(LocalizationLab.WHEEL_RADIUS, 5), false);


		//turn counter clockwise 330°, all 4 lines will have been crossed
		leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, -330), true);
		rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, -330), true);
		// store the heading of the robot when it crosses each line
		while (lineCounter != 4){
			lsIntensity.fetchSample(lsData,0);      
			intensity = (int)(lsData[0]*100.0);
			if(intensity<threshold){
				Sound.beep();
				lines[lineCounter] = odometer.getTheta();
				lineCounter ++;
			}
		}

		// calculate distance of robot from origin using grid line headings
		xStart= -offset*(Math.cos(Math.toRadians((lines[2]-lines[0])*0.5))); // X value of robot position
		odometer.setX(xStart);
		yStart= -offset*(Math.cos(Math.toRadians((lines[3]-lines[1])*0.5))); // Y value of robot position
		odometer.setY(yStart);

		// correct heading of robot
		deltaY= (lines[2]-lines[0]);
		deltaYMinus= ((lines[0])-180);
		deltaT= (90 - deltaYMinus + deltaY/2);

		System.out.println("");
		System.out.println("");
		System.out.println("");
		System.out.println(odometer.getTheta());
		System.out.println(deltaT);
		newTheta = odometer.getTheta()+deltaT;

		//adjust theta using line detection
		odometer.setTheta(odometer.getTheta()+newTheta); 

		// move to origin and orient to 0° direction
		moveTo(0.0,0.0);
		turnTo(0.0);
	}

	// turn robot to specific orientation
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
		leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, deltaT), true);
		rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, deltaT), false);
	}

	//move robot to given position
	void moveTo(double x, double y){
		double deltaX = x-odometer.getX();
		double deltaY = y-odometer.getY();
		double deltaD = Math.sqrt((deltaX*deltaX)+(deltaY*deltaY)); // find distance needing to be traveled

		double Theta = Math.toDegrees(Math.atan2(deltaX,deltaY)); //find angle that robot needs to turn to
		turnTo(Theta); // turn robot
		leftMotor.setSpeed(LocalizationLab.FORWARD_SPEED); //start motors
		rightMotor.setSpeed(LocalizationLab.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(LocalizationLab.WHEEL_RADIUS, deltaD), true);
		rightMotor.rotate(convertDistance(LocalizationLab.WHEEL_RADIUS, deltaD), false);			
	}




	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
}
