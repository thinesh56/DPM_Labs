package ca.mcgill.ecse211.localizationlab;

import ca.mcgill.ecse211.localizationlab.Odometer;
import ca.mcgill.ecse211.localizationlab.LocalizationLab;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;

public class UltrasonicLocalization extends Thread implements UltrasonicController{

	
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor usMotor;
	
	private int distance;
	private double deltaTheta;
	private double alpha;
	private double beta;
	double d = 40;
	
	public UltrasonicLocalization(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3MediumRegulatedMotor usMotor ){
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usMotor = usMotor;
		this.leftMotor.setAcceleration(100);
		this.rightMotor.setAcceleration(100);
	}
	
	public void localize(){
		//falling edge detection
		if(LocalizationLab.fallingEdge == true){
			//rotate clockwise until we find value of alpha
			while (distance > d) {
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            leftMotor.forward();
	            rightMotor.backward();
	        }
			alpha = odometer.getTheta();
			Sound.beep();
				
			//rotate counterclockwise for 40° to avoid detecting same wall twice
			while(Math.abs(odometer.getTheta()-alpha)<40){
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            leftMotor.backward();
	            rightMotor.forward();
			}
			//rotate counterclockwise until we find value of beta
			while(distance > d){
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		        rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		        leftMotor.backward();
		        rightMotor.forward();
			}
			beta = odometer.getTheta();
			Sound.beep();
			//calculate value to be added to robot heading
			deltaTheta = calculateTheta(alpha,beta);
			
			//set heading and turn to face 0° 
			odometer.setTheta(odometer.getTheta()+deltaTheta);
			turnTo(0.0);
			
		}
		
		//rising edge detection
		else if(LocalizationLab.fallingEdge == false){
			//rotate clockwise until we find value of alpha
			while (distance < d) {
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            leftMotor.forward();
	            rightMotor.backward();
	        }
			alpha = odometer.getTheta();
			Sound.beep();
				
			//rotate counterclockwise for 40° to avoid detecting same wall twice
			while(Math.abs(odometer.getTheta()-alpha)<40){
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            leftMotor.backward();
	            rightMotor.forward();
			}
			//rotate counterclockwise until we find value of beta
			while(distance < d){
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		        rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		        leftMotor.backward();
		        rightMotor.forward();
			}
			beta = odometer.getTheta();
			Sound.beep();
			//calculate value to be added to robot heading
			deltaTheta = calculateTheta(alpha,beta);
			
			//set heading and turn to face 0° 
			odometer.setTheta(odometer.getTheta()+deltaTheta);
			turnTo(0.0);
			leftMotor.stop();
			rightMotor.stop();
			
			
		}
	}
	
	//method calculating the value to be added to heading to correct robot orientation
	public double calculateTheta (double angleA, double angleB){
		if(LocalizationLab.fallingEdge == false){
			deltaTheta = 37 - (0.5)*(angleA+angleB);
		}
		else{
			deltaTheta = 220 - (0.5)*(angleA+angleB);
		}
		return deltaTheta;
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		
	}

	@Override
	public int readUSDistance() {
		return this.distance;
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
	
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	
}
