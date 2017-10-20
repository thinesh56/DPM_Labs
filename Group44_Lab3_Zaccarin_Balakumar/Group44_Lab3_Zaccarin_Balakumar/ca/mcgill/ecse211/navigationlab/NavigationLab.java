package ca.mcgill.ecse211.navigationlab;

import ca.mcgill.ecse211.navigationlab.Odometer;
import ca.mcgill.ecse211.navigationlab.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;

public class NavigationLab {

	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	private static final EV3MediumRegulatedMotor usMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

	private static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));

	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 10.8;
	public static final double TILE_WIDTH = 30.48;
	public static final int FORWARD_SPEED = 175;
	public static final int ROTATE_SPEED = 130;
	public static boolean canAvoid; // is the robot going to try and avoid obstacles

	public static void main(String[] args) {
		int buttonChoice;

		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		final Navigation navigator = new Navigation(odometer, leftMotor, rightMotor,usMotor); 
		OdometryDisplay display = new OdometryDisplay(odometer,t); 
		@SuppressWarnings("resource")
		UltrasonicPoller usPoller = null;							// the selected controller on each cycle
		SampleProvider usDistance = usSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[usDistance.sampleSize()];


		do{
			t.clear();

			// ask the user whether the robot should drive to points or avoid obstacles
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString("Drive  | avoid  ", 0, 2);
			t.drawString("to     | obstacles ", 0, 3);
			t.drawString("Points |   ", 0, 4);

			buttonChoice = Button.waitForAnyPress();

		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		//drive without obstacle avoidance
		if (buttonChoice == Button.ID_LEFT) {
			canAvoid = false;
			odometer.start();
			display.start();
			(new Thread() {
				public void run() {
					navigator.navigate();
				}
			}).start();
		}
		//drive with obstacle avoidance
		else if (buttonChoice == Button.ID_RIGHT) {
			canAvoid = true;
			usPoller = new UltrasonicPoller(usDistance,usData,navigator);
			odometer.start();
			display.start();
			usPoller.start();
			(new Thread() {
				public void run() {
					navigator.navigate();
				}
			}).start();  
		}


		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

}
