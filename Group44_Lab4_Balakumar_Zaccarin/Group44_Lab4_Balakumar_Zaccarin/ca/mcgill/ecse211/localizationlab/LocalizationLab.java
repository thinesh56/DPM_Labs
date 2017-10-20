package ca.mcgill.ecse211.localizationlab;

import ca.mcgill.ecse211.localizationlab.Odometer;
import ca.mcgill.ecse211.localizationlab.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;

public class LocalizationLab {

	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3MediumRegulatedMotor usMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S3"));

	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 10.8;
	public static final double TILE_WIDTH = 30.48;
	public static final int FORWARD_SPEED = 100;
	public static final int ROTATE_SPEED = 75;
	public static boolean fallingEdge; //robot is localizing using falling edge detection

	public static void main(String[] args) {
		int buttonChoice;

		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		final UltrasonicLocalization USlocalizer = new UltrasonicLocalization(odometer, leftMotor, rightMotor,usMotor); 
		final LightLocalization lLocalizer = new LightLocalization(odometer, leftMotor, rightMotor);
		OdometryDisplay display = new OdometryDisplay(odometer,t); 
		@SuppressWarnings("resource")
		UltrasonicPoller usPoller = null;							// the selected controller on each cycle
		SampleProvider usDistance = usSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[usDistance.sampleSize()];
		

		do{
			t.clear();

			// ask the user whether the robot should use rising or falling edge detection
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString("Falling| Rising  ", 0, 2);
			t.drawString("edge   | edge ", 0, 3);
			t.drawString("       |       ", 0, 4);

			buttonChoice = Button.waitForAnyPress();

		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		//use falling edge detection
		if (buttonChoice == Button.ID_LEFT) {
			fallingEdge = true;
			usPoller = new UltrasonicPoller(usDistance,usData,USlocalizer);
			odometer.start();
			display.start();
			usPoller.start();
			(new Thread() {
				public void run() {
					USlocalizer.localize();
				}
			}).start();  
			//after Ultrasonic localization wait for button press to start light localization
			Button.waitForAnyPress();
			lLocalizer.localize();
			
		}
		//use rising edge detection
		else if (buttonChoice == Button.ID_RIGHT) {
			fallingEdge = false;
			usPoller = new UltrasonicPoller(usDistance,usData,USlocalizer);
			odometer.start();
			display.start();
			usPoller.start();
			(new Thread() {
				public void run() {
					USlocalizer.localize();
				}
			}).start(); 
			//after Ultrasonic localization wait for button press to start light localization
			Button.waitForAnyPress();
			lLocalizer.localize();
		}


		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

}
