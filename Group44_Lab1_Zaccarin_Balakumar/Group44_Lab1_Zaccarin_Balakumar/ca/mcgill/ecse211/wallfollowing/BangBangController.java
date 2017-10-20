package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	private static final int FILTER_OUT = 30; // added for filter
	private int filterControl; // added for filter


	
	
 BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving
														// forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		//this.distance = distance;
		// TODO: process a movement based on the us distance passed in
		// (BANG-BANG style)
			
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		
		// robot within acceptable distance from wall
		 if (distance < ((30) *2/ Math.sqrt(2) )&& distance > (25 *2/ Math.sqrt(2))) {
			
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
			
			
		} else if (distance >= (30 *2/ Math.sqrt(2))) { //robot too far from wall
		
	
			
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		
			
			
		} else if (distance <= (25 *2/ Math.sqrt(2))) { // robot too close to wall
		
		

			if(distance <= (12.5 *2/ Math.sqrt(2))){ // robot very close to wall
				
				if(distance <= (10 *2/ Math.sqrt(2))){ // robot extremely close to wall, back up
			
					  WallFollowingLab.leftMotor.setSpeed(motorHigh+250);
					  WallFollowingLab.rightMotor.setSpeed(motorHigh+100);
					  WallFollowingLab.leftMotor.backward();
					  WallFollowingLab.rightMotor.backward();
					  
					  //get more distance from wall
					  WallFollowingLab.leftMotor.setSpeed(motorHigh+250);
					  WallFollowingLab.rightMotor.setSpeed(motorHigh+100);
					  WallFollowingLab.leftMotor.backward();
					  WallFollowingLab.rightMotor.backward();
				} 
				else {
					
				//occurs between 12.5 - 10 cm from wall
		
			  WallFollowingLab.leftMotor.setSpeed(motorHigh+100);
			  WallFollowingLab.rightMotor.setSpeed(motorHigh);
			  WallFollowingLab.leftMotor.backward();
			  WallFollowingLab.rightMotor.forward();
				}
			  
			}
			else{
				//occurs between 25-12.5 cm from wall
	
			 
			
			WallFollowingLab.leftMotor.setSpeed(motorLow);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
			
		} 
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}

