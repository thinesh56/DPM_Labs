package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int FILTER_OUT = 30;
	
	
	// Added by us
	//private static final int FWDSPEED = 100;
	private static final int MAXCORRECTION=100; //maximum motor speed change allowed
	private static final int ERRORTOL =1; // allowed deviation from WALLDIST
	private static int distError =0;
	private int diff;
	private static final int WALLDIST=45; // wall distance from sensor (at 45deg angle)
	private static final double PROPCONST = 5; // proportionality constant
	//
	
	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;

	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor
															// rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@SuppressWarnings("unused")
	@Override
	public void processUSData(int distance) {
		
		// error calculation, how many cm the robot is from its ideal position
		distError = WALLDIST - distance;

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 70 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 70) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}
			
		// TODO: process a movement based on the us distance passed in (P style)
		
		
		// error within the tolerated deviation from ideal position 
		if(Math.abs(distError)<= ERRORTOL ){
			

			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED+100);
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+100);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();

		}
		
		//robot too close to wall
		else if(distError>0 ){

			diff = calcProp(distError);

			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-diff);
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+diff);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		
		// robot too far from wall
		else if (distError<0 ){
			
			diff =  (calcProp(distError));


					
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED );
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED-diff);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		
		
	}


	// method calculating motor speed adjustment according to error calculated
	public int calcProp(int diff){
		int correction = 0;
		
		if(diff<0){
			//diff = (0-diff);
			diff = Math.abs(diff);
		}
		
		correction = (int)(PROPCONST * (double)diff);
		
		if(correction>= MOTOR_SPEED) {
			correction = MAXCORRECTION;
			
		}
		
		return correction;
			
	}


	
	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
