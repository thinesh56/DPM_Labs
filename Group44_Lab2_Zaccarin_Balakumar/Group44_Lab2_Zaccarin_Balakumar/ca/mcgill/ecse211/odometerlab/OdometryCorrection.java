/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometerlab;

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
//
  private static final Port lsPort = LocalEV3.get().getPort("S1");
  SensorModes lsSensor = new EV3ColorSensor(lsPort);
  SampleProvider lsIntensity = lsSensor.getMode("Red");
  float[] lsData = new float[lsIntensity.sampleSize()];
  private static final double TILE_WIDTH = 30.48;
  // constructor
  public OdometryCorrection(Odometer odometer) {
    this.odometer = odometer;
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart;
	long correctionEnd;
	double currPosition; // current odometer value of x or y when crossing a line
	double multiple; // how many lines robot has crossed 
    int lineNum; // how many lines from origin we are
    int intensity; // intensity of light from colorSensor
    // boolean CrossedY1 = false;
    //boolean CrossedX1= false;
    
    ArrayList<Integer> data = new ArrayList<Integer>() ;
    ArrayList<Integer> filtered = new ArrayList<Integer>();
    boolean isLine = false;
    int nxt=0;
    

    while (true) {
      correctionStart = System.currentTimeMillis();

      lsIntensity.fetchSample(lsData,0);
      
      
      intensity = (int)(lsData[0]*100.0);
      data.add(intensity);
      
      //makes sure filtering does not start before there are enough samples for the median filter
      if(data.size()>4){
      	  nxt = medianFiltre(data);
      	  filtered.add(nxt);
      	  // make sure there are enough filtered samples to calculate derivatives
      	  if(filtered.size()>2){
      		  isLine = derivative(filtered);
      	  }
        }

      //TODO Place correction implementation here
      //what the correction does when crossing a line
      if(isLine == true){
    	  Sound.playNote(Sound.FLUTE,400,100);
    	  //travelling vertically
    	  if((odometer.getTheta()>=355 || odometer.getTheta()<= 5) ||(odometer.getTheta()>= 175 && odometer.getTheta()<=185)){
    		  /*if(crossedY1==false){
    			  odometer.setY(0.0);
    			  crossedY1 = true;
    		  }
    		  else{*/
    		  currPosition = odometer.getY();
    		  multiple = currPosition/TILE_WIDTH;
    		  //determines how many lines from the origin we are
    		  lineNum = (int)(Math.round(multiple));
    		  odometer.setY(lineNum*TILE_WIDTH);
    		 // }
    	  }
    	  //travelling horizontally
    	  else if((odometer.getTheta()>= 85 && odometer.getTheta()<=95)||(odometer.getTheta()>= 265 && odometer.getTheta()<=275)){
    		  /*if(crossedY1==false){
			  	  odometer.setY(0.0);
			  	  crossedY1 = true;
		        }
		        else{*/
    		  currPosition = odometer.getX();
    		  multiple = currPosition/TILE_WIDTH;
    		  //determines how many lines from the origin we are
    		  lineNum = (int)(Math.round(multiple));
    		  odometer.setX(lineNum*TILE_WIDTH);
    		  //}
    	  }
      }
     
      
      
      
      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometry correction will be
          // interrupted by another thread
        }
      }
    }
  }
  
  // helper method to calculate the derivative of light intensity
  public boolean derivative(ArrayList<Integer> signal){
  	  int derivative = 0;
  	  int size = signal.size();
  	  int[] values = new int[2];
  	  int j=0;
  	  for(int i = size-1; i>size-3; i--){  
  		  values[j] = signal.get(i);
  		  j++;
  	  }
  	  derivative = values[1]-values[0];
  	 // System.out.println(derivative);
  	  if((derivative) > 5){
  		  return true;
  	  }
  	  else {return false;}
    }
    // median filter that takes 5 samples as input
    public int medianFiltre(ArrayList<Integer> signal){
  	  int median = 0;
  	  int size = signal.size();
  	  int[] values = new int[5];
  	  boolean continues = true;
  	  int j=0;
  	  for (int i =size-1; i>size-6;i--){
  		  values[j] = signal.get(i);
  		  j++;
  	  }
  	  //sort 5 last values to identify median
  	  while(continues){
  	    int ct =0;
  		  for(int i=0; i<values.length-ct;i++){
  		  continues = false;
  		  int curr = values[i];
  		  int next = values[i+1];
  		  if(curr>next){
  			 values[i]=next;
  			 values[i+1]= curr;
  			 continues = true;
  		  }
  		  ct++;
  	  }
  	  }
  	  //median is in middle position of array once sorted
  	  median = values[2];
  	  return median;
    }
    
}
