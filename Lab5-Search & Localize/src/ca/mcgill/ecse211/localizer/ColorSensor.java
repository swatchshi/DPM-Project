package ca.mcgill.ecse211.localizer;

import ca.mcgill.ecse211.lab5.Navigation;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.utility.Delay;

/**
 * Class for handling the light sensor sampling
 * 
 * @author Xavier Pellemans
 * @author Thomas Bahen
 */
public class ColorSensor {
	/**
	 * Variables for sensor reading
	 */
	private EV3ColorSensor lightSensor;
	private static final long PING_PERIOD = 5;
	private float[] baseSample;
	
	
	/**
	 * Creates an instance of a lightSensor
	 * @param lightSensor the EV3ColorSensor used
	 */
	public ColorSensor(EV3ColorSensor lightSensor) {
		this.lightSensor=lightSensor;
		lightSensor.setCurrentMode("Red");
	}

	/**
	 * Checks if a line is crossed, doesn't return until found
	 * @return if a line is crossed
	 */
	public boolean lineCrossed() {
		boolean lineSeen=false;
		long correctionStart, correctionEnd=0; //time elapsed
	    float[] currentSample;
	    
	    float sampleRange1;
	    
	   
	    currentSample = new float[1]; 
	    baseSample = new float[1];  
	   
	    lightSensor.setCurrentMode("Red"); //set Sensor to use red light alone
	    lightSensor.fetchSample(baseSample, 0); //takes a base sample data
	    
	    while(true) {
	    	correctionStart = System.currentTimeMillis();
	    	sampleRange1=baseSample[0]-7*baseSample[0]/10;//prevents repetitive calculation
		    
		    lightSensor.fetchSample(currentSample, 0); //reads the data
		     
		      
		    //reacts to any strange value of the current Sample
		    if(currentSample[0]<sampleRange1){
		       lineSeen=true;
		       Sound.beep();
		       break;
		    }
		
		    // this ensure the odometry correction occurs only once every period
		    correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < PING_PERIOD) {
			    try {
			      Thread.sleep(PING_PERIOD - (correctionEnd - correctionStart));
			    } catch (InterruptedException e) {
			       // there is nothing to be done here
			    }
			}
	    }
	    return lineSeen;   
	}
}
