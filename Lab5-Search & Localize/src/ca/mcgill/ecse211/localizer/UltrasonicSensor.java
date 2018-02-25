package ca.mcgill.ecse211.localizer;

import ca.mcgill.ecse211.lab5.*;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * Class for handling the ultrasonic sensor sampling
 * 
 * @author Xavier Pellemans
 * @author Thomas Bahen
 */
public class UltrasonicSensor {
	/**
	 * Variables
	 */
	private SampleProvider us;
	private float[] usData;
	private static final int US_FILTER=2;
	private static final int US_ERROR=5;
	private int distance=0;


	/**
	 * Constructor of the sampling object
	 * @param us SampleProvider used
	 * @param usData float[] to hold the us samples
	 */
	public UltrasonicSensor(SampleProvider us, float[] usData) {
	    this.us = us;
	    this.usData = usData;
	}
	  
	
	/**
	 * Gets the distance seen by the US
	 * @return the normalized distance seen (as an integer)
	 */
	public  int readDistance() {
		int newDistance=0;
		int filterCount = 0;
		while(filterCount<US_FILTER) {
			us.fetchSample(usData, 0); // acquire data
			newDistance=(int) (usData[0] * 100.0);// extract from buffer, cast to int and add to total
			
			if(newDistance<=distance+US_ERROR && newDistance>=distance-US_ERROR) {
				filterCount++;
			}else {
				filterCount=0;
			}
			distance=newDistance;
			Delay.msDelay(30);
		}
		return distance; 
	}
	
	
	
	/**
	 * Gets the distance previously recorded by the ultrasonic sensor
	 * @return the distance in cm (as an int)
	 */
	public int getDistance() {
		return distance;
	}
}
