package ca.mcgill.ecse211.localizer;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

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
	public static final int US_ERROR=2;
	private int distance=0;


	/**
	 * Constructor of the sampling object
	 * 
	 * @param ultraSSensor EV3UltraSonicSensor used
	 */
	public UltrasonicSensor(EV3UltrasonicSensor ultraSSensor) {
		us = ultraSSensor.getMode("Distance"); // usDistance provides samples from
																// this instance
		usData = new float[us.sampleSize()]; // usData is the buffer in which data are
	}
	  
	
	/**
	 * Gets the distance seen by the US
	 * @return the normalized distance seen (as an integer)
	 */
	public int readDistance() {
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
		}
		return distance; 
	}
	
	/**
	 * Gets the raw distance seen by the US
	 * @return the normalized distance seen (as an integer)
	 */
	public  int rawDistance() {
		us.fetchSample(usData, 0); // acquire data
		distance=(int) (usData[0] * 100.0);// extract from buffer, cast to int and add to total
			
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
