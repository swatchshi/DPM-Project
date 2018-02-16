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
	private int distance;
	private static final int SAMPLE_SIZE=1;


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
	public  int getDistance() {
		int total=0;
		for(int i=0;i<SAMPLE_SIZE;i++) {
			us.fetchSample(usData, 0); // acquire data
			total+=(int) (usData[0] * 100.0);// extract from buffer, cast to int and add to total
			Delay.msDelay(30);
		}
		
	    distance = (int) (total/SAMPLE_SIZE); //normalize
		return distance; 
	}
}
