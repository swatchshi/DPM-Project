package ca.mcgill.ecse211.localizer;

import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Class for handling the light sensor sampling methods
 * Handles light sensor in mode RGB (color) and Red mode (line detecting)
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
	private static final int COLOR_FILTER=4;
	
	/**
	 * Gaussian RGB parameters for red color
	 */
	private final static double RED_R_MEAN=0.12396416;
	private final static double RED_G_MEAN=0.02064270;
	private final static double RED_B_MEAN=0.01372543;
	private final static double RED_R_STD=0.050752937;
	private final static double RED_G_STD=0.007693734;
	private final static double RED_B_STD=0.005220381;

	/**
	 * Gaussian RGB parameters for yellow color
	 */
	private final static double YELLOW_R_MEAN=0.215953659;
	private final static double YELLOW_G_MEAN=0.12932264;
	private final static double YELLOW_B_MEAN=0.022103387;
	private final static double YELLOW_R_STD=0.089106906;
	private final static double YELLOW_G_STD=0.060347026;
	private final static double YELLOW_B_STD=0.009508931;

	/**
	 * Gaussian RGB parameters for white color
	 */
	private final static double WHITE_R_MEAN=0.18946119;
	private final static double WHITE_G_MEAN=0.18917648;
	private final static double WHITE_B_MEAN=0.12824939;
	private final static double WHITE_R_STD=0.095847914;
	private final static double WHITE_G_STD=0.089227586;
	private final static double WHITE_B_STD=0.056484251;

	/**
	 * Gaussian RGB parameters for blue color
	 */
	private final static double BLUE_R_MEAN=0.02340686;
	private final static double BLUE_G_MEAN=0.04534314;
	private final static double BLUE_B_MEAN=0.05202206;
	private final static double BLUE_R_STD=0.018837398;
	private final static double BLUE_G_STD=0.016473312;
	private final static double BLUE_B_STD=0.016708785;
	
	
	/**
	 * Enumeration for the colors possible
	 * BLUE: color blue
	 * RED: color red
	 * YELLOW: color yellow
	 * WHITE: color white
	 * NoColorFoudn: unable to get the RGB representation
	 * 				 of one of the colors above
	 */
	public static enum BlockColor{
		BLUE, RED, YELLOW, WHITE, NoColorFound
	}
	/**
	 * Creates an instance of a lightSensor
	 * @param lightSensor the EV3ColorSensor used
	 */
	public ColorSensor(EV3ColorSensor lightSensor) {
		this.lightSensor=lightSensor;
	}

	/**
	 * Checks if a line is crossed, doesn't return until found
	 * Uses the Red mode of the EV3ColorSensor
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
	
	/**
	 * Checks if a line is detected, doesn't return until found
	 * Uses the colorID mode of the EV3ColorSensor
	 * 
	 * @return if a line is detected
	 */
	public boolean lineDetected() {
		boolean lineSeen=false;
		long correctionStart, correctionEnd=0; //time elapsed
	    
	   
	  
	    lightSensor.setCurrentMode("ColorID"); //set Sensor to use red light alone
	   
	    
	    while(true) {
	    	correctionStart = System.currentTimeMillis();
		    //tries to detect the line with its ID
	    	//The black line of the boards has an ID of 13
		    if(lightSensor.getColorID()==13){
		       lineSeen=true;
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
	
	/**
	 * Calls the calculation method to get the color and returns the filtered color seen by the sensor
	 * @return The BlockColor seen by the ColorSensor
	 */
	public BlockColor getColorSeen() {
		BlockColor color=BlockColor.NoColorFound, newColor;
		int colorCount=0;
		
		while(colorCount<COLOR_FILTER) { //Checks if it is really the seen color
			float[] rgb=getRGB();
			
			newColor= gaussianMehtod(rgb[0], rgb[1], rgb[2]);
					
			if(color==newColor) {
				colorCount++;
			}else {
				colorCount=0;
			}
			color=newColor;
		}
		return color; //the real color as a string
	}
	
	/**
	 * Gaussian method of determining the color seen, takes the r g b values and computes what BlockColor is the closest
	 * @param r the Red component between 0 and 1
	 * @param g the Green component between 0 and 1
	 * @param b the Blue component between 0 and 1
	 * @return the BlockColor associated to these RGB values
	 */
	private BlockColor gaussianMehtod(double r, double g, double b) {
		
		if(Math.abs(r-RED_R_MEAN)<=2*RED_R_STD && Math.abs(g-RED_G_MEAN)<=2*RED_G_STD && Math.abs(b-RED_B_MEAN)<=2*RED_B_STD)
			return BlockColor.RED;
		else if(Math.abs(r-YELLOW_R_MEAN)<=2*YELLOW_R_STD && Math.abs(g-YELLOW_G_MEAN)<=2*YELLOW_G_STD && Math.abs(b-YELLOW_B_MEAN)<=2*YELLOW_B_STD)
			return BlockColor.YELLOW;
		else if(Math.abs(r-BLUE_R_MEAN)<=2*BLUE_R_STD && Math.abs(g-BLUE_G_MEAN)<=2*BLUE_G_STD && Math.abs(b-BLUE_B_MEAN)<=2*BLUE_B_STD)
			return BlockColor.BLUE;
		else if(Math.abs(r-WHITE_R_MEAN)<=2*WHITE_R_STD && Math.abs(g-WHITE_G_MEAN)<=2*WHITE_G_STD && Math.abs(b-WHITE_B_MEAN)<=2*WHITE_B_STD)
			return BlockColor.WHITE;
		else {
			return BlockColor.NoColorFound;
		}
	}
	
	/**
	 * Gets the rgb values of the color sensor
	 * RGB values range between 0 and 1
	 * @return a float array with the R at 0, G at 1 and B at 2
	 */
	public float[] getRGB() {
		lightSensor.setCurrentMode("RGB");
		SampleProvider colorSensor=lightSensor.getRGBMode();
		float[] rgb=new float[colorSensor.sampleSize()];
		colorSensor.fetchSample(rgb, 0);
		
		return rgb;
	}
}
