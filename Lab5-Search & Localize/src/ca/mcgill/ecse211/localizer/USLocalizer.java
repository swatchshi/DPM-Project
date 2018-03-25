package ca.mcgill.ecse211.localizer;

import java.util.ArrayList;
import java.util.Stack;

import ca.mcgill.ecse211.lab5.*;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;

/**
 * Class for handling the ultrasonic localization procedure
 * 
 * @author Xavier Pellemans
 * @author Thomas Bahen
 */
public class USLocalizer{
	
	/**
	 * Variables for localization
	 */
	
	public static enum LocalizerType{
		RISING_EDGE, 
		FALLING_EDGE
	}
	private static final int WALL_THRESHOLD = 45;
	private static final int NO_WALL_FILTER = 3;
	private static final int WALL_FILTER = 2;
	private final Odometer odo;
	private final Navigation navigation;
	private final UltrasonicSensor usSensor;
	private LocalizerType loc;
	private boolean usLocalizerDone;
	private boolean doneTurning;
	private int filterControl;
	private int corner;
	
	
	/**
	 * Constructor of the object for the us localization procedure
	 * @param odo Odometer used
	 * @param navigation Navigation used
	 * @param us UltrasonicSensor used
	 * @param loc LocalizerType used
	 */
	public USLocalizer(Odometer odo, Navigation navigation, UltrasonicSensor us) {
		this.odo = odo;
		this.navigation = navigation;
		this.usSensor = us;
		filterControl=0;
		this.usLocalizerDone=true;
	}
	
	/**
	 * Computes the absolute difference in angle between two angles in radians
	 * 
	 * @param angle1 first angle in rad
	 * @param angle2 second angle in rad
	 * @return
	 */
	public static double getDiffAngle(double angle1, double angle2) {
		//acos(cos(a1-a2))=acos(cos(a1)*cos(a2)+sin(a1)*sin(a2))--trig identity
		return Math.abs(Math.acos(Math.cos(angle1) * Math.cos(angle2) + Math.sin(angle1) * Math.sin(angle2)));
	}
	
	
	/**
	 * Localization method for both falling edge and falling edge
	 * Considers it is in the corner 0
	 */
	public void doLocalization() {
		try {
			doLocalization(0);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} //do localization like it's in the corner 0
	}
	
	/**
	 * Localization method for both falling edge and falling edge
	 * Considers it is in the corner specified
	 * @param corner (int) the corner where the localization takes place
	 */
	public void doLocalization(int corner) throws Exception{
		
		usLocalizerDone=false;
		determineLocType();
		try {
		if (loc == LocalizerType.FALLING_EDGE) {
			
			turnToNoWall(Navigation.Turn.CLOCK_WISE);
			turnToWall(Navigation.Turn.CLOCK_WISE);
			double angle1 = odo.getTheta(); //Record back wall angle
			
			turnToNoWall(Navigation.Turn.COUNTER_CLOCK_WISE);
			turnToWall(Navigation.Turn.COUNTER_CLOCK_WISE);
			double angle2 = odo.getTheta(); //Record left wall angle
			
			double angle = Math.toDegrees(getDiffAngle(Math.toRadians(angle1), Math.toRadians(angle2))) / 2 + angle1;
			
			odo.setTheta(odo.getTheta() +225 - angle); //reset angle
			
		} else if (loc == LocalizerType.RISING_EDGE) {
			turnToWall(Navigation.Turn.CLOCK_WISE);
			turnToNoWall(Navigation.Turn.COUNTER_CLOCK_WISE);
			double angle1 = odo.getTheta();	//Back wall angle
			
			turnToWall(Navigation.Turn.CLOCK_WISE);
			turnToNoWall(Navigation.Turn.CLOCK_WISE);
			double angle2 = odo.getTheta(); //Left wall angle
			
			double angle = Math.toDegrees(getDiffAngle(Math.toRadians(angle1), Math.toRadians(angle2))) / 2 + angle1;
			
			odo.setTheta(odo.getTheta() +225 - angle); //reset angle
		}
		
		odo.setTheta(odo.getTheta()-90*corner);
		//Finally turn to absolute 0 degrees
		navigation.turnTo(0);
		usLocalizerDone=true;
		}
		catch(Exception e) {
			 System.err.println("Error: main " + e.getMessage());
			 Button.waitForAnyPress();
		}
	}
	
	
	/**
	 * Turns until the ultrasonic sensor reads distances less or equal than the WALL_THRESHOLD
	 * 
	 * @param direction Turn clockwise of counter-clockwise
	 */
	private void turnToWall(Navigation.Turn direction) {
		navigation.rotate(direction);
		doneTurning=false;
		
		while(!doneTurning) {
			if(usSensor.rawDistance()<WALL_THRESHOLD) { 
				filterControl++;
				if(filterControl>=WALL_FILTER) {
					doneTurning=true;
					navigation.stopMotors();
				}
			}
			else {
				filterControl=0;
			}
		}
	}

	/**
	 * Turns until the ultrasonic sensor reads distances greater or equal than the WALL_THRESHOLD
	 * 
	 * @param direction Turn clockwise of counter-clockwise
	 */
	private void turnToNoWall(Navigation.Turn direction) {
		navigation.rotate(direction);
		doneTurning=false;
		
		while(!doneTurning) {
			if(usSensor.rawDistance()>WALL_THRESHOLD) {
			  filterControl++;
				if(filterControl>=NO_WALL_FILTER) {
					filterControl=0;
					doneTurning=true;
					navigation.stopMotors();
				}
			}
			else {
				filterControl=0;
			}
		}
	}

	/**
	 * Gets if the USLocalizer has finished
	 * @return if the localization by the us is done
	 */
	public boolean isFinished() {
		return usLocalizerDone;
	}
	
	
	/**
	 * Sets the localization type if not in a localization procedure
	 * @param loc the LocalizerType desired
	 */
	public void setLocalizerType(LocalizerType loc) {
		if(!usLocalizerDone)
			this.loc=loc;
	}
	
	/**
	 * Determines the localizer type based on the distance seen
	 */
	public void determineLocType() {
		if(usSensor.rawDistance()<WALL_THRESHOLD) { //Deciding what localizer should be used
			this.loc = LocalizerType.RISING_EDGE;
		}else {
			this.loc=LocalizerType.FALLING_EDGE;
		}
	}
}
