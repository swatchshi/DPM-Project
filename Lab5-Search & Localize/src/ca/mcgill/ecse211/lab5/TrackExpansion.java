package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;

public class TrackExpansion {

	// free space between wheels: 13.7 cm
	// wheel width 2.2 (EACH)
	// wheel diameter: 4.4
	public static final double WHEEL_RAD = 2.12;
	public static final double MIN_TRACK = 15.7; // adjust from 13.7 to 18.1
	public static final double MAX_TRACK= 25.4; //21.3 to 29.5

	private static float SCREW_SPEED = 100;
	private static float MAINTENANCE_SPEED = 100;
	private static double SCREW_PITCH = 0.34544;
	private double track;
	private boolean expanded=false;

	/**
	 * Cconstructor of the TrackExpansion
	 * Considers the initial track to be at its minimum when starting
	 */
	public TrackExpansion() {
		this.track=MIN_TRACK;
	}
	
	
	
	/**
	 * Expand the track to the maximum track
	 * 
	 * @return True if the track expanded, false if was already expanded
	 */
	public boolean expandToMax() {
		int initialTachoCount,finalTachoCount;
		if(!expanded) {
			//Expand
			GamePlan.trackExpansionMotor.setSpeed(SCREW_SPEED);
			initialTachoCount=GamePlan.trackExpansionMotor.getTachoCount();
			GamePlan.trackExpansionMotor.rotate(convertExpansion(MAX_TRACK-track)); //Rotate to expand
			finalTachoCount=GamePlan.trackExpansionMotor.getTachoCount();
			
			track+=convertRotation(Math.abs(finalTachoCount-initialTachoCount)); //calculate the real track
			expanded=true;
			return true;
		}else {
			return false;
		}
	}
	
	/**
	 * Retract the track to the minimum track value
	 * 
	 * @return True if the track retracted, false if was already retracted
	 */
	public boolean retractToMin() {
		int initialTachoCount,finalTachoCount;
		if(expanded) {
			//Retrack
			GamePlan.trackExpansionMotor.setSpeed(SCREW_SPEED);
			initialTachoCount=GamePlan.trackExpansionMotor.getTachoCount();
			GamePlan.trackExpansionMotor.rotate(-convertExpansion(track-MIN_TRACK)); //Rotate to expand
			finalTachoCount=GamePlan.trackExpansionMotor.getTachoCount();
			
			track-=convertRotation(Math.abs(finalTachoCount-initialTachoCount)); //calculate the real track
			expanded=false;
			return true;
		}else {
			return false;
		}
	}
	
	/**
	 * Procedure to adjust the track to its minimum manually
	 * The track will start to slowly retract
	 * Waits for the user to press any button when the minimum track has been reached
	 * Cancel option included
	 */
	public void adjustToMin(TextLCD lcd) {
		int buttonID;
		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("Screw Maintenance", 0, 0);
			lcd.drawString(" Press enter to ", 0, 1);
			lcd.drawString(" start min set  ", 0, 2);
			lcd.drawString(" Press back     ", 0, 3);
			lcd.drawString(" to skip min set", 0, 4);
			buttonID=Button.waitForAnyPress();

		}while(buttonID==Button.ID_ENTER || buttonID==Button.ID_ESCAPE);
		
		if(buttonID==Button.ID_ENTER) {
			GamePlan.trackExpansionMotor.setSpeed(MAINTENANCE_SPEED);
			lcd.clear();
			lcd.drawString("Adjust to Min Track", 0, 0);
			lcd.drawString("Press any button   ", 0, 1);
			lcd.drawString(" when reached      ", 0, 2);
			GamePlan.trackExpansionMotor.backward();
			Button.waitForAnyPress();
			GamePlan.trackExpansionMotor.stop();
			lcd.clear();
			track=MIN_TRACK;
		}
	}
	
	/**
	 * Procedure to adjust the track to its maximum manually
	 * The track will start to slowly expand
	 * Waits for the user to press any button when the maximum track has been reached
	 * Cancel option included
	 */
	public void adjustToMax(TextLCD lcd) {
		int buttonID;
		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("Screw Maintenance", 0, 0);
			lcd.drawString(" Press enter to ", 0, 1);
			lcd.drawString(" start max set  ", 0, 2);
			lcd.drawString(" Press back     ", 0, 3);
			lcd.drawString(" to skip max set", 0, 4);
			buttonID=Button.waitForAnyPress();

		}while(buttonID==Button.ID_ENTER || buttonID==Button.ID_ESCAPE);
		
		if(buttonID==Button.ID_ENTER) {
			GamePlan.trackExpansionMotor.setSpeed(MAINTENANCE_SPEED);
			lcd.clear();
			lcd.drawString("Adjust to Max Track", 0, 0);
			lcd.drawString("Press any button   ", 0, 1);
			lcd.drawString(" when reached      ", 0, 2);
			GamePlan.trackExpansionMotor.forward();
			Button.waitForAnyPress();
			GamePlan.trackExpansionMotor.stop();
			lcd.clear();
			track=MIN_TRACK;
		}
	}
	
	
	/**
	 * Gets if the track is expanded
	 * 
	 * @return True if the track is at its maximum
	 */
	public boolean isExpanded() {
		return expanded;
	}
	
	/**
	 * Gets the current track of the robot
	 * @return The size of the track (in cm)
	 */
	public double getTrack() {
		return track;
	}
	
	
	/**
	 * Converts the track expansion displacement into rotation in degrees
	 * 
	 * @param distance The displacement needed to expand the track
	 * @return The rotation needed to expand the tracks (in degrees)
	 */
	private int convertExpansion(double distance) {
		return (int) (360 * distance/SCREW_PITCH);
	}
	
	/**
	 * Converts rotation in degrees into the track expansion displacement
	 * 
	 * @param The rotation of the screw (in degrees)
	 * @return distance The displacement of the track
	 */
	private double convertRotation(int rotation) {
		return (double) (SCREW_PITCH* rotation /360);
	}
}
