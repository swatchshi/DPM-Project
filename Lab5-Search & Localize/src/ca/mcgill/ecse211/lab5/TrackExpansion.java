package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * Class responsible for expanding the track and providing methods to regulate
 * the width of the robot.
 * 
 * @author Xavier Pellemans
 *
 */
public class TrackExpansion {

	// free space between wheels: 13.7 cm
	// wheel width 2.2 (EACH)
	// wheel diameter: 4.4
	public static final double SCREW_WHEEL_RAD = 2.4;
	public static final double TANK_WHEEL_RAD = 1.655;
	public static final double TANK_TRACK = 19.138; // tested value or 19.21
	public static final double MIN_SCREW_TRACK = 16.3; // adjust from 13.7 to 18.1
	public static final double MAX_SCREW_TRACK = 16.3; // 21.3 to 29.5
	public static final double LIGHT_SENSOR_DISTANCE_SCREW = 6;
	public static final double LIGHT_SENSOR_DISTANCE_TANK = -8; // -7.8 to -8.8 sensor at the back
  
    
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static float SCREW_SPEED = 600;
	private static float MAINTENANCE_SPEED = 400;
	private static double SCREW_PITCH = 0.34544;
	private double track;
	private double wheelRad;
	private double lightSensorDistance;
	private boolean expanded = false;
	private boolean expandable;
	private EV3MediumRegulatedMotor trackExpansionMotor;

	/**
	 * Constructor of the TrackExpansion Considers default values to be of the screw
	 * design Considers the initial track to be at its minimum when starting
	 */
	public TrackExpansion() {
		this.track = MIN_SCREW_TRACK;
		this.wheelRad = SCREW_WHEEL_RAD;
		this.lightSensorDistance = LIGHT_SENSOR_DISTANCE_SCREW;
	}

	/**
	 * Expand the track to the maximum track
	 * 
	 * @return True if the track expanded, false if was already expanded or not
	 *         expandable
	 */
	public boolean expandToMax() {
		int initialTachoCount, finalTachoCount;
		try {
			if (expandable) {
				if (!expanded) {
					// Expand
					trackExpansionMotor.setSpeed(SCREW_SPEED);
					initialTachoCount = trackExpansionMotor.getTachoCount();
					trackExpansionMotor.rotate(convertExpansion(MAX_SCREW_TRACK - track)); // Rotate to expand
					finalTachoCount = trackExpansionMotor.getTachoCount();

					track += convertRotation(Math.abs(finalTachoCount - initialTachoCount)); // calculate the real track
					expanded = true;
					return true;
				}
			}
		} catch (NullPointerException e) {
			// motor not defined
		}
		return false;
	}

	/**
	 * Retract the track to the minimum track value
	 * 
	 * @return True if the track retracted, false if was already retracted or not
	 *         expandable
	 */
	public boolean retractToMin() {
		int initialTachoCount, finalTachoCount;
		try {
			if (expandable) {
				if (expanded) {
					// Retrack
					trackExpansionMotor.setSpeed(SCREW_SPEED);
					initialTachoCount = trackExpansionMotor.getTachoCount();
					trackExpansionMotor.rotate(-convertExpansion(track - MIN_SCREW_TRACK)); // Rotate to expand
					finalTachoCount = trackExpansionMotor.getTachoCount();

					track -= convertRotation(Math.abs(finalTachoCount - initialTachoCount)); // calculate the real track
					expanded = false;
					return true;
				}
			}
		} catch (NullPointerException e) {
			// motor not defined
		}

		return false;
	}

	/**
	 * Procedure to adjust the track to its minimum manually The track will start to
	 * slowly retract Waits for the user to press any button when the minimum track
	 * has been reached Cancel option included
	 */
	public void adjustToMin() {
		int buttonID;
		if (expandable) {
			do {
				// clear the display
				lcd.clear();

				// ask the user whether the motors should drive in a square or float
				lcd.drawString("Screw Maintenance", 0, 0);
				lcd.drawString(" Press enter to ", 0, 1);
				lcd.drawString(" start min set  ", 0, 2);
				lcd.drawString(" Press back     ", 0, 3);
				lcd.drawString(" to skip min set", 0, 4);
				buttonID = Button.waitForAnyPress();

			} while (buttonID != Button.ID_ENTER && buttonID != Button.ID_ESCAPE);

			if (buttonID == Button.ID_ENTER) {
				trackExpansionMotor.setSpeed(MAINTENANCE_SPEED);
				lcd.clear();
				lcd.drawString("Adjust to Min Track", 0, 0);
				lcd.drawString("Press any button   ", 0, 1);
				lcd.drawString(" when reached      ", 0, 2);
				trackExpansionMotor.backward();
				Button.waitForAnyPress();
				trackExpansionMotor.stop();
				lcd.clear();
				track = MIN_SCREW_TRACK;
			}
		} else {
			// if not expandable
			lcd.clear();
			lcd.drawString("Track not adjustable", 0, 0);
			lcd.drawString("Press any button   ", 0, 1);
			Button.waitForAnyPress();
		}
	}

	/**
	 * Procedure to adjust the track to its maximum manually The track will start to
	 * slowly expand Waits for the user to press any button when the maximum track
	 * has been reached Cancel option included
	 */
	public void adjustToMax() {
		int buttonID;
		if (expandable) {
			do {
				// clear the display
				lcd.clear();

				// ask the user whether the motors should drive in a square or float
				lcd.drawString("Screw Maintenance", 0, 0);
				lcd.drawString(" Press enter to ", 0, 1);
				lcd.drawString(" start max set  ", 0, 2);
				lcd.drawString(" Press back     ", 0, 3);
				lcd.drawString(" to skip max set", 0, 4);
				buttonID = Button.waitForAnyPress();

			} while (buttonID != Button.ID_ENTER && buttonID != Button.ID_ESCAPE);

			if (buttonID == Button.ID_ENTER) {
				trackExpansionMotor.setSpeed(MAINTENANCE_SPEED);
				lcd.clear();
				lcd.drawString("Adjust to Max Track", 0, 0);
				lcd.drawString("Press any button   ", 0, 1);
				lcd.drawString(" when reached      ", 0, 2);
				trackExpansionMotor.forward();
				Button.waitForAnyPress();
				trackExpansionMotor.stop();
				lcd.clear();
				track = MIN_SCREW_TRACK;
			}
		} else {
			// if not expandable
			lcd.clear();
			lcd.drawString("Track not adjustable", 0, 0);
			lcd.drawString("Press any button   ", 0, 1);
			Button.waitForAnyPress();
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
	 * 
	 * @return The size of the track (in cm)
	 */
	public double getTrack() {
		return track;
	}
	
	public void setTrack (double track) {
		this.track = track;
	}

	/**
	 * Gets if the track is expandable
	 * 
	 * @return True if the track can expand
	 */
	public boolean isExpandable() {
		return expandable;
	}

	/**
	 * Sets if the track is expandable
	 * 
	 * @param expandable
	 *            True if the track can expand
	 */
	public void setExpandable(boolean expandable) {
		this.expandable = expandable;
		if (!expandable) {

		}
	}

	/**
	 * Gets the used wheel radius previously set Default is the wheel radius of the
	 * screw design
	 * 
	 * @return the wheel radius (double)
	 */
	public double getWheelRad() {
		return wheelRad;
	}

	/**
	 * Gets the light sensor distance used depending on the design Default is the
	 * light sensor distance of the screw
	 * 
	 * @return the light sensor used
	 */
	public double getLightSensorDistance() {
		return lightSensorDistance;
	}

	/**
	 * Method to set the robot parameters according to the design used. If the
	 * SCREW_DESIGN is given, considers the track is at its minimum.
	 * 
	 * @param robotUsed
	 *            Robot design used
	 */
	public void setDesignConstants(GamePlan.Robot robotUsed) {
		switch (robotUsed) {
		case SCREW_DESIGN:
			this.track = MIN_SCREW_TRACK;
			this.wheelRad = SCREW_WHEEL_RAD;
			this.lightSensorDistance = LIGHT_SENSOR_DISTANCE_SCREW;
			this.expandable = true;
			this.trackExpansionMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
			break;
		case TANK:
			this.track = TANK_TRACK;
			this.wheelRad = TANK_WHEEL_RAD;
			this.lightSensorDistance = LIGHT_SENSOR_DISTANCE_TANK;
			this.expandable = false;
			break;

		}
	}

	/**
	 * Method to set the robot parameters according to the design used. If the
	 * SCREW_DESIGN is given, considers the track is at its minimum.
	 * 
	 * @param robotUsed
	 *            Robot design used
	 * @param expanded
	 *            If the SCREW_DESIGN is given, True for expanded will set the track
	 *            to its maximum
	 */
	public void setDesignConstants(GamePlan.Robot robotUsed, boolean expanded) {
		switch (robotUsed) {
		case SCREW_DESIGN:
			this.wheelRad = SCREW_WHEEL_RAD;
			this.lightSensorDistance = LIGHT_SENSOR_DISTANCE_SCREW;
			this.expandable = true;
			this.trackExpansionMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
			if (expanded) {
				this.track = MAX_SCREW_TRACK;
			} else {
				this.track = MIN_SCREW_TRACK;
			}
			break;
		case TANK:
			this.track = TANK_TRACK;
			this.wheelRad = TANK_WHEEL_RAD;
			this.lightSensorDistance = LIGHT_SENSOR_DISTANCE_TANK;
			this.expandable = false;
			break;
		}
	}

	/**
	 * Converts the track expansion displacement into rotation in degrees
	 * 
	 * @param distance
	 *            The displacement needed to expand the track
	 * @return The rotation needed to expand the tracks (in degrees)
	 */
	private int convertExpansion(double distance) {
		return (int) (360 * distance / SCREW_PITCH);
	}

	/**
	 * Converts rotation in degrees into the track expansion displacement
	 * 
	 * @param The
	 *            rotation of the screw (in degrees)
	 * @return distance The displacement of the track
	 */
	private double convertRotation(int rotation) {
		return (double) (SCREW_PITCH * rotation / 360);
	}
}
