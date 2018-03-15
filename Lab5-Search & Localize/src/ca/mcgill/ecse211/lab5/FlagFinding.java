package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.localizer.ColorSensor;
import ca.mcgill.ecse211.localizer.UltrasonicSensor;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.*;
import lejos.utility.Delay;

/**
 * Class for finding the desired colored flag with the specified search zone
 * 
 * @author Xavier Pellemans
 * @author Thomas Bahen
 * @author Zhang Guangyi
 * @author Zhang Cara
 * @author Shi WenQi
 *
 */
public class FlagFinding {

	private static final int US_ROTATION_AMPLITUDE = 90;
	private static final int SUCCESSFUL_BEEPING = 3;
	private static final int NOT_IT_BEEPING = 2;
	private static final int FAILURE_BEEPING = 6;
	private static final int BLOCK_WIDTH = 10;
	private static final long MAX_SEARCH_TIME = 270000; // 4.5 min is 270 000 ms
	private double LLx, LLy, URx, URy;
	private double xRange, yRange;
	private boolean headTurned = false;
	private TrackExpansion dynamicTrack;
	private ColorSensor colorSensor;
	private UltrasonicSensor usSensor;
	private Navigation navigation;
	private Odometer odo;
	private ColorSensor.BlockColor blockWanted;
	private boolean flagFound;
	private long searchStart; // start time

	
	/**
	 * Constructor of the FlagFinding class
	 * 
	 * @param colorSensor
	 *            ColorSensor used
	 * @param usSensor
	 *            UltrasonicSensor used
	 * @param blockWanted
	 *            ColorSensor.BlockColor of the block wanted
	 * @param LLx
	 *            (int) X of the lower left corner of the search zone
	 * @param LLy
	 *            (int) Y of the lower left corner of the search zone
	 * @param URx
	 *            (int) X of the lower left corner of the search zone
	 * @param URy
	 *            (int) Y of the lower left corner of the search zone
	 * @throws OdometerExceptions
	 *             Throws an OdometerExceptions error back to the main if error in
	 *             instances
	 */
	public FlagFinding(TrackExpansion dynamicTrack, ColorSensor colorSensor, UltrasonicSensor usSensor,
			ColorSensor.BlockColor blockWanted, int LLx, int LLy, int URx, int URy) throws OdometerExceptions {
		this.dynamicTrack = dynamicTrack;
		this.colorSensor = colorSensor;
		this.usSensor = usSensor;
		this.navigation = Navigation.getNavigation();
		this.odo = Odometer.getOdometer();
		this.blockWanted = blockWanted;
		this.LLx = LLx * Navigation.TILE_SIZE;
		this.LLy = LLy * Navigation.TILE_SIZE;
		this.URx = URx * Navigation.TILE_SIZE;
		this.URy = URy * Navigation.TILE_SIZE;
		this.xRange = Math.abs(URx - LLx) * Navigation.TILE_SIZE;
		this.yRange = Math.abs(URy - LLy) * Navigation.TILE_SIZE;
		this.searchStart = System.currentTimeMillis();
	}

	/**
	 * Method to turn the ultrasonic sensor to the left
	 * 
	 * @param usSensorTurned
	 *            true if the ultrasonic sensor should turn to face left
	 * @return true if the ultrasonic sensor faces left
	 */
	public boolean rotateUltrasonicSensor(boolean usSensorTurn) {

		if (usSensorTurn && !headTurned) { //
			GamePlan.usSensorMotor.rotate(US_ROTATION_AMPLITUDE, true);
			headTurned = true;
		} else if (!usSensorTurn && headTurned) {
			GamePlan.usSensorMotor.rotate(-US_ROTATION_AMPLITUDE, true);
			headTurned = false;
		}
		return headTurned;
	}

	/**
	 * Looks for the wanted block by sweeping the search area using the
	 * ultrasonicsensor and performing color detection using the light sensor Makes
	 * the robot circle the search area in a counter clockwise motion Delays are
	 * included to prevent motor interruption
	 * 
	 * @return true if the block was found
	 */
	public boolean findBlock() {
		Sound.beep();
		travelToLowerLeft();
		Sound.beep();
		int i = 0;
		while (i < GamePlan.Direction.values().length && timeElapsed() < MAX_SEARCH_TIME) { // if the robot has not detected 4 blocks
																				// on one side and still some time left

			GamePlan.Direction sideTest = GamePlan.Direction.values()[i];

			switch (sideTest) {

			case SOUTH: // x-axis
				Delay.msDelay(500);
				navigation.turnTo(90);
				rotateUltrasonicSensor(true);
				navigation.travelForward();
				while (usSensor.readDistance() > yRange + dynamicTrack.getTrack()
						&& odo.getX() < URx + dynamicTrack.getTrack()) {
					// continue going forward until end of search zone
				}
				navigation.stopMotors(); // stop robot
				if (checkForFlag(yRange + dynamicTrack.getTrack())) { // confirm there is a block

					navigation.stopMotors(); // make sure the robot is stopped
					// rotate sensor 90 degrees facing front
					Delay.msDelay(500);
					navigation.turnTo(0);
					navigation.stopMotors();
					Delay.msDelay(500);
					navigation.travelForward(); // travel indefinitely

					while (colorSensor.getColorSeen() == ColorSensor.BlockColor.NoColorFound && odo.getY() < URy) {
						// when it comes close enough to the block to do colorID or too far
					}
					navigation.stopMotors(); // stop robot

					if (isDesiredBlock()) { // checks if it is of the right color
						beepSequence(SUCCESSFUL_BEEPING); // plays the success sequence

						// go to UR corner of search zone
						Delay.msDelay(500);
						navigation.backUpTo(odo.getX(), LLy - dynamicTrack.getTrack());
						navigation.stopMotors();
						rotateUltrasonicSensor(false);
						travelToUpperRight();
						return true; // return that you found the block
					} else {
						// continue searching
						beepSequence(NOT_IT_BEEPING);
						Delay.msDelay(500);
						navigation.backUpTo(odo.getX(), LLy - dynamicTrack.getTrack());
						navigation.stopMotors();
						Delay.msDelay(500);
						navigation.turnTo(90);
						navigation.stopMotors();
						Delay.msDelay(500);
						navigation.travel(BLOCK_WIDTH);
					}
				}
				if (odo.getX() >= URx + dynamicTrack.getTrack()) { // end of BOTTOM side
					i++; // next side
				}
				break;
			case EAST: // y-axis
				Delay.msDelay(500);
				navigation.turnTo(0);
				rotateUltrasonicSensor(true);
				navigation.travelForward();
				while (usSensor.readDistance() > xRange + dynamicTrack.getTrack()
						&& odo.getY() < URy + dynamicTrack.getTrack()) {
					// continue going forward until end of search zone
				}
				navigation.stopMotors(); // stop robot
				if (checkForFlag(xRange + dynamicTrack.getTrack())) { // confirm there is a block

					navigation.stopMotors(); // make sure the robot is stopped
					// rotate sensor 90 degrees facing front
					Delay.msDelay(500);
					navigation.turnTo(270);
					navigation.stopMotors();
					Delay.msDelay(500);
					navigation.travelForward(); // travel indefinitely to go see block

					while (colorSensor.getColorSeen() == ColorSensor.BlockColor.NoColorFound && odo.getX() > LLx) {
						// when it comes close enough to the block to do colorID or too far
					}
					navigation.stopMotors(); // stop robot

					if (isDesiredBlock()) { // checks if it is of the right color
						beepSequence(SUCCESSFUL_BEEPING); // plays the success sequence

						// go to UR corner of search zone
						Delay.msDelay(500);
						navigation.backUpTo(URx + dynamicTrack.getTrack(), odo.getY());
						navigation.stopMotors();
						rotateUltrasonicSensor(false);
						travelToUpperRight();
						return true; // return that you found the block

					} else {
						// continue searching
						beepSequence(NOT_IT_BEEPING);
						Delay.msDelay(500);
						navigation.backUpTo(URx + dynamicTrack.getTrack(), odo.getY());
						navigation.stopMotors();
						Delay.msDelay(500);
						navigation.turnTo(0);
						navigation.stopMotors();
						Delay.msDelay(500);
						navigation.travel(BLOCK_WIDTH);
					}
				}
				if (odo.getY() >= URy + dynamicTrack.getTrack()) { // end of RIGHT side
					i++; // next side
				}
				break;
			case NORTH: // x-axis on the top
				Delay.msDelay(500);
				navigation.turnTo(270);
				rotateUltrasonicSensor(true);
				navigation.travelForward();
				while (usSensor.readDistance() > yRange + dynamicTrack.getTrack()
						&& odo.getX() > LLx - dynamicTrack.getTrack()) {
					// continue going forward until end of search zone
				}
				navigation.stopMotors(); // stop robot
				if (checkForFlag(yRange + dynamicTrack.getTrack())) { // confirm there is a block

					navigation.stopMotors(); // make sure the robot is stopped
					// rotate sensor 90 degrees facing front
					Delay.msDelay(500);
					navigation.turnTo(180);
					navigation.stopMotors();
					Delay.msDelay(500);
					navigation.travelForward(); // travel indefinitely

					while (colorSensor.getColorSeen() == ColorSensor.BlockColor.NoColorFound && odo.getY() > LLy) {
						// when it comes close enough to the block to do colorID or too far
					}
					navigation.stopMotors(); // stop robot

					if (isDesiredBlock()) { // checks if it is of the right color
						beepSequence(SUCCESSFUL_BEEPING); // plays the success sequence

						// go to UR corner of search zone
						Delay.msDelay(500);
						navigation.backUpTo(odo.getX(), URy + dynamicTrack.getTrack());
						navigation.stopMotors();
						rotateUltrasonicSensor(false);
						travelToUpperRight();
						return true; // return that you found the block

					} else {
						// continue searching
						beepSequence(NOT_IT_BEEPING);
						Delay.msDelay(500);
						navigation.backUpTo(odo.getX(), URy + dynamicTrack.getTrack());
						navigation.stopMotors();
						Delay.msDelay(500);
						navigation.turnTo(270);
						navigation.stopMotors();
						Delay.msDelay(500);
						navigation.travel(BLOCK_WIDTH);
					}
				}
				if (odo.getX() <= LLx - dynamicTrack.getTrack()) { // end of TOP side
					i++; // next side
				}
				break;
			case WEST: // y-axis back to starting point
				Delay.msDelay(500);
				navigation.turnTo(180);
				rotateUltrasonicSensor(true);
				navigation.travelForward();
				while (usSensor.readDistance() > xRange + dynamicTrack.getTrack()
						&& odo.getY() > LLy - dynamicTrack.getTrack()) {
					// continue going forward until end of search zone
				}
				navigation.stopMotors(); // stop robot
				if (checkForFlag(xRange + dynamicTrack.getTrack())) { // confirm there is a block

					navigation.stopMotors(); // make sure the robot is stopped
					// rotate sensor 90 degrees facing front
					Delay.msDelay(500);
					navigation.turnTo(90);
					navigation.stopMotors();
					Delay.msDelay(500);
					navigation.travelForward(); // travel indefinitely

					while (colorSensor.getColorSeen() == ColorSensor.BlockColor.NoColorFound && odo.getX() < URx) {
						// when it comes close enough to the block to do colorID or too far
					}
					navigation.stopMotors(); // stop robot

					if (isDesiredBlock()) { // checks if it is of the right color
						beepSequence(SUCCESSFUL_BEEPING); // plays the success sequence

						// go to UR corner of search zone
						Delay.msDelay(500);
						navigation.backUpTo(LLx - dynamicTrack.getTrack(), odo.getY());
						navigation.stopMotors();
						rotateUltrasonicSensor(false);
						travelToUpperRight();
						return true; // return that you found the block

					} else {
						// continue searching
						beepSequence(NOT_IT_BEEPING);
						Delay.msDelay(500);
						navigation.backUpTo(LLx - dynamicTrack.getTrack(), odo.getY());
						navigation.stopMotors();
						Delay.msDelay(500);
						navigation.turnTo(180);
						navigation.stopMotors();
						Delay.msDelay(500);
						navigation.travel(BLOCK_WIDTH);
					}
				}
				if (odo.getY() <= LLy - dynamicTrack.getTrack()) { // end of LEFT side
					i++; // next side
				}
				break;
			}
		}

		beepSequence(FAILURE_BEEPING);
		travelToUpperRight();
		return false;
	}

	/**
	 * Gets if it is the flag is of the desired color
	 * 
	 * @return (boolean) true if the block color is of the desired color
	 */
	public boolean isDesiredBlock() {

		if (colorSensor.getColorSeen() == blockWanted) {
			Sound.beep();
			flagFound = true;
		}
		return flagFound;
	}

	/**
	 * Check whether there is a block closer ahead or not (will advance to see)
	 * 
	 * @param range Range the UltrasonicSensor has to look in
	 * @return true when the robot needs to turn and go find out the color
	 */
	public boolean checkForFlag(double range) {

		boolean blockNear = true;

		double distanceFirstBlock = usSensor.readDistance();

		if (distanceFirstBlock <= range) {
			// the block was seen
			navigation.stopMotors();
			Sound.beepSequence();

			// move 1.5 block ahead forward
			navigation.travel(1.0 * BLOCK_WIDTH);
			navigation.stopMotors();
			// checks if there is a block ahead which would be hit
			if (usSensor.readDistance() <= distanceFirstBlock - UltrasonicSensor.US_ERROR) {
				blockNear = true; // block will be hit
			} else {
				blockNear = false; // no block to be hit
				Sound.beepSequenceUp();
			}
		}
		return (!blockNear); // tells if can go check colored block seen
	}

	/**
	 * Makes the EV3 beep a certain amount of beeps
	 * 
	 * @param numberBeeps
	 *            number of beeps wanted
	 */
	public void beepSequence(int numberBeeps) {
		for (int j = 0; j < numberBeeps; j++) {
			Sound.beep();
		}
	}

	/**
	 * Makes the robot travel to the lower left corner of the search zone by going
	 * around Depending on where it is currently
	 * 
	 * @return true if it has reached the lower left corner
	 */
	public boolean travelToLowerLeft() {
		GamePlan.Direction side;
		// determine the side of the of the search zone
		if (odo.getX() <= LLx) {
			side = GamePlan.Direction.WEST;
		} else if (odo.getX() <= URx) {
			if (odo.getY() <= LLy) {
				side = GamePlan.Direction.SOUTH;
			} else if (odo.getY() >= URy) {
				side = GamePlan.Direction.NORTH;
			} else {
				// center of search zone
				navigation.travelTo(LLx - dynamicTrack.getTrack(), LLy - dynamicTrack.getTrack());
				return true;
			}
		} else {
			side = GamePlan.Direction.EAST;
		}
		// goes around the search zone
		switch (side) {
		case WEST:
			navigation.travelTo(LLx - dynamicTrack.getTrack(), LLy - dynamicTrack.getTrack());
			break;
		case SOUTH:
			navigation.travelTo(LLx - dynamicTrack.getTrack(), LLy - dynamicTrack.getTrack());
			break;
		case EAST:
			navigation.travelTo(URx + dynamicTrack.getTrack(), LLy - dynamicTrack.getTrack());
			navigation.travelTo(LLx - dynamicTrack.getTrack(), LLy - dynamicTrack.getTrack());
			break;
		case NORTH:
			navigation.travelTo(LLx - dynamicTrack.getTrack(), URy + dynamicTrack.getTrack());
			navigation.travelTo(LLx - dynamicTrack.getTrack(), LLy - dynamicTrack.getTrack());
			break;
		}
		return true;
	}

	/**
	 * Makes the robot travel to the upper right corner of the search zone by going
	 * around Depending on where it is currently
	 * 
	 * @return true if it has reached the upper right corner
	 */
	public boolean travelToUpperRight() {
		GamePlan.Direction side;
		// determine the side of where the robot is
		if (odo.getX() <= LLx) {
			side = GamePlan.Direction.WEST;
		} else if (odo.getX() <= URx) {
			if (odo.getY() <= LLy) {
				side = GamePlan.Direction.SOUTH;
			} else if (odo.getY() >= URy) {
				side = GamePlan.Direction.NORTH;
			} else {
				// center of search zone
				navigation.travelTo(URx, URy);
				return true;
			}
		} else {
			side = GamePlan.Direction.EAST;
		}

		// go around search zone
		switch (side) {
		case WEST:
			navigation.travelTo(LLx - dynamicTrack.getTrack(), URy + dynamicTrack.getTrack());
			navigation.travelTo(URx, URy);

			break;
		case SOUTH:
			navigation.travelTo(URx + dynamicTrack.getTrack(), LLy - dynamicTrack.getTrack());
			navigation.travelTo(URx, URy);
			break;
		case EAST:
			navigation.travelTo(URx, URy);
			break;
		case NORTH:
			navigation.travelTo(URx, URy);
			break;
		}
		return true;
	}

	/**
	 * Gets the time elapsed in milliseconds
	 * 
	 * @return the number of milliseconds elapse since the start of the creation of
	 *         this FLagFinding instance
	 */
	public long timeElapsed() {
		return (System.currentTimeMillis() - searchStart);
	}
}
