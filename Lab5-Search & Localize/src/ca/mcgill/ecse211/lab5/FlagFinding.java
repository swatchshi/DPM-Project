package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.EV3WifiClient.CoordParameter;
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
	
	private double LLx, LLy, URx, URy;
	private double xRange, yRange;
	private boolean headTurned = false;
	private TrackExpansion dynamicTrack;
	private ColorSensor colorSensor;
	private UltrasonicSensor usSensor;
	private Navigation navigation;
	private Odometer odo;
	private InternalClock internalClock;
	private ColorSensor.BlockColor blockWanted;
	private boolean flagFound;

	
	/**
	 * Constructor of the FlagFinding class
	 * 
	 * @param colorSensor
	 *            ColorSensor used
	 * @param usSensor
	 *            UltrasonicSensor used
	 *
	 * @throws OdometerExceptions
	 *             Throws an OdometerExceptions error back to the main if error in
	 *             instances
	 */
	public FlagFinding(TrackExpansion dynamicTrack, ColorSensor colorSensor, UltrasonicSensor usSensor,
			InternalClock internalClock) throws OdometerExceptions {
		this.dynamicTrack = dynamicTrack;
		this.colorSensor = colorSensor;
		this.usSensor = usSensor;
		this.navigation = Navigation.getNavigation();
		this.odo = Odometer.getOdometer();
		this.internalClock=internalClock;
	}

	/**
	 * Method to turn the ultrasonic sensor to the left
	 * 
	 * @param usSensorTurned
	 *            true if the ultrasonic sensor should turn to face left
	 *         
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
	 * included to prevent motor interruption.
	 * 
	 * Search algorithm:
	 * 1-Goes to the nearest side of the search zone
	 * 2-Begins searching by going counter-clockwise around
	 * the sides of the search zone.
	 * 3-When a block is detected by the ultrasonic sensor,
	 * the robot moves a bit forward to look if it is in
	 * danger of hitting a block ahead.
	 * a-If so, it skips the
	 * block with the obstructed path (will eventually detect it later).
	 * and checks if another block obstructs the path of the second
	 * 4-If not, the robot goes straight to the block and keeps going until it detects 
	 * its color (or goes beyond the search zone if there is a false positive)
	 * 5-After detecting the color, the robot backs up to the side it just came from.
	 * 6-If the block detected was not the desired one, the search algorithm continues (step 2).
	 * 7-After detecting the right block of if there is no time left to search,
	 * the search algorithm will end, thus returning to the procedures in GamePlan.
	 * 
	 * @param side
	 * 				on what side of the search zone the robot will start looking
	 * @param Lx
	 *            (int) X line of the lower left corner of the search zone
	 * @param Ly
	 *            (int) Y line of the lower left corner of the search zone
	 * @param Ux
	 *            (int) X line of the lower left corner of the search zone
	 * @param Uy
	 *            (int) Y line of the lower left corner of the search zone
	 *   
	 * @param blockWanted
	 *            ColorSensor.BlockColor of the block wanted
	 *
	 * @return true if the block was found
	 */
	public boolean findBlock(GamePlan.Direction side, int Lx, int Ly, int Ux, int Uy, 
			ColorSensor.BlockColor blockWanted) {
		//variables for the search
		internalClock.startSearchClock();
		this.LLx = Lx * Navigation.TILE_SIZE;
		this.LLy = Ly * Navigation.TILE_SIZE;
		this.URx = Ux * Navigation.TILE_SIZE;
		this.URy = Uy * Navigation.TILE_SIZE;
		this.xRange = Math.abs(this.URx - this.LLx) * Navigation.TILE_SIZE;
		this.yRange = Math.abs(this.URy - this.LLy) * Navigation.TILE_SIZE;
		this.blockWanted = blockWanted;
		
		
		//Go to closest corner of the search zone
		Sound.beep();
		switch(side) {
		case NORTH:
			goToUpperRight();
			break;
		case WEST:
			goToUpperLeft();
			break;
		case SOUTH:
			goToLowerLeft();
			break;
		case EAST:
			goToLowerRight();
			break;
		case CENTER:
			goToUpperRight(); //could have been any corner
			break;
		}
		
		Sound.beepSequenceUp();
		/* Search algorithm:
		 * 1-Goes to the nearest side of the search zone
		 * 2-Begins searching by going counter-clockwise around
		 * the sides of the search zone.
		 * 3-When a block is detected by the ultrasonic sensor,
		 * the robot moves a bit forward to look if it is in
		 * danger of hitting a block ahead.
		 * a-If so, it skips the
		 * block with the obstructed path (will eventually detect it later).
		 * and checks if another block obstructs the path of the second
		 * 4-If not, the robot goes straight to the block and keeps going until it detects 
		 * its color (or goes beyond the search zone if there is a false positive)
		 * 5-After detecting the color, the robot backs up to the side it just came from.
		 * 6-If the block detected was not the desired one, the search algorithm continues (step 2).
		 * 7-After detecting the right block of if there is no time left to search,
		 * the search algorithm will end, thus returning to the procedures in GamePlan.
		 */
		while (!internalClock.isSearchTimeUp() ) { // if the robot has not detected the target block
													//yet and there is some time left
			switch (side) {

			case SOUTH: // x-axis
				Delay.msDelay(500);
				navigation.turnTo(90);
				Sound.beepSequenceUp();
				rotateUltrasonicSensor(true);
				navigation.travelForward();
				while (usSensor.readDistance() > yRange + dynamicTrack.getTrack()
						&& odo.getX() < this.URx + dynamicTrack.getTrack()) {
					// continue going forward until end of search zone
				}
				navigation.stopMotors(); // stop robot
				if (checkForFlag(yRange + dynamicTrack.getTrack())) { // confirm there is a block

					navigation.stopMotors(); // make sure the robot is stopped
					// rotate sensor 90 degrees facing front
					Delay.msDelay(500);
					navigation.turnTo(0);
					rotateUltrasonicSensor(false);
					Delay.msDelay(500);
					navigation.travelForward(); // travel indefinitely

					while (colorSensor.getColorSeen() == ColorSensor.BlockColor.NoColorFound && odo.getY() < URy) {
						// when it comes close enough to the block to do colorID or too far
					}
					navigation.stopMotors(); // stop robot

					if (isDesiredBlock()) { // checks if it is of the right color
						beepSequence(SUCCESSFUL_BEEPING); // plays the success sequence

						// get out of search zone
						Delay.msDelay(500);
						navigation.backUpTo(odo.getX(), LLy - dynamicTrack.getTrack());
						rotateUltrasonicSensor(false);
						return true; // return that you found the block
					} else {
						// continue searching
						beepSequence(NOT_IT_BEEPING);
						Delay.msDelay(500);
						navigation.backUpTo(odo.getX(), LLy - dynamicTrack.getTrack());
						navigation.turnTo(90);
						rotateUltrasonicSensor(true);
						navigation.travel(BLOCK_WIDTH); //to unsee the previous block
					}
				}
				if (odo.getX() >= URx + dynamicTrack.getTrack()) { // end of SOUTH side
					side=GamePlan.Direction.EAST; // next side
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
					rotateUltrasonicSensor(false);
					Delay.msDelay(500);
					navigation.travelForward(); // travel indefinitely to go see block

					while (colorSensor.getColorSeen() == ColorSensor.BlockColor.NoColorFound && odo.getX() > LLx) {
						// when it comes close enough to the block to do colorID or too far
					}
					navigation.stopMotors(); // stop robot

					if (isDesiredBlock()) { // checks if it is of the right color
						beepSequence(SUCCESSFUL_BEEPING); // plays the success sequence

						// get out of search zone
						Delay.msDelay(500);
						navigation.backUpTo(URx + dynamicTrack.getTrack(), odo.getY());
						rotateUltrasonicSensor(false);
						return true; // return that you found the block

					} else {
						// continue searching
						beepSequence(NOT_IT_BEEPING);
						Delay.msDelay(500);
						navigation.backUpTo(URx + dynamicTrack.getTrack(), odo.getY());
						navigation.turnTo(0);
						rotateUltrasonicSensor(true);
						navigation.travel(BLOCK_WIDTH); //to unsee the previous block
					}
				}
				if (odo.getY() >= URy + dynamicTrack.getTrack()) { // end of EAST side
					side=GamePlan.Direction.NORTH; // next side
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
					rotateUltrasonicSensor(false);
					Delay.msDelay(500);
					navigation.travelForward(); // travel indefinitely

					while (colorSensor.getColorSeen() == ColorSensor.BlockColor.NoColorFound && odo.getY() > LLy) {
						// when it comes close enough to the block to do colorID or too far
					}
					navigation.stopMotors(); // stop robot

					if (isDesiredBlock()) { // checks if it is of the right color
						beepSequence(SUCCESSFUL_BEEPING); // plays the success sequence

						// get out of search zone
						Delay.msDelay(500);
						navigation.backUpTo(odo.getX(), URy + dynamicTrack.getTrack());
						rotateUltrasonicSensor(false);
						return true; // return that you found the block

					} else {
						// continue searching
						beepSequence(NOT_IT_BEEPING);
						Delay.msDelay(500);
						navigation.backUpTo(odo.getX(), URy + dynamicTrack.getTrack());
						navigation.turnTo(270);
						rotateUltrasonicSensor(true);
						navigation.travel(BLOCK_WIDTH); //to unsee the previous block
					}
				}
				if (odo.getX() <= LLx - dynamicTrack.getTrack()) { // end of NORTH side
					side=GamePlan.Direction.WEST; // next side
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
					rotateUltrasonicSensor(false);
					Delay.msDelay(500);
					navigation.travelForward(); // travel indefinitely

					while (colorSensor.getColorSeen() == ColorSensor.BlockColor.NoColorFound && odo.getX() < URx) {
						// when it comes close enough to the block to do colorID or too far
					}
					navigation.stopMotors(); // stop robot

					if (isDesiredBlock()) { // checks if it is of the right color
						beepSequence(SUCCESSFUL_BEEPING); // plays the success sequence

						// get out of search zone
						Delay.msDelay(500);
						navigation.backUpTo(LLx - dynamicTrack.getTrack(), odo.getY());
						rotateUltrasonicSensor(false);
						return true; // return that you found the block

					} else {
						// continue searching
						beepSequence(NOT_IT_BEEPING);
						Delay.msDelay(500);
						navigation.backUpTo(LLx - dynamicTrack.getTrack(), odo.getY());
						navigation.turnTo(180);
						rotateUltrasonicSensor(true);
						navigation.travel(BLOCK_WIDTH); //to unsee the previous block
					}
				}
				if (odo.getY() <= LLy - dynamicTrack.getTrack()) { // end of WEST side
					side=GamePlan.Direction.SOUTH; // next side
				}
				break;
			}
		}
		beepSequence(FAILURE_BEEPING);
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

		if (distanceFirstBlock +UltrasonicSensor.US_ERROR <= range) {
			// the block was seen
			navigation.stopMotors();
			Sound.beepSequence();

			// move .5 block ahead forward
			navigation.travel(0.5 * BLOCK_WIDTH);
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
	 * Makes the robot navigate to the lower left corner of the search zone in the
	 * zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToLowerLeft() {
		navigation.travelTo(
				LLx - dynamicTrack.getTrack(),
				LLy - dynamicTrack.getTrack());
		return true;
	}

	/**
	 * Makes the robot navigate to the lower right corner of the search zone in the
	 * zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToLowerRight() {
		navigation.travelTo(
				URx + dynamicTrack.getTrack(),
				LLy - dynamicTrack.getTrack());
		return true;
	}

	/**
	 * Makes the robot navigate to the upper left corner of the search zone in the
	 * zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToUpperLeft() {
		navigation.travelTo(
				LLx - dynamicTrack.getTrack(),
				URy + dynamicTrack.getTrack());
		return true;
	}

	/**
	 * Makes the robot navigate to the upper right corner of the search zone in the
	 * zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToUpperRight() {
		navigation.travelTo(
				URx + dynamicTrack.getTrack(),
				URy + dynamicTrack.getTrack());
		return true;
	}
}
