package ca.mcgill.ecse211.localizer;

import ca.mcgill.ecse211.lab5.*;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.utility.Delay;

/**
 * Class for handling the light localization procedure at any crossing
 * 
 * @author Xavier Pellemans
 * @author Thomas Bahen
 */
public class LightLocalizer {

	/**
	 * Variable for light localization
	 */
	private final static int CRASH_LOC_ERROR=2;// cm of error
	private final Navigation navigation;
	private final TrackExpansion dynamicTrack;
	private final ColorSensor lightSensor;
	private final Odometer odo;
	private final Gyroscope gyroscope;
	private boolean lightLocalizerDone; // tells when the localization is over

	/**
	 * Create a lightLocalizer object
	 * 
	 * @param navigation
	 *            Navigation used
	 * @param lightSensor
	 *            EV3ColorSensor used
	 * @param odo
	 *            Odometer used
	 * @param config
	 *            The Lab5.RobotConfig, i.e. the wheel positioning
	 */
	public LightLocalizer(Navigation navigation, TrackExpansion dynamicTrack, ColorSensor lightSensor, Odometer odo, Gyroscope gyro) {
		this.odo = odo;
		this.navigation = navigation;
		this.dynamicTrack=dynamicTrack;
		this.lightSensor = lightSensor;
		this.gyroscope=gyro;
		this.lightLocalizerDone = false;
	}

	/**
	 * Localization in any line crossing
	 * Only corrects the Odometer values
	 * Does not go over the line crossing after localizing
	 * Assumes light sensor is at the front (positive Light sensor distance)
	 * @param xLine x line coordinate (0, 1, 2, ...) according to number of lines in x
	 * @param yLine x line coordinate (0, 1, 2, ...) according to number of lines in y
	 * @param corner  Corner from where the robot is relative to this intersection
	 * 0: robot is in lower left quadrant
	 * 1: robot is in lower right quadrant
	 * 2: robot is in upper right quadrant
	 * 3: robot is in upper left quadrant
	 */
	public void doLocalization(int xLine, int yLine, int corner) {
		Sound.beep();
		navigation.turnTo(45-90*corner);
		navigation.travelForward();
		if (lightSensor.lineDetected()) { // wait to cross a line
			navigation.stopMotors();
		}
		// travel to "center" the robot on the cross (actually makes it deliberately offset)
		if(dynamicTrack.getLightSensorDistance()>0) {
			navigation.travel(1.2*dynamicTrack.getLightSensorDistance());
		}else {
			navigation.backUp(1.2*dynamicTrack.getLightSensorDistance());
		}

		// Read in the angles
		double[] lineAngles = new double[4];
		int index = (corner * -1) % 4; // 0 -> 0, 1 -> 3, 2 -> 2, 3 -> 1

		if(dynamicTrack.getLightSensorDistance()<0) { //if sensor is at the back
			index=(index+2)%4;
		}
		for (int lineCounter = 0; lineCounter < 4; lineCounter++) {

			navigation.rotate(Navigation.Turn.CLOCK_WISE);// Rotate

			Delay.msDelay(500); // wait for initialization to make sure not initializing on the line

			if (lightSensor.lineDetected()) { // wait to cross a line
				
				lineAngles[index] = odo.getTheta(); // 0: positive x, 1: negative y, 2: negative x, 3: positive y
				index = (index + 1) % 4; // cycle index to record good angle with line
			}
		}
		navigation.stopMotors(); // stop

		// Angle and position calculation
		double thetaY = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[1]), Math.toRadians(lineAngles[3]));
		double ThetaX = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[0]), Math.toRadians(lineAngles[2]));
		double x = 0, y = 0;
		
		switch (corner) {
		// correct coordinates
		// end corner may change due to Light Sensor distance
		case 0:
			x = dynamicTrack.getLightSensorDistance() * Math.cos(thetaY / 2); // ends more in upper right
			y = dynamicTrack.getLightSensorDistance() * Math.cos(ThetaX / 2);
			break;
		case 1:
			x = -dynamicTrack.getLightSensorDistance() * Math.cos(thetaY / 2); // ends more in upper left
			y = dynamicTrack.getLightSensorDistance() * Math.cos(ThetaX / 2);
			break;
		case 2:
			x = -dynamicTrack.getLightSensorDistance() * Math.cos(thetaY / 2); // ends more in lower left
			y = -dynamicTrack.getLightSensorDistance() * Math.cos(ThetaX / 2);
			break;
		case 3:
			x = dynamicTrack.getLightSensorDistance() * Math.cos(thetaY / 2); // ends more in lower right
			y = -dynamicTrack.getLightSensorDistance() * Math.cos(ThetaX / 2);
			break;
		}
		double dTheta = Math.toDegrees(thetaY) / 2 + 270 - lineAngles[2];

		odo.setXYT(x + xLine * Navigation.TILE_SIZE, y + yLine * Navigation.TILE_SIZE,
				odo.getTheta() + dTheta - 90 * corner);
		Sound.beep();
		lightLocalizerDone = true;
	}


	/**
	 * Angle localization method for when the robot is on top of a crossing.
	 * The robot needs to have its center roughly near a crossing.
	 * The robot will detect the 4 lines,
	 * Then it will correct its theta based on its 
	 * old theta value.
	 * Method useful for localizing before/after crossing the river
	 */
	public void angleLocalizer() {
		Sound.beepSequence();
		Sound.beepSequence();
		Sound.beepSequence();
		double[] lineAngles = new double[4];
		double baseTheta;
		//determine what line will be seen first
		int index;
		if(odo.getTheta()<45) {
			baseTheta=45;
			index=3;
		}else if(odo.getTheta()<135) {
			baseTheta=135;
			index=0;
		}else if(odo.getTheta()<225) {
			baseTheta=225;
			index=1;
		}else {
			baseTheta=315;
			index=2;
		}
		navigation.turnTo(baseTheta);
		
		//only needs to read 3 lines to correct the theta
		for (int lineCounter = 0; lineCounter < 3; lineCounter++) {

			navigation.rotate(Navigation.Turn.CLOCK_WISE);// Rotate

			Delay.msDelay(500); // wait for initialization to make sure not initializing on the line

			if (lightSensor.lineDetected()) { // wait to cross a line
				
				lineAngles[index] = odo.getTheta(); // 0: positive y, 1: positive x, 2: negative y, 3: negative x
				index = (index + 1) % 4; // cycle index to record good angle with line
			}
		}
		navigation.stopMotors(); // stop

		// Angle and position calculation
		double thetaY = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[1]), Math.toRadians(lineAngles[3]));
		double dTheta = Math.toDegrees(thetaY) / 2 + baseTheta;
		odo.setTheta(odo.getTheta() + dTheta);
	}
	
	
	/**
	 * Localization method for when the robot is in a corner tile
	 * Works only for TANK design with propulsion
	 * @param corner Corner where the robot is
	 * 0: robot is in lower left corner of the map
	 * 1: robot is in lower right corner of the map
	 * 2: robot is in upper right corner of the map
	 * 3: robot is in upper left corner of the map
	 */
	public void crashLocalizer(int corner) {
		//This method only works when the robot is
		//In one of the corner tiles
		// A wall parallel to it on its left
		// A wall perpendicular to it behind
		// no wall to the right
		// so like theta=0 in corner 0
		
		//makes it think its in the middle of the tile in corner 0
		odo.setXYT(navigation.TILE_SIZE/2, navigation.TILE_SIZE/2, 0);
		Delay.msDelay(500);
		//back up into the back wall
		crashIntoWall(GamePlan.Direction.SOUTH);
		
		//go forward until you detect the first y line
		navigation.travelForward();
		while (!lightSensor.lineDetected());
		odo.setY(Navigation.TILE_SIZE+Math.abs(dynamicTrack.getLightSensorDistance())+CRASH_LOC_ERROR);
		navigation.stopMotors();
		navigation.backUp(Navigation.TILE_SIZE/2);
		
		//turn right and go see the first x line
		navigation.turn(90);
		navigation.travelForward();
		while (!lightSensor.lineDetected());
		odo.setX(Navigation.TILE_SIZE+Math.abs(dynamicTrack.getLightSensorDistance())+CRASH_LOC_ERROR);
		navigation.stopMotors();
		navigation.backUp(Navigation.TILE_SIZE/2);
		
		// go the the first crossing 
		
		navigation.setForwardSpeed(Navigation.LOCALIZATION_SPEED);
		navigation.goToPoint(1, 1);
		navigation.turnTo(0);
		odo.correctAngle();
		navigation.turnTo(0);
		
		//updates the odometer values to their right values
		switch (corner) {		
		case 0:
			odo.setXYT(navigation.TILE_SIZE, navigation.TILE_SIZE, 0);
			break;
		case 1:
			odo.setXYT(7*navigation.TILE_SIZE, navigation.TILE_SIZE, 270); //(7,1)
			gyroscope.setAngle(270);
			break;
		case 2:
			odo.setXYT(7*navigation.TILE_SIZE, 7*navigation.TILE_SIZE, 180);//(7,7)
			gyroscope.setAngle(180);
			break;
		case 3:
			odo.setXYT(navigation.TILE_SIZE, 7*navigation.TILE_SIZE, 90);//(1,7)
			gyroscope.setAngle(90);
			break;
		}
	}
	
	/**
	 * Method to crash into the specified wall and update the 
	 * odometer and gyroscope angles as well as the coordinate
	 * of the axis perpendicular to that wall
	 *  
	 * @param wallSide Wall to crash into
	 */
	public void crashIntoWall(GamePlan.Direction wallSide) {
		switch(wallSide) {
		case CENTER:
			//do nothing
			break;
		case EAST:
			navigation.turnTo(270);
			navigation.backUp((EV3WifiClient.X_GRID_LINES+0.5)*Navigation.TILE_SIZE-odo.getX());
			odo.setTheta(270);
			gyroscope.setAngle(270);
			odo.setX(EV3WifiClient.X_GRID_LINES*Navigation.TILE_SIZE-Math.abs(dynamicTrack.getLightSensorDistance())-CRASH_LOC_ERROR);
			break;
		case NORTH:
			navigation.turnTo(180);
			navigation.backUp((EV3WifiClient.Y_GRID_LINES+0.5)*Navigation.TILE_SIZE-odo.getY());
			odo.setTheta(180);
			gyroscope.setAngle(180);
			odo.setY(EV3WifiClient.Y_GRID_LINES*Navigation.TILE_SIZE-Math.abs(dynamicTrack.getLightSensorDistance())-CRASH_LOC_ERROR);
			break;
		case WEST:
			navigation.turnTo(90);
			navigation.backUp((0.5)*Navigation.TILE_SIZE+odo.getX());
			odo.setTheta(90);
			gyroscope.setAngle(90);
			odo.setX(Math.abs(dynamicTrack.getLightSensorDistance())+CRASH_LOC_ERROR);
			break;
		case SOUTH:
			navigation.turnTo(0);
			navigation.backUp((0.5)*Navigation.TILE_SIZE+odo.getY());
			odo.setTheta(0);
			gyroscope.setAngle(90);
			odo.setY(Math.abs(dynamicTrack.getLightSensorDistance())+CRASH_LOC_ERROR);
			break;
		}
		
	}
	
	
	/**
	 * Method to initialize odometer coordinates while traveling to the first water feature.
	 * The robot will take the corner it is in and the direction of its entry point.
	 * It will follow the wall after crashing to the one behind it.
	 * To decide which wall to follow, it will see what path will lead it
	 * the closest to the entry point specified.
	 * After going along the wall and stopping at the height on the entry point,
	 * it will crash into the wall it followed to update the next coordinate parameter.
	 * After that, it will travel to the entry point specified.
	 * 
	 * This method does not care about the search zones, so it will not
	 * try to avoid them.
	 * 
	 * @param corner Starting corner of the robot
	 * @param entryPoint Direction of the entry side of the first water feature to cross
	 * @param xEntry X coordinate of the entry point (in cm)
	 * @param yEntry Y coordinate of the entry point (in cm)
	 * @throws Exception if the entry point specified doesn't match the possibilities
	 * 			for the specified corner.
	 */
	public void initialCrashLoc(int corner, GamePlan.Direction entryPoint, double xEntry, double yEntry) throws Exception {
		switch(corner) {
		case 0:
			switch(entryPoint) {
			//Entry side of the first water feature to cross.
			case CENTER:
			case NORTH:
			case EAST:
				throw new Exception("Erroneous entry point found when localizing.");
			case SOUTH:
				//Robot will follow the SOUTH wall
				odo.setXYT(Navigation.TILE_SIZE/2, Navigation.TILE_SIZE/2, 0);
				crashIntoWall(GamePlan.Direction.WEST);
				navigation.travelTo(xEntry, odo.getY());
				crashIntoWall(GamePlan.Direction.SOUTH);
				navigation.travelTo(xEntry, yEntry);
				break;
			case WEST:
				//Robot will follow the WEST wall
				odo.setXYT(Navigation.TILE_SIZE/2, Navigation.TILE_SIZE/2, 0);
				crashIntoWall(GamePlan.Direction.SOUTH);
				navigation.travelTo(odo.getX(), yEntry);
				crashIntoWall(GamePlan.Direction.WEST);
				navigation.travelTo(xEntry, yEntry);
				break;
			}
			break;
		case 1:
			switch(entryPoint) {
			case CENTER:
			case NORTH:
			case WEST:
				throw new Exception("Erroneous entry point found when localizing.");
			case SOUTH:
				//Robot will follow the SOUTH wall
				odo.setXYT(Navigation.TILE_SIZE*(EV3WifiClient.X_GRID_LINES-0.5), Navigation.TILE_SIZE/2, 270);
				crashIntoWall(GamePlan.Direction.EAST);
				navigation.travelTo(xEntry, odo.getY());
				crashIntoWall(GamePlan.Direction.SOUTH);
				navigation.travelTo(xEntry, yEntry);
				break;
			case EAST:
				//Robot will follow the EAST wall
				odo.setXYT(Navigation.TILE_SIZE*(EV3WifiClient.X_GRID_LINES-0.5), Navigation.TILE_SIZE/2, 270);
				crashIntoWall(GamePlan.Direction.SOUTH);
				navigation.travelTo(odo.getX(), yEntry);
				crashIntoWall(GamePlan.Direction.EAST);
				navigation.travelTo(xEntry, yEntry);
				break;
			}
			break;
		case 2:
			switch(entryPoint) {
			case CENTER:
			case SOUTH:
			case WEST:
				throw new Exception("Erroneous entry point found when localizing.");
			case NORTH:
				//Robot will follow the NORTH wall
				odo.setXYT(Navigation.TILE_SIZE*(EV3WifiClient.X_GRID_LINES-0.5), Navigation.TILE_SIZE*(EV3WifiClient.X_GRID_LINES-0.5), 180);
				crashIntoWall(GamePlan.Direction.EAST);
				navigation.travelTo(xEntry, odo.getY());
				crashIntoWall(GamePlan.Direction.NORTH);
				navigation.travelTo(xEntry, yEntry);
				break;
			case EAST:
				//Robot will follow the EAST wall
				odo.setXYT(Navigation.TILE_SIZE*(EV3WifiClient.X_GRID_LINES-0.5), Navigation.TILE_SIZE*(EV3WifiClient.X_GRID_LINES-0.5), 180);
				crashIntoWall(GamePlan.Direction.NORTH);
				navigation.travelTo(odo.getX(), yEntry);
				crashIntoWall(GamePlan.Direction.EAST);
				navigation.travelTo(xEntry, yEntry);
				break;
			}
			break;
		case 3:
			switch(entryPoint) {
			case CENTER:
			case SOUTH:
			case EAST:
				throw new Exception("Erroneous entry point found when localizing.");
			case NORTH:
				//Robot will follow the NORTH wall
				odo.setXYT(Navigation.TILE_SIZE/2, Navigation.TILE_SIZE*(EV3WifiClient.X_GRID_LINES-0.5), 90);
				crashIntoWall(GamePlan.Direction.WEST);
				navigation.travelTo(xEntry, odo.getY());
				crashIntoWall(GamePlan.Direction.NORTH);
				navigation.travelTo(xEntry, yEntry);
				break;
			case WEST:
				//Robot will follow the WEST wall
				odo.setXYT(Navigation.TILE_SIZE/2, Navigation.TILE_SIZE*(EV3WifiClient.X_GRID_LINES-0.5), 90);
				crashIntoWall(GamePlan.Direction.NORTH);
				navigation.travelTo(odo.getX(), yEntry);
				crashIntoWall(GamePlan.Direction.WEST);
				navigation.travelTo(xEntry, yEntry);
				break;
			}
			break;
		}
	}
	
	

	
	/**
	 * Gets if the localization is done
	 * 
	 * @return if the localization has been made
	 */
	public boolean isFinished() {
		return lightLocalizerDone;
	}
}
