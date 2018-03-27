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
		if (lightSensor.lineCrossed()) { // wait to cross a line
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

			if (lightSensor.lineCrossed()) { // wait to cross a line
				
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
		double dTheta = Math.toDegrees(thetaY) / 2 + 270 - lineAngles[((index-1)%4)];

		odo.setXYT(x + xLine * Navigation.TILE_SIZE, y + yLine * Navigation.TILE_SIZE,
				odo.getTheta() + dTheta - 90 * corner);
		Sound.beep();
		lightLocalizerDone = true;
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
	public void lightloc(int corner) {
		//This method only works when the robot is
		//In one of the corner tiles
		// A wall parallel to it on its left
		// A wall perpendicular to it behind
		// no wall to the right
		// so like theta=0 in corner 0
		
		//makes it think its in the middle of the tile in corner 0
		odo.setXYT(navigation.TILE_SIZE/2, navigation.TILE_SIZE/2, 0);
		
		//back up into the back wall
		navigation.backUp(Navigation.TILE_SIZE/2);
		odo.setTheta(0);
		gyroscope.setAngle(0);
		
		//go forward until you detect the first y line
		navigation.travelForward();
		while (!lightSensor.lineCrossed());
		odo.setY(Navigation.TILE_SIZE+Math.abs(dynamicTrack.getLightSensorDistance()));
		navigation.stopMotors();
		
		//back up a bit
		navigation.backUp(Math.abs(dynamicTrack.getLightSensorDistance()));
		
		//turn right and go see the first x line
		navigation.turn(90);
		navigation.travelForward();
		while (!lightSensor.lineCrossed());
		odo.setX(Navigation.TILE_SIZE+Math.abs(dynamicTrack.getLightSensorDistance()));
		navigation.stopMotors();
		
		// go the the first crossing 
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
			odo.setXYT(navigation.TILE_SIZE, navigation.TILE_SIZE, 270);
			gyroscope.setAngle(270);
			break;
		case 2:
			odo.setXYT(navigation.TILE_SIZE, navigation.TILE_SIZE, 180);
			gyroscope.setAngle(180);
			break;
		case 3:
			odo.setXYT(navigation.TILE_SIZE, navigation.TILE_SIZE, 90);
			gyroscope.setAngle(90);
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
