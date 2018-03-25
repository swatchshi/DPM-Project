package ca.mcgill.ecse211.localizer;

import ca.mcgill.ecse211.lab5.*;
import ca.mcgill.ecse211.odometer.*;
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
	public LightLocalizer(Navigation navigation, TrackExpansion dynamicTrack, ColorSensor lightSensor, Odometer odo) {
		this.odo = odo;
		this.navigation = navigation;
		this.dynamicTrack=dynamicTrack;
		this.lightSensor = lightSensor;
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
		// travel to "center" the robot on the cross
		if(dynamicTrack.getLightSensorDistance()>0) {
			navigation.travel(dynamicTrack.getLightSensorDistance());
		}else {
			navigation.backUp(dynamicTrack.getLightSensorDistance());
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
		case 0:
			x = dynamicTrack.getLightSensorDistance() * Math.cos(thetaY / 2) + Navigation.TILE_SIZE; // ends more in upper right
			y = dynamicTrack.getLightSensorDistance() * Math.cos(ThetaX / 2) + Navigation.TILE_SIZE;
			break;
		case 1:
			x = -dynamicTrack.getLightSensorDistance() * Math.cos(thetaY / 2) + Navigation.TILE_SIZE; // ends more in upper left
			y = dynamicTrack.getLightSensorDistance() * Math.cos(ThetaX / 2) + Navigation.TILE_SIZE;
			break;
		case 2:
			x = -dynamicTrack.getLightSensorDistance() * Math.cos(thetaY / 2) + Navigation.TILE_SIZE; // ends more in lower left
			y = -dynamicTrack.getLightSensorDistance() * Math.cos(ThetaX / 2) + Navigation.TILE_SIZE;
			break;
		case 3:
			x = dynamicTrack.getLightSensorDistance() * Math.cos(thetaY / 2) + Navigation.TILE_SIZE; // ends more in lower right
			y = -dynamicTrack.getLightSensorDistance() * Math.cos(ThetaX / 2) + Navigation.TILE_SIZE;
			break;
		}
		double dTheta = Math.toDegrees(thetaY) / 2 + 270 - lineAngles[((index-1)%4)];

		odo.setXYT(x + xLine * Navigation.TILE_SIZE, y + yLine * Navigation.TILE_SIZE,
				odo.getTheta() + dTheta - 90 * corner);
		Sound.beep();
		lightLocalizerDone = true;
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
