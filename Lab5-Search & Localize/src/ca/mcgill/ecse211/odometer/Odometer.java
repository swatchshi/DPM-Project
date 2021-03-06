package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab5.Display;
import ca.mcgill.ecse211.lab5.GamePlan;
import ca.mcgill.ecse211.lab5.TrackExpansion;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Class responsible for keeping track of the movements of the robot Runs on a
 * thread
 * 
 * @author Xavier Pellemans
 * @author Thomas Bahen
 * @author Zhang Guangyi
 * @author Zhang Cara
 * @author Shi WenQi
 */
public class Odometer extends OdometerData implements Runnable {
	
	private static Odometer odo = null; // Returned as singleton

	/**
	 *  Motors and related variables
	 */
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private int leftMotorLastTachoCount;
	private int rightMotorLastTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private TrackExpansion.RobotConfig config;
	private TrackExpansion dynamicTrack;

	private double[] position;

	private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

	/**
	 * This is the default constructor of this class. It initiates all motors and
	 * variables once.It cannot be accessed externally.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param dynamicTrack
	 * @param gyroscope
	 * @param config
	 *            The GamePlan.RobotConfig, i.e. the wheel positioning
	 * @throws OdometerExceptions
	 */
	private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, TrackExpansion dynamicTrack,
			Gyroscope gyroscope, TrackExpansion.RobotConfig config) throws OdometerExceptions {
		// manipulation methods
		super(gyroscope); // creates OdometerData object
		this.config = config;
		switch (config) {
		case PROPULSION:
			this.leftMotor = leftMotor;
			this.rightMotor = rightMotor;
			break;
		case TRACTION:
			this.leftMotor = rightMotor;
			this.rightMotor = leftMotor;
			break;
		}

		// Reset the values of x, y and z to 0
		setXYT(0, 0, 0);
		gyroscope.resetToZero();
		// tacho count initializations
		this.leftMotorLastTachoCount = 0;
		this.rightMotorLastTachoCount = 0;
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;

		this.dynamicTrack = dynamicTrack;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used
	 * throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param dynamicTrack
	 * @param gyroscope
	 * @param config
	 *            The Lab5.RobotConfig, i.e. the wheel positioning
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			TrackExpansion dynamicTrack, Gyroscope gyroscope, TrackExpansion.RobotConfig config) throws OdometerExceptions {

		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			odo = new Odometer(leftMotor, rightMotor, dynamicTrack, gyroscope, config);
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be
	 * used only if an odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 * @throws OdometerException
	 *             when no Odometer instance is found
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods
	 * provided from the OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
		if (enablePrint) {
			try {
				this.display = Display.getDisplay(GamePlan.lcd, gyroscope);
			} catch (OdometerExceptions e1) {
				// the display does not work, but it is not needed
			}
		}
		long updateStart, updateEnd;
		double dX, dY, dTheta, distL, distR;

		leftMotorLastTachoCount = leftMotor.getTachoCount();
		rightMotorLastTachoCount = rightMotor.getTachoCount();

		while (true) {
			updateStart = System.currentTimeMillis();

			leftMotorTachoCount = leftMotor.getTachoCount();
			rightMotorTachoCount = rightMotor.getTachoCount();

			// Calculate new robot position based on tachometer counts

			position = getXYT(); // get the coordinates

			distL = Math.PI * dynamicTrack.getWheelRad() * (leftMotorTachoCount - leftMotorLastTachoCount) / 180; // convert
																													// left
																													// rotation
																													// to
																													// wheel
																													// displacement
			distR = Math.PI * dynamicTrack.getWheelRad() * (rightMotorTachoCount - rightMotorLastTachoCount) / 180; // convert
																													// right
																													// rotation
																													// to
																													// wheel
																													// displacement

			dTheta = (distL - distR) / dynamicTrack.getTrack(); // Calculating the instantaneous rotation magnitude
			
			//orientation of the motors on the robot
			if (config == TrackExpansion.RobotConfig.PROPULSION) {
				//motorized wheels are at the back of the robot
				dTheta = -Math.toDegrees(dTheta); // conversion to degrees
				position[2] += dTheta;

				dX = -0.5 * (distL + distR) * Math.sin(Math.toRadians(position[2])); // displacement in X with new angle
				dY = -0.5 * (distL + distR) * Math.cos(Math.toRadians(position[2])); // displacement in Y with new angle
			} else { // TRACTION
				//motorized wheels are at the front of the robot
				dTheta = Math.toDegrees(dTheta); // conversion to degrees
				position[2] += dTheta;

				dX = 0.5 * (distL + distR) * Math.sin(Math.toRadians(position[2])); // displacement in X with new angle
				dY = 0.5 * (distL + distR) * Math.cos(Math.toRadians(position[2])); // displacement in Y with new angle
			}

			leftMotorLastTachoCount = leftMotorTachoCount; // resets value of the left tachoCount
			rightMotorLastTachoCount = rightMotorTachoCount; // resets value of the right tachoCount

			// Update odometer values with new calculated values
			update(dX, dY, dTheta);
			//print new values if enabled
			if (enablePrint) {
				try {
					display.printData();
				} catch (Exception e) {
					// couldn't print
					// erase display
					GamePlan.lcd.clear();
				}
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}
}
