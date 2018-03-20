package ca.mcgill.ecse211.odometer;

/**
 * This class is used to handle errors regarding the singleton pattern used for
 * the odometer and odometerData
 *
 */
@SuppressWarnings("serial")
public class OdometerExceptions extends Exception {

	/**
	 * Creates the exception
	 * @param Error the error message
	 */
	public OdometerExceptions(String Error) {
		super(Error);
	}

}
