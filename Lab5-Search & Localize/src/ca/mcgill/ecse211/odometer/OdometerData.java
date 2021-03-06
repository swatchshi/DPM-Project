package ca.mcgill.ecse211.odometer;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import ca.mcgill.ecse211.lab5.Display;
import ca.mcgill.ecse211.lab5.GamePlan;

/**
 * This class stores and provides thread safe access to the odometer data.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 * @author Xavier Pellemans
 * @author Thomas Bahen
 */

public class OdometerData {

	// Position parameters
	private volatile double x; // x-axis position
	private volatile double y; // y-axis position
	private volatile double theta; // Head angle


	// Thread control tools
	private static Lock lock = new ReentrantLock(true); // Fair lock for
														// concurrent writing
	private volatile boolean isReseting = false; // Indicates if a thread is
													// trying to reset any
													// position parameters
	private Condition doneReseting = lock.newCondition(); // Let other threads
															// know that a reset
															// operation is
															// over.
	public static final int MAX_ANGLE_ERROR = 20;
	public static final int MIN_ANGLE_ERROR = 2;
	protected boolean doThetaCorrection = false;
	protected boolean enablePrint = true;
	protected Display display;
	
	protected Gyroscope gyroscope;

	/**
	 * Default constructor. The constructor is private. A factory is used instead
	 * such that only one instance of this class is ever created.
	 * 
	 * @param gyroscope
	 *            used by the Odometer
	 * @throws OdometerExceptions  if display creation error
	 */
	protected OdometerData(Gyroscope gyroscope) throws OdometerExceptions {
		this.x = 0;
		this.y = 0;
		this.theta = 0;
		this.gyroscope = gyroscope;
	}

	

	/**
	 * Return the Odomometer data.
	 * <p>
	 * Writes the current position and orientation of the robot onto the odoData
	 * array. odoData[0] = x, odoData[1] = y; odoData[2] = theta;
	 * 
	 * @param position
	 *            the array to store the odometer data
	 * @return the odometer data.
	 */
	public synchronized double[] getXYT() {
		double[] position = new double[3];
		lock.lock();
		try {
			while (isReseting) { // If a reset operation is being executed, wait
				// until it is over.
				doneReseting.await(); // Using await() is lighter on the CPU
				// than simple busy wait.
			}

			position[0] = x;
			position[1] = y;
			position[2] = theta;

		} catch (InterruptedException e) {
			// Print exception to screen
			e.printStackTrace();
		} finally {
			lock.unlock();
		}

		return position;
	}

	/**
	 * Gets the theta
	 * 
	 * @return the angle theta
	 */
	public double getTheta() {
		return theta;
	}

	/**
	 * Gets the x coordinate
	 * 
	 * @return the x
	 */
	public double getX() {
		return x;
	}

	/**
	 * Gets the y coordinate
	 * 
	 * @return the y
	 */
	public double getY() {
		return y;
	}
	
	/**
	 * Gets the gyroscope theta angle used (with offset and defined clockwise)
	 * 
	 * @return the gyroscope angle
	 */
	public double getGyroTheta() {
		return gyroscope.getAngle();
	}
	

	/**
	 * Adds dx, dy and dtheta to the current values of x, y and theta, respectively.
	 * Useful for odometry.
	 * 
	 * @param dx
	 * @param dy
	 * @param dtheta
	 */
	public void update(double dx, double dy, double dtheta) {
		lock.lock();
		isReseting = true;
		try {
			x += dx;
			y += dy;
			theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates
															// within 360
															// degrees
			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
										// done reseting
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overrides the values of x, y and theta. Use for odometry correction.
	 * 
	 * @param x
	 *            the value of x
	 * @param y
	 *            the value of y
	 * @param theta
	 *            the value of theta
	 */
	public void setXYT(double x, double y, double theta) {
		lock.lock();
		isReseting = true;
		try {
			this.x = x;
			this.y = y;
			this.theta = theta;
			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
										// done reseting
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overrides x. Use for odometry correction.
	 * 
	 * @param x
	 *            the value of x
	 */
	public void setX(double x) {
		lock.lock();
		isReseting = true;
		try {
			this.x = x;
			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
										// done reseting
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overrides y. Use for odometry correction.
	 * 
	 * @param y
	 *            the value of y
	 */
	public void setY(double y) {
		lock.lock();
		isReseting = true;
		try {
			this.y = y;
			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
										// done reseting
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overrides theta. Use for odometry correction.
	 * 
	 * @param theta
	 *            the value of theta
	 */
	public void setTheta(double theta) {
		lock.lock();
		isReseting = true;
		try {
			this.theta = theta % 360;
			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
										// done reseting
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Correction of the angle with the gyroscope angle value if the difference is
	 * greater than the max angle error
	 * 
	 * @return a linear combination of the two angle if there is a big difference
	 */
	public void correctAngle() {
		double gyroAngle = gyroscope.getAngle();
		
		//Verifies that the error is not too small nor too big to correct
		if (Math.abs(theta - gyroAngle) < MAX_ANGLE_ERROR && Math.abs(theta - gyroAngle) > MIN_ANGLE_ERROR) {
			this.theta = theta * 0.2 + gyroAngle * 0.8; // change proportions to get more accurate correction
			gyroscope.setAngle(this.theta);
		}else {
			//if error is too small or too big, do nothing
		}
	}

	/**
	 * Sets the mode for the theta correction by the gyroscope.
	 * 
	 * @param doThetaCorrection
	 *            True if the theta will be corrected
	 */
	public void setDoThetaCorrection(boolean doThetaCorrection) {
		this.doThetaCorrection = doThetaCorrection;
	}
	
	/**
	 * Sets the print function of the display
	 * 
	 * @param enablePrint
	 *            True if the Display is allowed to print
	 */
	public void setEnablePrint(boolean enablePrint) {
		this.enablePrint = enablePrint;
		if(!enablePrint) { //if false, erase everything that was written before
			GamePlan.lcd.clear();
		}
	}
}
