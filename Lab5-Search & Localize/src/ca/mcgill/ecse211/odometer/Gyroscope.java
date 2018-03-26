package ca.mcgill.ecse211.odometer;

import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

/**
 * Class for polling the data from the EV3GyroSensor.
 * @author Xavier
 *
 */
public class Gyroscope {
	private EV3GyroSensor gyroSensor;
	private SampleProvider gyro;
	private double previousAngle;
	private double angleOffset;
	/**
	 * Constructor for the gyroscope
	 * Initializes the gyroscope variables
	 * 
	 * @param gyroSensor The EV3GyroSensor used
	 */
	public Gyroscope(EV3GyroSensor gyroSensor) {
		this.gyroSensor=gyroSensor;
        gyro= gyroSensor.getAngleMode();
		resetToZero();
	}

	
	/**
	 * Reset the angle value to zero
	 */
	public void resetToZero() {
		gyroSensor.reset();
		previousAngle=0;
	}
	
	public void setPreviousAngle(double previousAngle) {
		previousAngle = this.previousAngle;
	}
	/**
	 * Sets the angle by adding an offset to the gyroscope angle
	 * 
	 * @param angle Angle at which to reset the gyroscope
	 */
	public void setAngle(double angle) {
		angleOffset=angle-getAngle();
	}
	
	/**
	 * Method to save the raw value of the gyroscope
	 * as the previous angle.
	 */
	public void startAngleCalculation() {
		float[] sample=new float[gyro.sampleSize()];
		gyro.fetchSample(sample, 0);
		previousAngle=sample[0];
	}
	
	/**
	 * Method to get the angle displacement using the previous angle saved
	 * @return The angle displacement (positive clockwise)
	 */
	public double getAngleDisplacement() {
		double dTheta;
		float[] sample=new float[gyro.sampleSize()];
		gyro.fetchSample(sample, 0);
		dTheta=sample[0]-previousAngle;
		previousAngle=sample[0];
		return dTheta;
	}
	
	/**
	 * Gets the angle of the gyroscope (with offset)
	 * Positively defined clockwise
	 * 
	 * @return The angle with its offset
	 */
	public double getAngle() {
		float[] sample=new float[gyro.sampleSize()];
		gyro.fetchSample(sample, 0);
		return (double) ((-1*((sample[0]+angleOffset) % 360)) % 360);
	}
	
	/**
	 * Gets the angle calculated by the gyroscope
	 * (no offset)
	 * Positively defined clockwise
	 * 
	 * @return The angle with no offset
	 */
	public double getCalculatedAngle() {
		float[] sample=new float[gyro.sampleSize()];
		gyro.fetchSample(sample, 0);
		return (double) ((-1*(sample[0]) % 360) % 360);
	}
	
	/**
	 * Gets the raw angle of the gyroscope
	 * (no offset)
	 * Positively defined counter clockwise
	 * 
	 * @return The angle with no offset
	 */
	public double getRawAngle() {
		float[] sample=new float[gyro.sampleSize()];
		gyro.fetchSample(sample, 0);
		return (double) ((sample[0]) % 360);
	}
}
