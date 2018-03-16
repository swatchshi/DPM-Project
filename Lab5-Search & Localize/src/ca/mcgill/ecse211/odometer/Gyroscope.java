package ca.mcgill.ecse211.odometer;

import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

public class Gyroscope {
	private EV3GyroSensor gyroSensor;
	private SampleProvider gyro;
	private double previousAngle;
	
	public Gyroscope(EV3GyroSensor gyroSensor) {
		this.gyroSensor=gyroSensor;
		SampleProvider gyroscope = gyroSensor.getAngleMode();
		resetToZero();
	}

	
	public void resetToZero() {
		gyroSensor.reset();
		previousAngle=0;
	}
	
	public void startAngleCalculation() {
		float[] sample=new float[gyro.sampleSize()];
		gyro.fetchSample(sample, 0);
		previousAngle=sample[0];
	}
	
	public double getAngleDisplacement() {
		double dTheta;
		float[] sample=new float[gyro.sampleSize()];
		gyro.fetchSample(sample, 0);
		dTheta=sample[0]-previousAngle;
		previousAngle=sample[0];
		return dTheta;
	}
	
	public double getAngle() {
		float[] sample=new float[gyro.sampleSize()];
		gyro.fetchSample(sample, 0);
		return (double) sample[0];
	}
}
