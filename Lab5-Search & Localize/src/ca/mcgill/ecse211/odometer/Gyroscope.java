package ca.mcgill.ecse211.odometer;

import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

public class Gyroscope {
	private EV3GyroSensor gyroSensor;
	private SampleProvider gyro;
	
	
	public Gyroscope(EV3GyroSensor gyroSensor) {
		this.gyroSensor=gyroSensor;
		SampleProvider gyroscope = gyroSensor.getAngleMode();
		resetToZero();
	}

	
	public void resetToZero() {
		gyroSensor.reset();
	}
	
	public void startAngleCalculation() {
		
	}
}
