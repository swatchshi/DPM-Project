package ca.mcgill.ecse211.lab5;

import java.util.*;
import ca.mcgill.ecse211.*;
import ca.mcgill.ecse211.localizer.ColorSensor;
import lejos.ev3.*;
import lejos.hardware.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ColorRotation {

	private boolean status; // false: top position, true bot position
	private EV3MediumRegulatedMotor rotateMotor;

	public ColorRotation(EV3MediumRegulatedMotor rotateMotor) {
		this.rotateMotor = rotateMotor;

		// TODO Auto-generated constructor stub
	}

	public void run() {
		rotateMotor.setSpeed(40);
		status = false;
		rotate(status);

	}

	public boolean rotate(boolean status) {
		if (status == true) {
			rotateMotor.rotate(-135, true);
			status = false;
		} else {
			rotateMotor.rotate(135, true);
			status = true;
		}
		return status;
	}

}
