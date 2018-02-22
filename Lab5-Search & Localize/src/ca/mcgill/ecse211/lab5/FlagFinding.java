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


public class FlagFinding {
	
	private boolean dirUlt;  // direction facing of the ultsensor, ture = left, false = forward
	private EV3MediumRegulatedMotor rotateUltMotor = Lab5.sensorMotor;
	
public FlagFinding() {
	// TODO Auto-generated constructor stub
}
public boolean rotateUltsenor (boolean dirUlt) {
	
	if (dirUlt == false) { //
		rotateUltMotor.rotate(-90, true);
		dirUlt = true;
	}
	else  {
		rotateUltMotor.rotate(90, true);
		dirUlt = false;
	}
	
	
	
	return dirUlt;
	
}


public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
 }

 /**
  * Converts angle into distance for the wheels
  * 
  * @param radius of the wheels
  * @param width of the wheel base
  * @param angle rotation of the robot
  * @return
  */
 public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
 }




}



