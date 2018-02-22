package ca.mcgill.ecse211.lab5;


import java.util.*;
import ca.mcgill.ecse211.*;
import ca.mcgill.ecse211.localizer.ColorSensor;
import ca.mcgill.ecse211.localizer.ColorSensor.BlockColor;
import ca.mcgill.ecse211.localizer.UltrasonicSensor;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.ev3.*;
import lejos.hardware.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


public class FlagFinding {
	
	
	private static final int armRotationAmplitude=90;
	private static final int usRotationAmplitude=90;
	private boolean dirUlt=false;  // direction facing of the ultsensor, true = left, false = forward
	private boolean armDown=false;
	
	private EV3MediumRegulatedMotor rotateUltMotor = Lab5.sensorMotor;
	private ColorSensor colorSensor;
	private UltrasonicSensor usSensor;
	private Navigation navigation;
	private Odometer odo;
	private ColorSensor.BlockColor blockWanted;
	private boolean flagFound;
	
	public static enum Side{
		SIDE1,
		SIDE2,
		SIDE3,
		SIDE4
		
	}
	
	public FlagFinding(ColorSensor colorSensor, UltrasonicSensor usSensor, ColorSensor.BlockColor blockWanted) throws OdometerExceptions {
		this.colorSensor=colorSensor;
		this.usSensor=usSensor;
		this.navigation=Navigation.getNavigation();
		this.odo=Odometer.getOdometer();
		this.blockWanted=blockWanted;
	}
	
	
	
	public boolean rotateUltsenor (boolean dirUlt) {
		
		if (dirUlt) { //
			rotateUltMotor.rotate(-usRotationAmplitude, true);
			dirUlt = true;
		}
		else  {
			rotateUltMotor.rotate(usRotationAmplitude, true);
			dirUlt = false;
		}
		return dirUlt;
	}
	
	
	public boolean putArmDown(boolean armDown) {	
		if (armDown) { //
			rotateUltMotor.rotate(-armRotationAmplitude, true);
			armDown = true;
		}
		else  {
			rotateUltMotor.rotate(armRotationAmplitude, true);
			armDown = false;
		}
		return armDown;
	}
	 
	
	 
	 /**
	  * Looks for the wanted block by sweeping the search area using the ultrasonicsensor 
	  * and performing color detection using the light sensor
	  * 
	  * @param
	  */
	 public void sweep() {			
		int tempDistance = usSensor.getDistance();
		double y = odo.getY();
		int i = 0;

		while (i < Side.values().length){ //if the robot has not detected 4 blocks on one side
			
			Side sideTest=Side.values()[i];
		
			switch (sideTest){
				
				case SIDE1 : //x-axis
					navigation.goToPoint(7, 3);
					if (tempDistance <120){ //if the robot detects an object
						// if do boolean block detection == true
						navigation.stopMotors();
						//rotate sensor 90 degrees facing front
						navigation.turn(-90);
						navigation.travelForward();
						
						if (tempDistance < 10){ //when it comes close enough to the block to do colorID
							navigation.stopMotors();
							////////////arm down
							
							if (isColorCondition() == true){ //if the color detected is the one we are seeking
								//////////////////arm up
								navigation.backUpTo(0, -y);
								navigation.turn(90);
								///////////////////rotate sensor 90 degrees facing blocks
								navigation.travelTo(7, 3);
								navigation.turn(-90);
								navigation.travelTo(7, 7);
								i++;
								
							} else {
								//arm up
								//rotate sensor 90 degrees facing blocks
								navigation.backUpTo(0, -y);
								navigation.goToPoint(3, 3);
							}
							
						}
					
					}
					
				case SIDE2: //y-axis
					navigation.goToPoint(7, 7);
					if (tempDistance <120){
						// if do boolean block detection == true
						navigation.stopMotors();
						//rotate sensor 90 degrees facing front
						navigation.turn(-90);
						navigation.travelForward();
						
						if (tempDistance < 10){
							navigation.stopMotors();
							//arm down
							
							if (isColorCondition() == true){
								//arm up
								navigation.backUpTo(0, -y);
								navigation.turn(90);
								//rotate sensor 90 degrees facing blocks
								navigation.travelTo(7, 7);		
								i++;
								
							} else {
								//arm up
								//rotate sensor 90 degrees facing blocks
								navigation.backUpTo(0, -y);
								navigation.goToPoint(7, 7);
							}
							
						}
					
					}
				
				case SIDE3: //x-axis on the top
					navigation.goToPoint(3, 7);
					if (tempDistance <120){
						// if do boolean block detection == true
						navigation.stopMotors();
						//rotate sensor 90 degrees facing front
						navigation.turn(-90);
						navigation.travelForward();
						
						if (tempDistance < 10){
							navigation.stopMotors();
							//arm down
							
							if (isColorCondition() == true){
								//arm up
								navigation.backUpTo(0, y);
								navigation.turn(-90);
								//rotate sensor 90 degrees facing blocks
								navigation.travelTo(7, 7);
								i++;
								
							} else {
								//arm up
								//rotate sensor 90 degrees facing blocks
								navigation.backUpTo(0, y);
								navigation.goToPoint(3, 7);
							}
						}
					
					}
					
				case SIDE4 : //y-axis back to starting point
					navigation.goToPoint(3, 3);
					if (tempDistance <120){
						// if do boolean block detection == true
						navigation.stopMotors();
						//rotate sensor 90 degrees facing front
						navigation.turn(-90);
						navigation.travelForward();
						
						if (tempDistance < 10){
							navigation.stopMotors();
							//arm down
							
							if (isColorCondition()){
								//arm up
								navigation.backUpTo(0, y);
								navigation.turn(-90);
								//rotate sensor 90 degrees facing blocks
								navigation.travelTo(3, 7);
								navigation.turn(90);
								navigation.travelTo(7, 7);
								i++;
								
							} else {
								//arm up
								//rotate sensor 90 degrees facing blocks
								navigation.backUpTo(0, y);
								navigation.goToPoint(3, 3);
							}
						}
					
					}

				}
		}
			
			
	}
		

	public boolean isColorCondition() {
		
		if (colorSensor.getColorSeen() == blockWanted){
				Sound.beep();
				flagFound = true;	
		}
		return flagFound;
	}
}



