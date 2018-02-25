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
import lejos.utility.Delay;


public class FlagFinding {
	
	
	private static final int ARM_ROTATION_AMPLITUDE=75;
	private static final int US_ROTATION_AMPLITUDE=90;
	private static final int COLLISION_DISTANCE=10;
	private static final int SUCCESSFUL_BEEPING=3;
	private static final int FAILURE_BEEPING=6;
	private static final int BLOCK_WIDTH = 10;
	private double LLx,LLy,URx,URy;
	private double xRange, yRange;
	private boolean isBlockNear = false;
	private boolean armDown=true;
	private boolean headTurned=false;
	private ColorSensor colorSensor;
	private UltrasonicSensor usSensor;
	private Navigation navigation;
	private Odometer odo;
	private ColorSensor.BlockColor blockWanted;
	private boolean flagFound;
	
	public static enum Side{
		BOTTOM,
		RIGHT,
		TOP,
		LEFT
		
	}
	
	/**
	 * Constructor of the FlagFinding class
	 * @param colorSensor ColorSensor used
	 * @param usSensor UltrasonicSensor used
	 * @param blockWanted ColorSensor.BlockColor of the block wanted
	 * @param LLx (int) X of the lower left corner of the search zone
	 * @param LLy (int) Y of the lower left corner of the search zone
	 * @param URx (int) X of the lower left corner of the search zone
	 * @param URy (int) Y of the lower left corner of the search zone
	 * @throws OdometerExceptions Throws an OdometerExceptions error back to the main if error in instances
	 */
	public FlagFinding(ColorSensor colorSensor, UltrasonicSensor usSensor, ColorSensor.BlockColor blockWanted, int LLx, int LLy, int URx, int URy) throws OdometerExceptions {
		this.colorSensor=colorSensor;
		this.usSensor=usSensor;
		this.navigation=Navigation.getNavigation();
		this.odo=Odometer.getOdometer();
		this.blockWanted=blockWanted;
		this.LLx=LLx*Navigation.TILE_SIZE;
		this.LLy=LLy*Navigation.TILE_SIZE;
		this.URx=URx*Navigation.TILE_SIZE;
		this.URy=URy*Navigation.TILE_SIZE;
		this.xRange=Math.abs(URx-LLx)*Navigation.TILE_SIZE;
		this.yRange=Math.abs(URy-LLy)*Navigation.TILE_SIZE;
	}
	
	
	/**
	 * Method to turn the ultrasonic sensor to the left
	 * @param usSensorTurned true if the ultrasonic sensor should turn to face left
	 * @return true if the ultrasonic sensor faces left
	 */
	public boolean rotateUltrasonicSensor (boolean usSensorTurn) {
		
		if (usSensorTurn && !headTurned) { //
			Lab5.usSensorMotor.rotate(US_ROTATION_AMPLITUDE, true);
			headTurned = true;
		}
		else if(!usSensorTurn&& headTurned) {
			Lab5.usSensorMotor.rotate(-US_ROTATION_AMPLITUDE, true);
			headTurned = false;
		}
		return headTurned;
	}
	
	/**
	 * Method to raise and lower the arm with the color sensor
	 * @param putDown true if the arm should be down
	 * @return true if the arm is down
	 */
	public boolean putArmDown(boolean putDown) {	
		if (putDown && !armDown) { //put arm down and arm is not down yet
			Lab5.armSensorMotor.rotate(ARM_ROTATION_AMPLITUDE, true);
			armDown = true;
		}
		else  if(!putDown && armDown){ //put arm up and arm is not up yet
			Lab5.armSensorMotor.rotate(-ARM_ROTATION_AMPLITUDE, true);
			armDown = false;
		}
		return armDown;
	}
	 
	
	 
	 /**
	  * Looks for the wanted block by sweeping the search area using the ultrasonicsensor 
	  * and performing color detection using the light sensor
	  * Makes the robot circle the search area in a counter clockwise motion
	  * @return true if the block was found
	  */
	 public boolean findBlock() {			
		Sound.beep();
		travelToLowerLeft();
		Sound.beep();
		putArmDown(true);
		int i = 0;
		while (i < Side.values().length){ //if the robot has not detected 4 blocks on one side
			
			Side sideTest=Side.values()[i];
			
			switch (sideTest){
				
				case BOTTOM : //x-axis
					navigation.turnTo(90);
					rotateUltrasonicSensor(true);
					navigation.travelForward();
					if (checkForFlag(yRange+Lab5.TRACK, URx+Lab5.TRACK-odo.getX())){ 
						
						navigation.stopMotors(); //make sure the robot is stopped
						//rotate sensor 90 degrees facing front
						navigation.turnTo(0);
						rotateUltrasonicSensor(false);
						navigation.travelForward(); //travel indefinitely
						
						while (usSensor.readDistance() < COLLISION_DISTANCE){ 
							//when it comes close enough to the block to do colorID
						}
						navigation.stopMotors(); //stop robot
						
						if(isDesiredBlock()) { //checks if it is of the right color
							beepSequence(SUCCESSFUL_BEEPING); //plays the success sequence
							
							//go to UR corner of search zone
							navigation.backUp(odo.getY()-LLy-Lab5.TRACK);
							
							rotateUltrasonicSensor(false);
							travelToUpperRight();
							return true; //return that you found the block
						} else {
							//continue searching
							navigation.backUp(odo.getY()-LLy-Lab5.TRACK);
						}
					}
					if(odo.getX()>=URx+Lab5.TRACK) { //end of BOTTOM side
						i++; //next side
					}
					break;
				case RIGHT: //y-axis
					navigation.turnTo(0);
					rotateUltrasonicSensor(true);
					navigation.travelForward();
					if (checkForFlag(xRange+Lab5.TRACK, URy+Lab5.TRACK - odo.getY())){ 
						
						navigation.stopMotors(); //make sure the robot is stopped
						//rotate sensor 90 degrees facing front
						navigation.turnTo(270);
						rotateUltrasonicSensor(false);
						navigation.travelForward(); //travel indefinitely to go see block
						
						while (usSensor.readDistance() < COLLISION_DISTANCE){ 
							//when it comes close enough to the block to do colorID
						}
						navigation.stopMotors(); //stop robot
						
						if(isDesiredBlock()) { //checks if it is of the right color
							beepSequence(SUCCESSFUL_BEEPING); //plays the success sequence
							
							//go to UR corner of search zone
							navigation.backUp(URx+Lab5.TRACK-odo.getX());
							
							rotateUltrasonicSensor(false);
							travelToUpperRight();
							return true; //return that you found the block
							
						} else {
							//continue searching
							navigation.backUp(URx+Lab5.TRACK-odo.getX());
						}
					}
					if(odo.getY()>=URy+Lab5.TRACK) { //end of RIGHT side
						i++; //next side
					}
					break;
				case TOP: //x-axis on the top
					navigation.turnTo(270);
					rotateUltrasonicSensor(true);
					navigation.travelForward();
					if (checkForFlag(yRange+Lab5.TRACK, odo.getX() - LLx -Lab5.TRACK)){ 
						
						navigation.stopMotors(); //make sure the robot is stopped
						//rotate sensor 90 degrees facing front
						navigation.turnTo(180);
						rotateUltrasonicSensor(false);
						navigation.travelForward(); //travel indefinitely
						
						while (usSensor.readDistance() < COLLISION_DISTANCE){ 
							//when it comes close enough to the block to do colorID
						}
						navigation.stopMotors(); //stop robot
						
						if(isDesiredBlock()) { //checks if it is of the right color
							beepSequence(SUCCESSFUL_BEEPING); //plays the success sequence
							
							//go to UR corner of search zone
							navigation.backUp(LLy+Lab5.TRACK-odo.getY());
							
							rotateUltrasonicSensor(false);
							travelToUpperRight();
							return true; //return that you found the block
							
						} else {
							//continue searching
							navigation.backUp(LLy+Lab5.TRACK-odo.getY());
						}
					}
					if(odo.getX()<=LLx-Lab5.TRACK) { //end of TOP side
						i++; //next side
					}
					break;
				case LEFT : //y-axis back to starting point
					navigation.turnTo(180);
					rotateUltrasonicSensor(true);
					navigation.travelForward();
					if (checkForFlag(xRange+Lab5.TRACK, odo.getY()-LLy-Lab5.TRACK)){ 
						
						navigation.stopMotors(); //make sure the robot is stopped
						//rotate sensor 90 degrees facing front
						navigation.turnTo(90);
						rotateUltrasonicSensor(false);
						navigation.travelForward(); //travel indefinitely
						
						while (usSensor.readDistance() < COLLISION_DISTANCE){ 
							//when it comes close enough to the block to do colorID
						}
						navigation.stopMotors(); //stop robot
						
						if(isDesiredBlock()) { //checks if it is of the right color
							beepSequence(SUCCESSFUL_BEEPING); //plays the success sequence
							
							//go to UR corner of search zone
							navigation.backUp(odo.getX()-LLx-Lab5.TRACK);
							
							rotateUltrasonicSensor(false);
							travelToUpperRight();
							return true; //return that you found the block
							
						} else {
							//continue searching
							navigation.backUp(odo.getX()-LLx-Lab5.TRACK);
						}
					}
					if(odo.getY()<=LLy-Lab5.TRACK) { //end of LEFT side
						i++; //next side
					}
					break;
			}
		}
		
		beepSequence(FAILURE_BEEPING);
		return false;
	}
		

	 /**
	  * Gets if it is the flag is of the desired color
	  * @return (boolean) true if the block color is of the desired color
	  */
	public boolean isDesiredBlock() {
		
		if (colorSensor.getColorSeen() == blockWanted){
				Sound.beep();
				flagFound = true;	
		}
		return flagFound;
	}
	
	/**
	 * check whether there is block 
	 */
	public boolean checkForFlag(double range, double driveRange) {

		boolean blockNear=true;
		
		double distanceFirstBlock=usSensor.readDistance();
		 
		if(distanceFirstBlock <= range) {
			
			
			navigation.stopMotors();
			// the block was seen
			
			// move 1.5 block ahead forward
			navigation.travel(1.5*BLOCK_WIDTH);

			//checks if there is a block ahead which would be hit
			if (usSensor.readDistance() <= distanceFirstBlock) {
				blockNear = true; //block will be hit
			} else {				
				blockNear = false; //no block to be hit
				Sound.beepSequenceUp();
			}
		}
		return (!blockNear); //tells if can go check colored block seen
	}
	
	/**
	 * Makes the EV3 beep a certain amount of beeps
	 * @param numberBeeps number of beeps wanted
	 */
	public void beepSequence(int numberBeeps) {
		for(int j=0; j<numberBeeps;j++) {
			Sound.beep();
		}
	}
	
	/**
	 * Makes the robot travel to the lower left corner of the search zone by going around
	 * Depending on where it is currently
	 * @return true if it has reached the lower left corner
	 */
	public boolean travelToLowerLeft() {
		Side side;
		//determine the side of the of the search zone
		if(odo.getX()<=LLx) {
			side=Side.LEFT;
		}else if(odo.getX()<=URx) {
			if(odo.getY()<=LLy) {
				side=Side.BOTTOM;
			}else if(odo.getY()>=URy){
				side=Side.TOP;
			}else{
				//center of search zone
				navigation.travelTo(LLx-Lab5.TRACK, LLy-Lab5.TRACK);
				return true;
			}
		}else {
			side=Side.RIGHT;
		}
		//goes around the search zone
		switch(side) {
			case LEFT:
			 navigation.travelTo(LLx-Lab5.TRACK, LLy-Lab5.TRACK);
			 break;
			case BOTTOM:
				navigation.travelTo(LLx-Lab5.TRACK, LLy-Lab5.TRACK);
				break;
			case RIGHT:
				navigation.travelTo(URx+Lab5.TRACK, LLy-Lab5.TRACK);
				navigation.travelTo(LLx-Lab5.TRACK, LLy-Lab5.TRACK);
				break;
			case TOP:
				navigation.travelTo(LLx-Lab5.TRACK, URy+Lab5.TRACK);
				navigation.travelTo(LLx-Lab5.TRACK, LLy-Lab5.TRACK);
				break;
		}
		return true;
	}
	
	/**
	 * Makes the robot travel to the upper right corner of the search zone by going around
	 * Depending on where it is currently
	 * @return true if it has reached the upper right corner
	 */
	public boolean travelToUpperRight() {
		Side side;
		//determine the side of where the robot is
		if(odo.getX()<=LLx) {
			side=Side.LEFT;
		}else if(odo.getX()<=URx) {
			if(odo.getY()<=LLy) {
				side=Side.BOTTOM;
			}else if(odo.getY()>=URy){
				side=Side.TOP;
			}else{
				//center of search zone
				navigation.travelTo(URx, URy);
				return true;
			}
		}else {
			side=Side.RIGHT;
		}
		
		//go around search zone
		switch(side) {
			case LEFT:
				navigation.travelTo(LLx-Lab5.TRACK, URy+Lab5.TRACK);
				navigation.travelTo(URx, URy);
				
				break;
			case BOTTOM:
				navigation.travelTo(URx+Lab5.TRACK, LLy-Lab5.TRACK);
				navigation.travelTo(URx, URy);
				break;
			case RIGHT:
				navigation.travelTo(URx, URy);
				break;
			case TOP:
				navigation.travelTo(URx, URy);
				break;
		}
		return true;
	}
}

