package ca.mcgill.ecse211.lab5;
import ca.mcgill.ecse211.lab5.*;
import ca.mcgill.ecse211.localizer.*;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Class to navigate with the robot on the map
 * Holds every method for wheel control
 * @author Xavier Pellemans
 * @author Thomas Bahen
 *
 */
public class Navigation {
	/**
	 * Variables for traveling
	 */
	public static final int FORWARD_SPEED = 550;
	public static final int LOCALIZATION_SPEED = 400;
	public static final int ROTATE_SPEED = 300;
	public static final int SLOW_ROTATE_SPEED = 100;
	private static final int ACCELERATION = 1000;
	private static final int DECELERATION = 2000;
	public static final double TILE_SIZE = 30.48;
	private Odometer odo;
	private double lastX;
	private double lastY;
	private int forwardSpeed;
	private TrackExpansion dynamicTrack;
	private static boolean navigating=false;
	private static boolean interrupt=false;
	
	private static Navigation nav; //Holds the used instance of this class
	private GamePlan.RobotConfig config;
	
	public static enum Turn{
		CLOCK_WISE, 
		COUNTER_CLOCK_WISE
	}
	
	/**
	 * Constructs a navigation object
	 * @param map : map used
	 * @param odo : Odometer used
	 * @param us : UltrasonicPoller used
	 * @param config The Lab5.RobotConfig, i.e. the wheel positioning
	 */
	public Navigation(Odometer odo, TrackExpansion dynamicTrack, GamePlan.RobotConfig config) {
		this.odo=odo;
		this.dynamicTrack=dynamicTrack;
		this.config=config;
		nav=this;
		setAcceleration(1000);
	}
	
	/**
	 * This method is meant to ensure only one instance of the Navigator is used throughout the code.
	 * 
	 * @return new or existing Navigation Object
	 * @throws OdometerExceptions when no Navigation instance is found
	 */
	public synchronized static Navigation getNavigation() throws OdometerExceptions {	

	    if (nav == null) {
	      throw new OdometerExceptions("No previous Odometer exits.");
	    }
	   // Return existing object
	    return nav;
	}
	
	/**
	 * Sets the acceleration of both traction/propulsion motors
	 * @param acceleration The desired acceleration
	 */
	private void setAcceleration(int acceleration) {
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {GamePlan.leftMotor, GamePlan.rightMotor}) {
			motor.stop();
			motor.setAcceleration(3000);
		}
	}
	
	/**
	 * Travels a certain amount of cm (with coordinates) 
	 * Does not turn when given negative coordinates
	 * 
	 * @param dX absolute displacement in cm
	 * @param dY absolute displacement in cm
	 */
	public void travel(double totalX, double totalY) {
		travel(Math.sqrt(totalX*totalX+totalY*totalY));
	
	}
	
	/**
	 * Travels a certain amount of cm (with distance) 
	 * 
	 * @param travelDistance absolute displacement in cm
	 */
	public void travel(double travelDistance) {
		double dX,dY;
		lastX=odo.getX();
		lastY=odo.getY();
		travelForward();
		
		while(!interrupt)  {
			dX=Math.abs(odo.getX()-lastX);
			dY=Math.abs(odo.getY()-lastY);
			if(dX*dX+dY*dY >= travelDistance*travelDistance) { //reached goal coordinates
				break;
			}
		}
		
		stopMotors(); 
	}
	
	

	/**
	 * Goes to specified coordinate (in grid coordinates)
	 * 
	 * @param x the x line coordinate
	 * @param y the y line coordinate
	 */
	public void goToPoint(double x, double y) {
		travelTo(x*TILE_SIZE, y*TILE_SIZE);
	}
	
	
	/**
	 * Travel to a specific coordinate specified in cm
	 * 
	 * @param x coordinates in x (positive right)
	 * @param y coordinates in y (positive up)
	 */
	public void travelTo(double x, double y) {
		
		lastX=odo.getX();
		lastY=odo.getY();
		double dX= x-lastX;
		double dY= y-lastY;
	     
		turnTo(Math.toDegrees(Math.atan2(dX, dY))); //turn to right direction
		
		travel(dX, dY);
	} 
	
	/**
	 * Travels non stop forward
	 */
	public void travelForward() {
		navigating=true;
		setMotorSpeed(forwardSpeed);
		switch(config) {
			case PROPULSION:
				GamePlan.rightMotor.backward();
				GamePlan.leftMotor.backward();
				break;
			case TRACTION:
				GamePlan.rightMotor.forward();
				GamePlan.leftMotor.forward();
				break;
		} 
	}
	
	/**
	 * Backs up to a specific coordinate specified in cm
	 * 
	 * @param x coordinates in x (positive right)
	 * @param y coordinates in y (positive up)
	 */
	public void backUpTo(double x, double y) {
		double travelDistance=0;
		
		lastX=odo.getX();
		lastY=odo.getY();
		
		double dX= x-lastX;
		double dY= y-lastY;
	     
		turnTo(Math.toDegrees(Math.atan2(dX, dY))+180); //turn to opposite direction
		travelDistance=Math.sqrt(dX*dX+dY*dY);
	     
		
		travelBackward();
				 
		while(!interrupt)  {
			dX= odo.getX()-lastX;
			dY= odo.getY()-lastY;
			if(dX*dX+dY*dY >= travelDistance*travelDistance) { //reached goal coordinates
				break;
			}
		}
		
		stopMotors();
	} 
	
	/**
	 * Backs up a certain amount of cm (with distance) 
	 * 
	 * @param travelDistance absolute displacement in cm
	 */
	public void backUp(double travelDistance) {
		double dX,dY;
		lastX=odo.getX();
		lastY=odo.getY();
		travelBackward();
		
		while(!interrupt)  {
			dX=Math.abs(odo.getX()-lastX);
			dY=Math.abs(odo.getY()-lastY);
			if(dX*dX+dY*dY >= travelDistance*travelDistance) { //reached goal coordinates
				break;
			}
		}
		stopMotors(); 
	}
	
	
	/**
	 * Travels non stop backward
	 */
	public void travelBackward() {
		navigating=true;
		setMotorSpeed(forwardSpeed);
		switch(config) {
			case TRACTION:
				GamePlan.rightMotor.backward();
				GamePlan.leftMotor.backward();
				break;
			case PROPULSION:
				GamePlan.rightMotor.forward();
				GamePlan.leftMotor.forward();
				break;
		}
	}
	
	
	
	/**
	 * Procedure to stop the motors at once
	 */
	public void stopMotors() {
		GamePlan.rightMotor.stop(true);
		GamePlan.leftMotor.stop(false);
		navigating=false;
	}
	
		
	
	/**
	 * Turns to specified angle (in degrees)
	 * 
	 * @param theta angles in degrees
	 */
	public void turnTo(double theta) {
	    
		double actualTheta=odo.getTheta();
		double rotation=(theta-actualTheta);
		if( rotation > 180) {
			rotation=rotation-360;
		}
		else if(rotation < -180) {
			rotation=rotation+360;
		}	
		else if(Math.abs(rotation)==180){
		   rotation= Math.abs(rotation);
		}
		turn(rotation);
	}
	
	/**
	 * Turn by a certain amount of degrees (defined clockwise)
	 * 
	 * @param rotation clockwise in degrees
	 */
	public void turn(double rotation) {
		navigating=true;
		setMotorSpeed(ROTATE_SPEED);
	    GamePlan.leftMotor.rotate(-convertAngle(dynamicTrack.getWheelRad(), dynamicTrack.getTrack(), rotation), true);
	    GamePlan.rightMotor.rotate(convertAngle(dynamicTrack.getWheelRad(), dynamicTrack.getTrack(), rotation), false);
	    navigating=false;
	}
	
	/**
	 * Rotate continuously in one direction
	 * 
	 * @param direction Direction which the robot needs to turn 
	 * (CLOCK_WISE, COUNTER_CLOCK_WISE)
	 * 
	 */
	public void rotate(Turn direction) {
		navigating=true;
		setMotorSpeed(ROTATE_SPEED);
	    switch(direction) {
			case CLOCK_WISE:
				GamePlan.rightMotor.forward();
				GamePlan.leftMotor.backward();
				break;
			case COUNTER_CLOCK_WISE:
				GamePlan.rightMotor.backward();
				GamePlan.leftMotor.forward();
				break;
	    }
		navigating=false;
	}
	
	
	/**
	 * Gets if the robot is moving towards a way point
	 * 
	 * @return true if the robot is heaading to a way point
	 */
	public boolean isNavigating() {
		return navigating;
	}
	
	
	
	/**
	 * Gets if interrupted navigation
	 * 
	 * @return if navigation is interrupted
	 */
	public boolean getInterrupt() {
		return interrupt;
	}
	
	/**
	 * Gets the last x coordinate (where the robot came from)
	 * 
	 * @return last x coordinate
	 */
	public double getLastX() {
		return lastX;
	}
	
	/**
	 * Gets the last y coordinate (where the robot came from)
	 * 
	 * @return last y coordinate
	 */
	public double getLastY() {
		return lastY;
	}
	
	
	 /**
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return rotation in degrees
	   */
	 public static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	 }

	 /**
	  * Converts robot angle rotation into wheel rotations
	  * 
	  * @param radius of the wheels
	  * @param width of the wheel base
	  * @param angle rotation of the robot
	  * @return the wheel rotation needed to make the robot turn
	  */
	 public static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	 }
	
	 /**
	  * Gets the instance of the odometer
	  * 
	  * @return the used odometer
	  */
	 private Odometer getUsedOdometer() {
		 return odo;
	 }

	/**
	 * Sets the navigating state
	 * 
	 * @param navigate : boolean if the robot is navigating
	 */
	public static void setNavigating(boolean navigate) {
		navigating=navigate;
	}

	/**
	 * Sets the interrupt variable
	 * 
	 * @param interrupt boolean of if interrupted
	 */
	public static void setInterrupt(boolean interrupt) {
	   Navigation.interrupt=interrupt;
	}
	
	/**
	 * Sets the speed of the motors
	 * 
	 * @param speed The desired speed
	 */
	public void setMotorSpeed(int speed) {
		GamePlan.leftMotor.setSpeed(speed);
	    GamePlan.rightMotor.setSpeed(speed);
	}
	
	/**
	 * Sets the forward speed value
	 * @param forwardSpeed
	 */
	public void setForwardSpeed(int forwardSpeed) {
		this.forwardSpeed = forwardSpeed;
	}
	
}
