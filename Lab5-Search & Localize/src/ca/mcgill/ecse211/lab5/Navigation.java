package ca.mcgill.ecse211.lab5;
import ca.mcgill.ecse211.lab5.*;
import ca.mcgill.ecse211.localizer.*;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Class to navigate to way points
 * @author Xavier Pellemans
 * @author Thomas Bahen
 *
 */
public class Navigation {
	/**
	 * Variables for traveling
	 */
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	public static final double TILE_SIZE = 30.48;
	private Odometer odo;
	private double lastX;
	private double lastY;
	private static boolean navigating=false;
	private static boolean interrupt=false;
	
	private static Navigation nav; //Holds the used instance of this class
	private Lab5.RobotConfig config;
	
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
	 * @param odoCor Thread instance of OdometerCorrection
	 */
	public Navigation(Odometer odo, Lab5.RobotConfig config) {
		this.odo=odo;
		this.config=config;
		nav=this;
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {Lab5.leftMotor, Lab5.rightMotor}) {
			motor.stop();
			motor.setAcceleration(2000);
		}
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
		Lab5.leftMotor.setSpeed(FORWARD_SPEED);
		Lab5.rightMotor.setSpeed(FORWARD_SPEED);
		switch(config) {
			case PROPULSION:
				Lab5.rightMotor.backward();
				Lab5.leftMotor.backward();
				break;
			case TRACTION:
				Lab5.rightMotor.forward();
				Lab5.leftMotor.forward();
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
		Lab5.leftMotor.setSpeed(FORWARD_SPEED);
		Lab5.rightMotor.setSpeed(FORWARD_SPEED);
		switch(config) {
			case TRACTION:
				Lab5.rightMotor.backward();
				Lab5.leftMotor.backward();
				break;
			case PROPULSION:
				Lab5.rightMotor.forward();
				Lab5.leftMotor.forward();
				break;
		}
	}
	
	
	
	/**
	 * Procedure to stop the motors at once
	 */
	public void stopMotors() {
		Lab5.rightMotor.stop(true);
		Lab5.leftMotor.stop(false);
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
		Lab5.leftMotor.setSpeed(ROTATE_SPEED);
	    Lab5.rightMotor.setSpeed(ROTATE_SPEED);
	    Lab5.leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, rotation), true);
	    Lab5.rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, rotation), false);
	    navigating=false;
	}
	
	/**
	 * Rotate continuously in one direction
	 */
	public void rotate(Turn direction) {
		navigating=true;
		Lab5.leftMotor.setSpeed(ROTATE_SPEED);
	    Lab5.rightMotor.setSpeed(ROTATE_SPEED);
	    switch(direction) {
			case CLOCK_WISE:
				Lab5.rightMotor.forward();
				Lab5.leftMotor.backward();
				break;
			case COUNTER_CLOCK_WISE:
				Lab5.rightMotor.backward();
				Lab5.leftMotor.forward();
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
	 * Gets if the navigation is interrupted
	 * 
	 * @return if it is interrupted (boolean)
	 */
	public static boolean getInterrrupt() {
		// TODO Auto-generated method stub
		return interrupt;
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
	   * @return
	   */
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
	
}
