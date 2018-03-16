package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.localizer.*;
import ca.mcgill.ecse211.odometer.*;
import lejos.ev3.tools.EV3ConnectionState;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * Class responsible for the game play. This class creates instances of a game,
 * which will make the proper procedure calls and decide the behavior of the
 * robot.
 * 
 * @author Xavier Pellemans
 *
 */
public class GamePlan {

	/**
	 * Enum for the wheel configuration of the robot
	 * TRACTION: wheels are in front of the robot
	 * PROPULSION: wheels are at the back of the robot (motor.backward() is forward)
	 */
	public enum RobotConfig {
		TRACTION, PROPULSION
	}
	
	/**
	 * Enum for the chosen robot
	 * SCREW_DESIGN: design with the expanding track and screw
	 * TANK: design with the tank tracks
	 */
	public enum Robot {
		SCREW_DESIGN, TANK
	}
	

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	public static final EV3MediumRegulatedMotor usSensorMotor = new EV3MediumRegulatedMotor(
			LocalEV3.get().getPort("C"));
	public static final EV3MediumRegulatedMotor trackExpansionMotor = new EV3MediumRegulatedMotor(
			LocalEV3.get().getPort("B"));

	public static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	public static final EV3ColorSensor armSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	public static final EV3GyroSensor gyroSensor = new EV3GyroSensor(LocalEV3.get().getPort("S3"));
	public static final EV3UltrasonicSensor ultraSSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static final RobotConfig CONFIG = RobotConfig.PROPULSION;
	public static final Robot robot = Robot.SCREW_DESIGN;

	/**
	 * Enum describing the cardinal point.
	 * Used to describe the side of a region.
	 * NORTH: side with the highest y
	 * EAST: side with the highest x
	 * SOUTH: side with the lowest y
	 * WEST: side with the lowest x
	 */
	public enum Direction {
		NORTH, EAST, SOUTH, WEST
	}

	private Odometer odometer;
	private Navigation navigation;
	private TrackExpansion dynamicTrack;
	private ColorSensor cSensor;
	private ColorSensor lSensor;
	private UltrasonicSensor ultraSensor;
	private EV3WifiClient serverData;
	private Display odometryDisplay;
	private OdometerCorrection odoCorrect;
	
	
	/**
	 * Creates an object of the GamePlan class. Initializes all instances needed in
	 * the game
	 * 
	 * @throws Exception error with Odometer instances or with data server retreival
	 */
	public GamePlan() throws Exception {
		
		cSensor = new ColorSensor(armSensor);
		lSensor = new ColorSensor(lightSensor);
		ultraSensor = new UltrasonicSensor(ultraSSensor);

		//track related object
		dynamicTrack = new TrackExpansion();
		dynamicTrack.setDesignConstants(robot); //sets the values for the chosen robot
		
		// Odometer related objects
		odometer = Odometer.getOdometer(leftMotor, rightMotor, dynamicTrack, CONFIG);
		odometryDisplay = new Display(lcd, ultraSensor);
		odoCorrect=new OdometerCorrection(lSensor, odometer, dynamicTrack);
		
		navigation = new Navigation(odometer, dynamicTrack, CONFIG);
		//serverData = new EV3WifiClient(); ////////////////////////////////////////////uncomment to enable data retrieval
	}

	/**
	 * Calling for the procedure of the dynamic track adjustment. Calls the maximum
	 * adjustment, then calls the minimum adjustment
	 */
	public void trackAdjust() {
		dynamicTrack.adjustToMax();
		dynamicTrack.adjustToMin();
	}

	
	
	/**
	 * Procedure to determine what Team color plan should be followed
	 * 
	 * @throws Exceptiion Exception thrown if the robot is not playing
	 */
	public void play() throws Exception {
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();
		Thread odoCorrectionThread=new Thread(odoCorrect);
		odoCorrectionThread.start();
		
		switch(serverData.getTeamColor()) {
		case RED:
			redPlan();
			break;
		case GREEN:
			greenPlan();
			break;
		}
		
		//TODO victory tune
	}
	
	

	/**
	 * Game plan of the red team. 
	 * 1- Localize (USLocalizer) 
	 * 2- Localize(LightLocalizer)
	 * 3- Travels to the bridge 
	 * 4- Crosses the bridge 
	 * 5- Search the Green search zone for the OG flag 
	 * 6- Travels to the tunnel 
	 * 7- Crosses the tunnel
	 * 8- finishes in its starting corner
	 * 
	 * @throws Exception When there is a problem with the data from the EV3WifiClass
	 */
	private void redPlan() throws Exception {
		// TODO call the procedure for the red team
		
		
		
		
		
		
		
		
		
		USLocalizer usLoc=new USLocalizer(odometer, navigation, ultraSensor);
		usLoc.doLocalization(serverData.getStartingCorner());
		LightLocalizer lightLoc=new LightLocalizer(navigation, dynamicTrack, lSensor, odometer, CONFIG);
		switch(serverData.getStartingCorner()) {
		case 0:
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		}
		
	}

	/**
	 * Game plan of the green team. 
	 * 1- Localize (USLocalizer) 
	 * 2- Localize(LightLocalizer)
	 * 3- Travels to the tunnel 
	 * 4- Crosses the tunnel 
	 * 5- Search the Red search zone for the OR flag 
	 * 6- Travels to the bridge 
	 * 7- Crosses the bridge
	 * 8- finishes in its starting corner
	 */
	private void greenPlan() {
		// TODO call the procedure for the green team
	}

	/**
	 * Procedure to cross the bridge
	 * 
	 * @return True when the bridge has been crossed
	 */
	private boolean crossBridge() {
		// TODO call the procedure for bridge traversal
		return true;
	}

	/**
	 * Procedure to cross the tunnel
	 * 
	 * @return True when the tunnel has been crossed
	 */
	private boolean crossTunnel() {
		// TODO call the procedure for tunnel traversal
		return true;
	}

	
	/**
	 * Finds the entry side the robot 
	 * has to take in order to cross the bridge
	 * 
	 * @return The Direction (side) which the 
	 * 			robot needs to enter the bridge
	 */
	private Direction getBridgeEntry() {
		//TODO look at the team color and check around the bridge for the right zone
		return Direction.EAST;
	}

	/**
	 * Finds the entry side the robot 
	 * has to take in order to cross the tunnel
	 * 
	 * @return The Direction (side) which the 
	 * 			robot needs to enter the tunnel
	 */
	private Direction getTunnelEntry() {
		//TODO look at the team color and check around the tunnel
		return Direction.EAST;
	}
	
	
	/**
	 * Procedure method to avoid the search zone and water areas 
	 * and go to the specified side of the bridge.
	 * 
	 * @param direction Direction (side) of the bridge entry
	 * @return True when reached
	 */
	private boolean goToBridge(Direction direction) {
		//TODO code the maneuver
		return true;
	}
	
	/**
	 * Procedure method to avoid the search zone and water areas 
	 * and go to the specified side of the tunnel.
	 * 
	 * @param direction Direction (side) of the tunnel entry
	 * @return True when reached
	 */
	private boolean goToTunnel(Direction direction) {
		//TODO code the maneuver
		return true;
	}
	
	/**
	 * Procedure method to avoid the search zone and water areas 
	 * and go to the starting corner.
	 * 
	 * @return True when reached
	 */
	private boolean goToStartingCorner() {
		//TODO code the maneuver
		return true;
	}
}