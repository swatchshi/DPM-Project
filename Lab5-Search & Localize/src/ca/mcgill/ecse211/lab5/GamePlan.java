package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.localizer.*;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
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

	public enum RobotConfig {
		TRACTION, PROPULSION
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
	public static final SensorModes ultraSSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));

	public static final RobotConfig CONFIG = RobotConfig.PROPULSION;

	private enum Direction {
		NORTH, EAST, SOUTH, WEST
	}

	private TextLCD lcd;
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
	 * @param lcd
	 *            TextLCD used for output on the EV3 brick
	 * @throws Exception error with Odometer instances or with data server retreival
	 */
	public GamePlan(TextLCD lcd) throws Exception {
		this.lcd = lcd;
		// US related objects
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SampleProvider us = ultraSSensor.getMode("Distance"); // usDistance provides samples from
																// this instance
		float[] usData = new float[us.sampleSize()]; // usData is the buffer in which data are
		cSensor = new ColorSensor(armSensor);
		lSensor = new ColorSensor(lightSensor);
		ultraSensor = new UltrasonicSensor(us, usData);

		// Odometer related objects
		dynamicTrack = new TrackExpansion();
		odometer = Odometer.getOdometer(leftMotor, rightMotor, dynamicTrack, CONFIG);
		odometryDisplay = new Display(lcd, ultraSensor);
		odoCorrect=new OdometerCorrection(lSensor, odometer);
		
		navigation = new Navigation(odometer, dynamicTrack, CONFIG);
		serverData=new EV3WifiClient();
	}

	/**
	 * Calling for the procedure of the dynamic track adjustment. Calls the maximum
	 * adjustment, then calls the minimum adjustment
	 */
	public void trackAdjust() {
		dynamicTrack.adjustToMax(lcd);
		dynamicTrack.adjustToMin(lcd);
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
	 */
	private void redPlan() {
		// TODO call the procedure for the red team

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
		//TODO look at the team color and check around the bridge
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