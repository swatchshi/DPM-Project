package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.EV3WifiClient.CoordParameter;
import ca.mcgill.ecse211.lab5.EV3WifiClient.Zone;
import ca.mcgill.ecse211.lab5.Navigation.Turn;
import ca.mcgill.ecse211.localizer.*;
import ca.mcgill.ecse211.localizer.ColorSensor.BlockColor;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;
import ca.mcgill.ecse211.WiFiClient.*;

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
	 * Enum for the wheel configuration of the robot TRACTION: wheels are in front
	 * of the robot PROPULSION: wheels are at the back of the robot
	 * (motor.backward() is forward)
	 */
	public enum RobotConfig {
		TRACTION, PROPULSION
	}

	/**
	 * Enum for the chosen robot SCREW_DESIGN: design with the expanding track and
	 * screw TANK: design with the tank tracks
	 */
	public enum Robot {
		SCREW_DESIGN, TANK
	}

	// Motor Objects, and Robot related parameters
	/**
	 * Left motor of the wheel base in PORT D
	 */
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	/**
	 * Right motor of the wheel base in PORT A
	 */
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	/**
	 * Motor of the Ultrasonic Sensor in PORT C
	 */
	public static final EV3MediumRegulatedMotor usSensorMotor = new EV3MediumRegulatedMotor(
			LocalEV3.get().getPort("C"));
	/**
	 * Light sensor under the robot in PORT 1
	 */
	public static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	/**
	 * Light sensor at the front of the robot in PORT 2
	 */
	public static final EV3ColorSensor armSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	/**
	 * Gyroscope on top of the robot in PORT 3
	 */
	public static final EV3GyroSensor gyroSensor = new EV3GyroSensor(LocalEV3.get().getPort("S3"));
	/**
	 * Ultrasonic sensor in front of the robot in PORT 4
	 */
	public static final EV3UltrasonicSensor ultraSSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
	/**
	 * Screen of the EV3
	 */
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	/**
	 * Configuration of the wheel base of the robot
	 */
	public static final RobotConfig CONFIG = RobotConfig.PROPULSION;
	/**
	 * Type of the robot used
	 */
	public static final Robot robot = Robot.TANK;
	

	/**
	 * Enum describing the cardinal point. Used to describe the side of a region.
	 * NORTH: side with the highest y EAST: side with the highest x SOUTH: side with
	 * the lowest y WEST: side with the lowest x CENTER: in the middle
	 */
	public enum Direction {
		NORTH, EAST, SOUTH, WEST, CENTER
	}

	/**
	 * Object to help with where the robot is
	 */
	private Odometer odometer;
	/**
	 * Object to help with how the robot can move
	 */
	private Navigation navigation;
	/**
	 * Object to help with traction values
	 */
	private TrackExpansion dynamicTrack;
	/**
	 * Object to help with how the Light sensor under the robot detects lines
	 */
	private ColorSensor lSensor;
	/**
	 * Object to help with how the Ultrasonic sensor at the head of the robot
	 * detects distances
	 */
	private UltrasonicSensor ultraSensor;
	/**
	 * Object to help with how the Gyroscope sensor on top of the robot detects
	 * angle changes
	 */
	private Gyroscope gyroscope;
	/**
	 * Object in charge of all variables from the game
	 */
	private EV3WifiClient serverData;
	
	/**
	 * Object in charge of correcting the trajectory
	 */
	private OdometerCorrection odoCorrect;
	/**
	 * Object in charge of the initial wall angle localization
	 */
	private USLocalizer usLoc;
	/**
	 * Object in charge of localizing at crossings and crash localizations
	 */
	private LightLocalizer lightLoc;
	/**
	 * Object in charge of keeping track of the time passed
	 */
	private InternalClock internalClock;
	
	/**
	 * Creates an object of the GamePlan class. Initializes all instances needed in
	 * the game
	 * 
	 * @throws Exception
	 *             error with Odometer instances or with data server retrieval
	 */
	public GamePlan() throws Exception {

		lSensor = new ColorSensor(lightSensor);
		ultraSensor = new UltrasonicSensor(ultraSSensor);
		gyroscope = new Gyroscope(gyroSensor);
		// track related object
		dynamicTrack = new TrackExpansion();
		dynamicTrack.setDesignConstants(robot); // sets the values for the chosen robot

		// Odometer related objects
		odometer = Odometer.getOdometer(leftMotor, rightMotor, dynamicTrack, gyroscope, CONFIG);
		//odometryDisplay = new Display(lcd, ultraSensor, gyroscope);
		odoCorrect = new OdometerCorrection(lSensor, odometer, dynamicTrack);
		navigation = new Navigation(odometer, dynamicTrack, CONFIG);
		//procedure objects
		usLoc = new USLocalizer(odometer, navigation, ultraSensor);
		lightLoc = new LightLocalizer(navigation, dynamicTrack, lSensor, odometer, gyroscope);
		internalClock=new InternalClock();
		
		
		Thread odoThread = new Thread(odometer);
		
		odoCorrect.setDoCorrection(false);
		odometer.setDoThetaCorrection(false);
		odometer.setEnablePrint(true);
		serverData = new EV3WifiClient(); //////////////////////////////////////////// uncomment to enable data
											//////////////////////////////////////////// retrieval
		odoThread.start();
	}

	/**
	 * Calling for the procedure of the dynamic track adjustment for the
	 * SCREW_DESIGN. Calls the maximum adjustment, then calls the minimum adjustment
	 */
	public void trackAdjust() {
		dynamicTrack.adjustToMax();
		dynamicTrack.adjustToMin();
	}
	
	/**
	 * Calculates the track experimentally by detecting 5 lines of a crossing
	 * and calculating the track needed to do a 360 turn (going back to the same line)
	 * Robot has to be centered on top of a crossing.
	 * The calculated track will be displayed on the screen.
	 * The method will then wait for user input to finish its procedure.
	 */
	public void calculateTrack() {
		int lineCount=5, i=0;
		int leftMotorLastTachoCount=0, rightMotorLastTachoCount=0,leftMotorTachoCount, rightMotorTachoCount;
	    double distL, distR, theta, track;
	    navigation.setRotateSpeed(Navigation.SLOW_ROTATE_SPEED);
	    navigation.rotate(Turn.CLOCK_WISE);
	    while(i<lineCount) {
	    	if(lSensor.lineDetected()) {
	    		i++;
	    	}
	    	if(i==1) {
	    		leftMotorLastTachoCount = leftMotor.getTachoCount();
	    		rightMotorLastTachoCount = rightMotor.getTachoCount();
	    	}
	    }
	    navigation.stopMotors();
	    leftMotorTachoCount = leftMotor.getTachoCount();
	    rightMotorTachoCount = rightMotor.getTachoCount();
		      
		distL=Math.PI*dynamicTrack.getWheelRad()*(leftMotorTachoCount-leftMotorLastTachoCount)/180; 	//convert left rotation to wheel displacement
		distR=Math.PI*dynamicTrack.getWheelRad()*(rightMotorTachoCount-rightMotorLastTachoCount)/180;	//convert right rotation to wheel displacement
		      
		track=(distR-distL)/(2*Math.PI); //Calculating the instantaneous rotation magnitude
		lcd.drawString("Track: "+ track, 0, 6);
		Button.waitForAnyPress();
		lcd.clear();
		navigation.setRotateSpeed(Navigation.ROTATE_SPEED);
	}
	
	
	/**
	 * Display the colors seen by the color sensor
	 * until the escape button is pressed.
	 * Press any other button to display the next color
	 */
	public void colorTest() {
		lcd.clear();
		/*int buttonID;
		do {
			lcd.drawString(cSensor.getColorSeen().toString(), 0, 0);
			buttonID=Button.waitForAnyPress();
		}while(buttonID!=Button.ID_ESCAPE);
		lcd.clear();
		odometryDisplay.setEnablePrint(true);*/
	}
	
	/**
	 * Drives the robot in squares 
	 * Waits for user input after
	 * Continues of user presses any button but the enter button
	 * 
	 * @param tiles Number of tiles per side
	 * @return True when the user ends the procedure
	 */
	public boolean squareDrive(int tiles) {
		int buttonID=0;
		do {
			navigation.travel(tiles*Navigation.TILE_SIZE);
			navigation.turn(90);
			navigation.travel(tiles*Navigation.TILE_SIZE);
			navigation.turn(90);
			navigation.travel(tiles*Navigation.TILE_SIZE);
			navigation.turn(90);
			navigation.travel(tiles*Navigation.TILE_SIZE);
			navigation.turn(90);
			buttonID=Button.waitForAnyPress();
			if(buttonID==Button.ID_UP) {
				dynamicTrack.setTrack(dynamicTrack.getTrack()+0.05);
			}else if(buttonID==Button.ID_DOWN) {
				dynamicTrack.setTrack(dynamicTrack.getTrack()-0.05);
			}
		}while(buttonID!=Button.ID_ENTER);
		System.exit(0);
		return true;
	}
	
	

	/**
	 * Procedure to determine what Team color plan should be followed
	 * 
	 * @throws Exception
	 *             Exception thrown if the robot is not playing
	 */
	public void play() throws Exception {
		
				
		internalClock.startClock();
		
		
		switch (serverData.getTeamColor()) {
		case RED:
			redPlan();
			break;
		case GREEN:
			greenPlan();
			break;
		}
		
	}

	
	
	/**
	 * Gets the direction opposite to the one specified
	 * 
	 * @param direction (North, South, East, West, Center)
	 * @return The opposite direction
	 */
	public Direction directionSwitch(Direction direction) {
		switch(direction) {
		case CENTER:
			return Direction.CENTER;
		case EAST:
			return Direction.WEST;
		case WEST: 
			return Direction.EAST;
		case NORTH:
			return Direction.SOUTH;
		case SOUTH:
			return Direction.NORTH;
		default:
			return Direction.CENTER;
		}
	}
	
	
	/**
	 * Game plan of the red team. 1- Localize (USLocalizer) 2-
	 * Localize(LightLocalizer) 3- Travels to the bridge 4- Crosses the bridge 5-
	 * Search the Green search zone for the OG flag 6- Travels to the tunnel 7-
	 * Crosses the tunnel 8- finishes in its starting corner
	 * 
	 * @throws Exception
	 *             When there is a problem with the data from the EV3WifiClass
	 */
	private void redPlan() throws Exception {
		
		//Localizing at the corner
				navigation.setForwardSpeed(Navigation.LOCALIZATION_SPEED);
				
				usLoc.doLocalization(); 
				usLoc=null;
				
				Sound.beepSequenceUp();
				
				//Light localizing at the closest crossing
				navigation.setEnableGyroscopeCorrection(true);
				lightLoc.crashLocalizer(serverData.getStartingCorner());
				
				//set parameters
				Sound.beepSequenceUp();
				Thread odoCorrectionThread = new Thread(odoCorrect);
				odoCorrectionThread.start();
				navigation.setForwardSpeed(Navigation.FORWARD_SPEED);
				odoCorrect.setDoCorrection(true);
				navigation.setEnableGyroscopeCorrection(true);
				
				//Going to the bridge
				goToBridge(getBridgeEntry());
				crossBridge();
				Sound.beepSequenceUp();
				
				
				//find the flag in the red search zone
				/*
				(new FlagFinding(dynamicTrack, new ColorSensor(armSensor), ultraSensor, internalClock)).findBlock(getSearchStartSide(Zone.SR, directionSwitch(getTunnelEntry())),
						serverData.getCoordParam(CoordParameter.SR_LL_x), serverData.getCoordParam(CoordParameter.SR_LL_y),
						serverData.getCoordParam(CoordParameter.SR_UR_x), serverData.getCoordParam(CoordParameter.SR_UR_y), 
						serverData.getFlagColor());
				*/
				
				
				
				//Going to the tunnel
				goToTunnel(getTunnelEntry());
				Sound.beepSequenceUp();
				crossTunnel();
				Sound.beepSequenceUp();
				
				//finish
				goToStartingCorner();
				Sound.beepSequenceUp();
	}

	/**
	 * Game plan of the green team. 1- Localize (USLocalizer) 2-
	 * Localize(LightLocalizer) 3- Travels to the tunnel 4- Crosses the tunnel 5-
	 * Search the Red search zone for the OR flag 6- Travels to the bridge 7-
	 * Crosses the bridge 8- finishes in its starting corner
	 * 
	 * @throws Exception
	 *             When there is a problem with the data from the EV3WifiClass
	 */
	private void greenPlan() throws Exception {
		
		
		
		
		//Localizing at the corner
		navigation.setForwardSpeed(Navigation.LOCALIZATION_SPEED);
		
		usLoc.doLocalization(); 
		usLoc=null;
		
		Sound.beepSequenceUp();
		
		//Light localizing at the closest crossing
		navigation.setEnableGyroscopeCorrection(true);
		lightLoc.crashLocalizer(serverData.getStartingCorner());
		
		//set parameters
		Sound.beepSequenceUp();
		Thread odoCorrectionThread = new Thread(odoCorrect);
		odoCorrectionThread.start();
		navigation.setForwardSpeed(Navigation.FORWARD_SPEED);
		odoCorrect.setDoCorrection(true);
		navigation.setEnableGyroscopeCorrection(true);
		
		//Going to the tunnel
		goToTunnel(getTunnelEntry());
		Sound.beepSequenceUp();
		crossTunnel();
		Sound.beepSequenceUp();
		
		
		//find the flag in the red search zone
		/*
		(new FlagFinding(dynamicTrack, new ColorSensor(armSensor), ultraSensor, internalClock)).findBlock(getSearchStartSide(Zone.SR, directionSwitch(getTunnelEntry())),
				serverData.getCoordParam(CoordParameter.SR_LL_x), serverData.getCoordParam(CoordParameter.SR_LL_y),
				serverData.getCoordParam(CoordParameter.SR_UR_x), serverData.getCoordParam(CoordParameter.SR_UR_y), 
				serverData.getFlagColor());
		*/
		
		//Going to the bridge
		goToBridge(getBridgeEntry());
		Sound.beepSequenceUp();
		crossBridge();
		Sound.beepSequenceUp();
		
		//finish 
		goToStartingCorner();
		Sound.beepSequenceUp();
		
		
	}

	
	/**
	 * Procedure to cross the bridge
	 * 
	 * @return True when the bridge has been crossed
	 * @throws Exception
	 *             if the specified entry point is incorrect
	 */
	private boolean crossBridge() throws Exception { // expansion method, travel directly
		odoCorrect.setDoCorrection(false);
		navigation.travel(navigation.TILE_SIZE * (1 + serverData.getBridgeWidth(getBridgeEntry())));
		// travels the width of the bridge plus an extra tile
		odoCorrect.setDoCorrection(true);
		return true;
	}

	/**
	 * Procedure to cross the tunnel
	 * 
	 * @return True when the tunnel has been crossed
	 * @throws Exception
	 *             if the specified entry point is incorrect
	 */
	private boolean crossTunnel() throws Exception {
		odoCorrect.setDoCorrection(false);
		navigation.travel(navigation.TILE_SIZE * (1 + serverData.getTunnelWidth(getTunnelEntry())));
		// travels the width of the tunnel plus an extra tile
		odoCorrect.setDoCorrection(true);
		return true;
	}

	/**
	 * Finds the entry side the robot has to take in order to cross the bridge.
	 * Returns Direction.CENTER if unable to find the entrance.
	 * 
	 * @return The Direction (side) which the robot needs to enter the bridge
	 * 
	 * @throws Exception
	 *             Exception thrown if the robot is not playing or a problem with the zone
	 */
	private Direction getBridgeEntry() throws Exception {
		//Saving coordinates of the bridge to local variables to make the code more readable
		double lowerLeftX = serverData.getCoordParam(CoordParameter.BR_LL_x) * Navigation.TILE_SIZE;
		double lowerLeftY = serverData.getCoordParam(CoordParameter.BR_LL_y) * Navigation.TILE_SIZE;
		double upperRightX = serverData.getCoordParam(CoordParameter.BR_UR_x) * Navigation.TILE_SIZE;
		double upperRightY = serverData.getCoordParam(CoordParameter.BR_UR_y) * Navigation.TILE_SIZE;
		
		//Algorithm works for any bridge length
		//Looks North of the bridge, to see what zone is there
		//The robot has to enter the bridge from the RED zone no matter what its team color
		switch(serverData.getZone(lowerLeftX + (upperRightX-lowerLeftX)/2, upperRightY+Navigation.TILE_SIZE/2)) {
		case BRIDGE:
		case TUNNEL:
			//Cases where the getZone method gives nonsense
			throw new Exception("Error in the zone for bridge entry");
		case GREEN:
		case SG:
			//Green zone is North of the bridge
			//so entry in RED zone is in the south because GREEN entry is in the North
			return Direction.SOUTH;
		case RED:
		case SR:
			//Red zone is North of the bridge
			//entry in RED zone is in the North
			return Direction.NORTH;
		case WATER:
			//Water is North of the bridge
			//look EAST of the bridge, to see what zone is there
			switch(serverData.getZone(upperRightX+Navigation.TILE_SIZE/2, lowerLeftY+ (upperRightY-lowerLeftY)/2)) {
			case BRIDGE:
			case TUNNEL:
			case WATER:
				//Cases where getZone gives nonsense
				throw new Exception("Error in the zone for bridge entry");
			case GREEN:
			case SG:
				//Green zone is East of bridge
				//so RED entry point is in the West
				return Direction.WEST;
			case RED:
			case SR:
				//Red zone is East of bridge
				//so RED entry point is in the East
				return Direction.EAST;
			}
			break;
		}
		//default
		return Direction.CENTER;
	}

	/**
	 * Finds the entry side the robot has to take in order to cross the tunnel
	 * 
	 * @return The Direction (side) which the robot needs to enter the tunnel
	 * @throws Exception
	 *             When there is a problem with the data from the EV3WifiClass
	 */
	private Direction getTunnelEntry() throws Exception {
		//Saving coordinates of the tunnel to local variables to make the code more readable
		double lowerLeftX = serverData.getCoordParam(CoordParameter.TN_LL_x) * Navigation.TILE_SIZE;
		double lowerLeftY = serverData.getCoordParam(CoordParameter.TN_LL_y) * Navigation.TILE_SIZE;
		double upperRightX = serverData.getCoordParam(CoordParameter.TN_UR_x) * Navigation.TILE_SIZE;
		double upperRightY = serverData.getCoordParam(CoordParameter.TN_UR_y) * Navigation.TILE_SIZE;
		
		//Algorithm works for any tunnel length
		//Looks North of the tunnel, to see what zone is there
		//The robot has to enter the tunnel from the GREEN zone no matter what its team color
		switch(serverData.getZone(lowerLeftX + (upperRightX-lowerLeftX)/2, upperRightY+Navigation.TILE_SIZE/2)) {
		case BRIDGE:
		case TUNNEL:
			//cases where getZone gives nonsense
			throw new Exception("Error in the zone for tunnel entry");
		case GREEN:
		case SG:
			//Green zone is North of the tunnel
			//entry of tunnel is in the North 
			return Direction.NORTH;
		case RED:
		case SR:
			//Red zone is in the North
			//entry of tunnel (in GREEN zone) is in the South 
			return Direction.SOUTH;
		case WATER:
			//Water is North of the tunnel
			//look EAST of the tunnel, to see what zone is there
			switch(serverData.getZone(upperRightX+Navigation.TILE_SIZE/2, lowerLeftY+ (upperRightY-lowerLeftY)/2)) {
			case BRIDGE:
			case TUNNEL:
			case WATER:
				//cases where getZone gives nonsense
				throw new Exception("Error in the zone for tunnel entry");
			case GREEN:
			case SG:
				//Green zone is east of the tunnel
				//EAST entry is the tunnel entry point
				return Direction.EAST;
			case RED:
			case SR:
				//Red zone is east of the tunnel
				//West entry is the tunnel entry point
				return Direction.WEST;
			}
			break;
		}
		//default
		return Direction.CENTER;
	}
	
	/**
	 * Method to get the side of the search zone which will
	 * make the robot travel the least distance to begin the 
	 * flag finding algorithm.
	 * Because the robot starts its search in the left corner of a side
	 * (WEST: upper left
	 *  SOUTH: lower left
	 *  EAST: lower right
	 *  NORTH: upper right)
	 *  And does the algorithm by going around couter-clockwise
	 *  Therefore, this method prevents the robot from doing
	 *  a detour to start the flag finding algorithm by specifying
	 *  the side of the nearest corner of the search zone.
	 *  
	 * @param searchZone The search zone in question
	 * @param exit 
	 * 			The Direction of the exit point of the water feature the robot just
	 * 			finished crossing before the search. (tip: use directionSwitch
	 * 			ex: directionSwitch(getTunnelEntry()) to get exit point of the tunnel)
	 * @return The side where the robot should start the search
	 */
	private Direction getSearchStartSide(Zone searchZone, Direction exit) {
		Direction flagSearchStartSide;
		//looks first at where the robot will exit the water feature on the map
		switch(exit) {
		case NORTH:
			//looks where the robot is with respect to the search zone desired
			switch(serverData.getSide(searchZone, odometer.getX(), odometer.getY())) {
			case CENTER:
			case WEST:
				flagSearchStartSide=Direction.SOUTH;
				break;
			default:
				flagSearchStartSide=serverData.getSide(searchZone, odometer.getX(), odometer.getY());
				break;
			}
			break;
		case EAST:
			//looks where the robot is with respect to the search zone desired
			switch(serverData.getSide(searchZone, odometer.getX(), odometer.getY())) {
			case CENTER:
			case NORTH:
				flagSearchStartSide=Direction.WEST;
				break;
			default:
				flagSearchStartSide=serverData.getSide(searchZone, odometer.getX(), odometer.getY());
				break;
			}
			break;
		case SOUTH:
			//looks where the robot is with respect to the search zone desired
			switch(serverData.getSide(searchZone, odometer.getX(), odometer.getY())) {
			case CENTER:
			case EAST:
				flagSearchStartSide=Direction.NORTH;
				break;
			default:
				flagSearchStartSide=serverData.getSide(searchZone, odometer.getX(), odometer.getY());
				break;
			}
			break;
		case WEST:
			//looks where the robot is with respect to the search zone desired
			switch(serverData.getSide(searchZone, odometer.getX(), odometer.getY())) {
			case CENTER:
			case SOUTH:
				flagSearchStartSide=Direction.EAST;
				break;
			default:
				flagSearchStartSide=serverData.getSide(searchZone, odometer.getX(), odometer.getY());
				break;
			}
			break;
		default: flagSearchStartSide=Direction.SOUTH;
		}
		return flagSearchStartSide;
	}
	
	/**
	 * Method for localizing before crossing the bridge
	 * Also goes to the bridge entry after doing the localization
	 * 
	 * @param direction Entry direction of the bridge
	 * @throws Exception if unable to retrieve the bridge data
	 */
	private void localizeBeforeBridge(Direction direction) throws Exception {
		//Saving coordinates of the bridge to local variables to make the code more readable
		int lowerLeftXLine = serverData.getCoordParam(CoordParameter.BR_LL_x);
		int lowerLeftYLine = serverData.getCoordParam(CoordParameter.BR_LL_y);
		int upperRightXLine = serverData.getCoordParam(CoordParameter.BR_UR_x);
		int upperRightYLine = serverData.getCoordParam(CoordParameter.BR_UR_y);
		double first, second;
		switch(direction) {
		case NORTH:
			
			odoCorrect.setDoCorrection(true);
			
			navigation.travelTo((lowerLeftXLine+0.45)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE);
			navigation.turnTo(270); //look west
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			first=odometer.getX();
			navigation.stopMotors();
			
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			second=odometer.getX();
			navigation.stopMotors();
			navigation.travel((first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(180);
			
			
			break;
		case SOUTH:
			//Entry of tunnel is in the South part of the tunnel
			odoCorrect.setDoCorrection(true);
			
			navigation.travelTo((lowerLeftXLine+0.45)*Navigation.TILE_SIZE, (lowerLeftYLine-0.5)*Navigation.TILE_SIZE);
			navigation.turnTo(270); //look west
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			first=odometer.getX();
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			second=odometer.getX();
			navigation.travel((first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(0);
			break;
		case EAST:
			//Entry of tunnel is in the East part of the tunnel
			odoCorrect.setDoCorrection(true);
			
			navigation.travelTo((upperRightXLine+0.5)*Navigation.TILE_SIZE, (upperRightYLine-0.45)*Navigation.TILE_SIZE);
			navigation.turnTo(0); //look north
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			first=odometer.getY();
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			second=odometer.getY();
			navigation.travel((first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(270);
			break;
		case WEST:
			//Entry of tunnel is in the West part of the tunnel
			odoCorrect.setDoCorrection(true);
			
			navigation.travelTo((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (lowerLeftYLine+0.55)*Navigation.TILE_SIZE);
			navigation.turnTo(0); //look north
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			first=odometer.getY();
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			second=odometer.getY();
			navigation.travel((first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(90);
			break;
		case CENTER:
			//do nothing
			break;
		}
	}
	
	/**
	 * Method for localizing before crossing the tunnel
	 * Also goes to the tunnel entry after doing the localization
	 * 
	 * @param direction Entry direction of the tunnel
	 * @throws Exception if unable to retrieve the tunnel data
	 */
	private void localizeBeforeTunnel(Direction direction) throws Exception {
		//Saving coordinates of the bridge to local variables to make the code more readable
		int lowerLeftXLine = serverData.getCoordParam(CoordParameter.TN_LL_x);
		int lowerLeftYLine = serverData.getCoordParam(CoordParameter.TN_LL_y);
		int upperRightXLine = serverData.getCoordParam(CoordParameter.TN_UR_x);
		int upperRightYLine = serverData.getCoordParam(CoordParameter.TN_UR_y);
		double first, second;
		switch(direction) {
		case NORTH:
			
			odoCorrect.setDoCorrection(true);
			
			navigation.travelTo((lowerLeftXLine+0.45)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE);
			navigation.turnTo(270); //look west
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			first=odometer.getX();
			navigation.stopMotors();
			
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			second=odometer.getX();
			navigation.stopMotors();
			navigation.travel((first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(180);
			
			
			break;
		case SOUTH:
			//Entry of tunnel is in the South part of the tunnel
			odoCorrect.setDoCorrection(true);
			
			navigation.travelTo((lowerLeftXLine+0.45)*Navigation.TILE_SIZE, (lowerLeftYLine-0.5)*Navigation.TILE_SIZE);
			navigation.turnTo(270); //look west
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			first=odometer.getX();
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			second=odometer.getX();
			navigation.travel((first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(0);
			break;
		case EAST:
			//Entry of tunnel is in the East part of the tunnel
			odoCorrect.setDoCorrection(true);
			
			navigation.travelTo((upperRightXLine+0.5)*Navigation.TILE_SIZE, (upperRightYLine-0.45)*Navigation.TILE_SIZE);
			navigation.turnTo(0); //look north
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			first=odometer.getY();
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			second=odometer.getY();
			navigation.travel((first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(270);
			break;
		case WEST:
			//Entry of tunnel is in the West part of the tunnel
			odoCorrect.setDoCorrection(true);
			
			navigation.travelTo((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (lowerLeftYLine+0.55)*Navigation.TILE_SIZE);
			navigation.turnTo(0); //look north
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			first=odometer.getY();
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			second=odometer.getY();
			navigation.travel((first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(90);
			break;
		case CENTER:
			//do nothing
			break;
		}
	}
	

	/**
	 * Procedure method to avoid the search zone and water areas and go to the
	 * specified side of the bridge.
	 * 
	 * @param direction
	 *            Direction (side) of the bridge entry
	 * @throws Exception
	 *             When there is a problem with the data from the EV3WifiClass or
	 *             with the entry point
	 */
	private void goToBridge(Direction direction) throws Exception {
		//Saving coordinates of the bridge to local variables to make the code more readable
		double lowerLeftX = serverData.getCoordParam(CoordParameter.BR_LL_x) * Navigation.TILE_SIZE;
		double lowerLeftY = serverData.getCoordParam(CoordParameter.BR_LL_y) * Navigation.TILE_SIZE;
		double upperRightX = serverData.getCoordParam(CoordParameter.BR_UR_x) * Navigation.TILE_SIZE;
		double upperRightY = serverData.getCoordParam(CoordParameter.BR_UR_y) * Navigation.TILE_SIZE;
		
		//Looks at what is the entry side of the bridge
		switch (direction) {
		// Entrance is in the North
		case NORTH:
			// entrance of the bridge is on the North side
			// avoid SR
			// look where the North entrance is relative to the SR zone
			switch (serverData.getSide(EV3WifiClient.Zone.SR, lowerLeftX + (upperRightX - lowerLeftX) / 2,
					upperRightY)) {
			case CENTER:
				//Bridge entry in SR zone
				throw new Exception("Bridge entry in search zone");
			case NORTH:
				// the search zone is south of the north bridge entry
				// very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to bridge entrance");
			case SOUTH:
				// the search zone is north of the north bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is north of the north bridge entry
					// goes south around by the west bound
					goToSRUpperLeft();
					goToSRLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the north bridge entry
					// rush for it
					break;
				case EAST:
					// robot is east of the search zone, which is north of the north bridge entry
					// go south by the east bound
					goToSRLowerRight();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the north bridge entry
					// go south by the west bound
					goToSRLowerLeft();
					break;
				}
				break;
			case EAST:
				// the search zone is west of the north bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is west of the north bridge entry
					// goes south around by the east bound
					goToSRUpperRight();
					goToSRLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the north bridge entry
					// go west
					goToSRLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the north bridge entry
					// rush for it
					break;
				case WEST:
					// robot is west of the search zone, which is west of the north bridge entry
					// go south by the west bound and then east
					goToSRLowerLeft();
					goToSRLowerRight();
					break;
				}
				break;
			case WEST:
				// the search zone is east of the north bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is east of the north bridge entry
					// goes south around by the west bound
					goToSRUpperLeft();
					goToSRLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the north bridge entry
					// go west
					goToSRLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the north bridge entry
					// go south by the east bound and then west
					goToSRLowerRight();
					goToSRLowerLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is east of the north bridge entry
					// rush for it
					break;
				}
				break;
			}
			break;

		// Entrance is in the East
		case EAST:
			// entrance of the bridge is on the East side
			// avoid SR
			switch (serverData.getSide(EV3WifiClient.Zone.SR, upperRightX,
					lowerLeftY + (upperRightY - lowerLeftY) / 2)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case EAST:
				// the search zone is west of the east bridge entry
				// very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to bridge entrance");
			case SOUTH:
				// the search zone is north of the east bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is north of the east bridge entry
					// goes south around by the east bound
					goToSRUpperRight();
					goToSRLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the east bridge entry
					// go east a bit
					goToSRLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is north of the east bridge entry
					// go south by the east bound
					goToSRLowerRight();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the east bridge entry
					// go south by the west bound and then east
					goToSRLowerLeft();
					goToSRLowerRight();
					break;
				}
				break;
			case NORTH:
				// the search zone is south of the east bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is south of the east bridge entry
					// go east a bit
					goToSRLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the east bridge entry
					// go north by the east bound
					goToSRLowerRight();
					goToSRUpperRight();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the east bridge entry
					// go north using east bound
					goToSRUpperRight();
					break;
				case WEST:
					// robot is west of the search zone, which is south of the east bridge entry
					// go north by the west bound and then east
					goToSRUpperLeft();
					goToSRUpperRight();
					break;
				}
				break;
			case WEST:
				// the search zone is east of the east bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is east of the east bridge entry
					// goes west
					goToSRUpperLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the east bridge entry
					// go west
					goToSRLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the east bridge entry

					if (lowerLeftY < odometer.getY()) {
						// go south by the east bound and then west
						goToSRLowerRight();
						goToSRLowerLeft();
					} else {
						goToSRUpperRight();
						goToSRUpperLeft();
					}
					break;
				case WEST:
					// robot is west of the search zone, which is east of the east bridge entry
					// rush for it
					break;
				}
				break;
			}
			break;

		// Entrance is in the West
		case WEST:
			// entrance of the bridge is on the West side
			// avoid SR
			switch (serverData.getSide(EV3WifiClient.Zone.SR, lowerLeftX,
					lowerLeftY + (upperRightY - lowerLeftY) / 2)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case WEST:
				// the search zone is east of the west bridge entry
				// very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to bridge entrance");
			case SOUTH:
				// the search zone is north of the west bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is north of the west bridge entry
					// goes south around by the west bound
					goToSRUpperLeft();
					goToSRLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the west bridge entry
					// go west a bit
					goToSRLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is north of the west bridge entry
					// go south by the east bound and then west
					goToSRLowerRight();
					goToSRLowerLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the west bridge entry
					// go south by the west bound
					goToSRLowerLeft();
					break;
				}
				break;
			case NORTH:
				// the search zone is south of the west bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is south of the west bridge entry
					// go west a bit
					goToSRLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the west bridge entry
					// go north by the west bound
					goToSRLowerLeft();
					goToSRUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the west bridge entry
					// go for it
					break;
				case WEST:
					// robot is west of the search zone, which is south of the west bridge entry
					// go north by the west bound
					goToSRUpperLeft();
					break;
				}
				break;
			case EAST:
				// the search zone is west of the west bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is west of the west bridge entry
					// goes east
					goToSRUpperRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the west bridge entry
					// go east
					goToSRLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the west bridge entry
					// rush for it

					break;
				case WEST:
					// robot is west of the search zone, which is west of the west bridge entry
					if (lowerLeftY < odometer.getY()) {
						// go south by the west bound and then east
						goToSRLowerLeft();
						goToSRLowerRight();
					} else {
						goToSRUpperLeft();
						goToSRUpperRight();
					}
					break;
				}
				break;
			}
			break;

		// Entrance is in the south
		case SOUTH:
			// entrance of the bridge is on the South side
			// avoid SR
			switch (serverData.getSide(EV3WifiClient.Zone.SR, lowerLeftX + (upperRightX - lowerLeftX) / 2,
					lowerLeftY)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case SOUTH:
				// the search zone is north of the south bridge entry
				// very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to bridge entrance");
			case NORTH:
				// the search zone is south of the south bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// robot in center of search zone
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is south of the south bridge entry
					// rush for it
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the south bridge entry
					// go north using the west bound
					goToSRLowerLeft();
					goToSRUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the south bridge entry
					// go north by the east bound
					goToSRUpperRight();
					break;
				case WEST:
					// robot is west of the search zone, which is south of the south bridge entry
					// go north by the west bound
					goToSRUpperLeft();
					break;
				}
				break;
			case EAST:
				// the search zone is west of the south bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is west of the south bridge entry
					// go east a bit
					goToSRUpperRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the south bridge entry
					// go north using the east bound
					goToSRLowerRight();
					goToSRUpperRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the south bridge entry
					// rush for it
					break;
				case WEST:
					// robot is west of the search zone, which is west of the south bridge entry
					// go north by the west bound and then east
					goToSRUpperLeft();
					goToSRUpperRight();
					break;
				}
				break;
			case WEST:
				// the search zone is east of the south bridge entry
				// look where the robot is compared to the SR zone
				switch (serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is east of the south bridge entry
					// go west a bit
					goToSRUpperLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the south bridge entry
					// go north using the west bound
					goToSRLowerLeft();
					goToSRUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the south bridge entry
					// go north by the east bound and then west
					goToSRUpperRight();
					goToSRUpperLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is east of the south bridge entry
					// rush for it
					break;
				}
				break;
			}
			break;
		}
		//go to the bridge and localize
		localizeBeforeBridge(direction);
	}

	/**
	 * Procedure method to avoid the search zone and water areas to go to the
	 * specified side of the tunnel.
	 * 
	 * @param direction
	 *            Direction (side) of the tunnel entry
	 * @throws Exception
	 *             When there is a problem with the data from the EV3WifiClass or
	 *             with the entry point
	 */
	private void goToTunnel(Direction direction) throws Exception {
		//Saving coordinates of the tunnel to local variables to make the code more readable
		double lowerLeftX = serverData.getCoordParam(CoordParameter.TN_LL_x) * Navigation.TILE_SIZE;
		double lowerLeftY = serverData.getCoordParam(CoordParameter.TN_LL_y) * Navigation.TILE_SIZE;
		double upperRightX = serverData.getCoordParam(CoordParameter.TN_UR_x) * Navigation.TILE_SIZE;
		double upperRightY = serverData.getCoordParam(CoordParameter.TN_UR_y) * Navigation.TILE_SIZE;

		switch (direction) {
		// Entrance is in the North
		case NORTH:
			// entrance of the tunnel is on the North side
			// avoid SG
			switch (serverData.getSide(EV3WifiClient.Zone.SG, lowerLeftX + (upperRightX - lowerLeftX) / 2,
					upperRightY)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case NORTH:
				// the search zone is south of the north tunnel entry
				// very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to tunnel entrance");
			case SOUTH:
				// the search zone is north of the north tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is north of the north tunnel entry
					// goes south around by the west bound
					
					goToSGUpperLeft();
					goToSGLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the north tunnel entry
					// rush for it
					break;
				case EAST:
					// robot is east of the search zone, which is north of the north tunnel entry
					// go south by the east bound
					Sound.beepSequenceUp();
					Sound.beepSequence();
					goToSGLowerRight();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the north tunnel entry
					// go south by the west bound
					goToSGLowerLeft();
					break;
				}
				break;
			case EAST:
				// the search zone is west of the north tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is west of the north tunnel entry
					// goes south around by the east bound
					goToSGUpperRight();
					goToSGLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the north tunnel entry
					// go west
					goToSGLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the north tunnel entry
					// rush for it
					break;
				case WEST:
					// robot is west of the search zone, which is west of the north tunnel entry
					// go south by the west bound and then east
					goToSGLowerLeft();
					goToSGLowerRight();
					break;
				}
				break;
			case WEST:
				// the search zone is east of the north tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is east of the north tunnel entry
					// goes south around by the west bound
					goToSGUpperLeft();
					goToSGLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the north tunnel entry
					// go west
					goToSGLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the north tunnel entry
					// go south by the east bound and then west
					goToSGLowerRight();
					goToSGLowerLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is east of the north tunnel entry
					// rush for it
					break;
				}
				break;
			}
			break;

		// Entrance is in the East
		case EAST:
			// entrance of the tunnel is on the East side
			// avoid SG
			switch (serverData.getSide(EV3WifiClient.Zone.SG, upperRightX,
					lowerLeftY + (upperRightY - lowerLeftY) / 2)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case EAST:
				// the search zone is west of the east tunnel entry
				// very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to tunnel entrance");
			case SOUTH:
				// the search zone is north of the east tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is north of the east tunnel entry
					// goes south around by the east bound
					goToSGUpperRight();
					goToSGLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the east tunnel entry
					// go east a bit
					goToSGLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is north of the east tunnel entry
					// go south by the east bound
					goToSGLowerRight();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the east tunnel entry
					// go south by the west bound and then east
					goToSGLowerLeft();
					goToSGLowerRight();
					break;
				}
				break;
			case NORTH:
				// the search zone is south of the east tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is south of the east tunnel entry
					// go east a bit
					goToSGLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the east tunnel entry
					// go north by the east bound
					goToSGLowerRight();
					goToSGUpperRight();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the east tunnel entry
					// go north using east bound
					goToSGUpperRight();
					break;
				case WEST:
					// robot is west of the search zone, which is south of the east tunnel entry
					// go north by the west bound and then east
					goToSGUpperLeft();
					goToSGUpperRight();
					break;
				}
				break;
			case WEST:
				// the search zone is east of the east tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is east of the east tunnel entry
					// goes west
					goToSGUpperLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the east tunnel entry
					// go west
					goToSGLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the east tunnel entry

					if (lowerLeftY < odometer.getY()) {
						// go south by the east bound and then west
						goToSGLowerRight();
						goToSGLowerLeft();
					} else {
						goToSGUpperRight();
						goToSGUpperLeft();
					}
					break;
				case WEST:
					// robot is west of the search zone, which is east of the east tunnel entry
					// rush for it
					break;
				}
				break;
			}
			break;

		// Entrance is in the West
		case WEST:
			// entrance of the tunnel is on the West side
			// avoid SG
			switch (serverData.getSide(EV3WifiClient.Zone.SG, lowerLeftX,
					lowerLeftY + (upperRightY - lowerLeftY) / 2)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case WEST:
				// the search zone is east of the west tunnel entry
				// very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to tunnel entrance");
			case SOUTH:
				// the search zone is north of the west tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is north of the west tunnel entry
					// goes south around by the west bound
					goToSGUpperLeft();
					goToSGLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the west tunnel entry
					// go west a bit
					goToSGLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is north of the west tunnel entry
					// go south by the east bound and then west
					goToSGLowerRight();
					goToSGLowerLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the west tunnel entry
					// go south by the west bound
					goToSGLowerLeft();
					break;
				}
				break;
			case NORTH:
				// the search zone is south of the west tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is south of the west tunnel entry
					// go west a bit
					goToSGLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the west tunnel entry
					// go north by the west bound
					goToSGLowerLeft();
					goToSGUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the west tunnel entry
					// go for it
					break;
				case WEST:
					// robot is west of the search zone, which is south of the west tunnel entry
					// go north by the west bound
					goToSGUpperLeft();
					break;
				}
				break;
			case EAST:
				// the search zone is west of the west tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is west of the west tunnel entry
					// goes east
					goToSGUpperRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the west tunnel entry
					// go east
					goToSGLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the west tunnel entry
					// rush for it

					break;
				case WEST:
					// robot is west of the search zone, which is west of the west tunnel entry
					if (lowerLeftY < odometer.getY()) {
						// go south by the west bound and then east
						goToSGLowerLeft();
						goToSGLowerRight();
					} else {
						goToSGUpperLeft();
						goToSGUpperRight();
					}
					break;
				}
				break;
			}
			break;

		// Entrance is in the south
		case SOUTH:
			// entrance of the tunnel is on the South side
			// avoid SG
			switch (serverData.getSide(EV3WifiClient.Zone.SG, lowerLeftX + (upperRightX - lowerLeftX) / 2,
					lowerLeftY)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case SOUTH:
				// the search zone is north of the south tunnel entry
				// very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to tunnel entrance");
			case NORTH:
				// the search zone is south of the south tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// robot in center of search zone
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is south of the south tunnel entry
					// rush for it
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the south tunnel entry
					// go north using the west bound
					goToSGLowerLeft();
					goToSGUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the south tunnel entry
					// go north by the east bound
					goToSGUpperRight();
					break;
				case WEST:
					// robot is west of the search zone, which is south of the south tunnel entry
					// go north by the west bound
					goToSGUpperLeft();
					break;
				}
				break;
			case EAST:
				// the search zone is west of the south tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is west of the south tunnel entry
					// go east a bit
					goToSGUpperRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the south tunnel entry
					// go north using the east bound
					goToSGLowerRight();
					goToSGUpperRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the south tunnel entry
					// rush for it
					break;
				case WEST:
					// robot is west of the search zone, which is west of the south tunnel entry
					// go north by the west bound and then east
					goToSGUpperLeft();
					goToSGUpperRight();
					break;
				}
				break;
			case WEST:
				// the search zone is east of the south tunnel entry
				// look where the robot is compared to the SG zone
				switch (serverData.getSide(EV3WifiClient.Zone.SG, odometer.getX(), odometer.getY())) {
				case CENTER:
					// rush for it
					break;
				case NORTH:
					// robot is north of the search zone, which is east of the south tunnel entry
					// go west a bit
					goToSGUpperLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the south tunnel entry
					// go north using the west bound
					goToSGLowerLeft();
					goToSGUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the south tunnel entry
					// go north by the east bound and then west
					goToSGUpperRight();
					goToSGUpperLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is east of the south tunnel entry
					// rush for it
					break;
				}
				break;
			}
			break;
		}
		//finally localize and go to the tunnel
		localizeBeforeTunnel(direction);
	}

	/**
	 * Procedure method to avoid the search zone and water areas and go to the
	 * starting corner.
	 * 
	 * @return True when reached
	 * @throws Exception
	 *             When there is an error with EV3WifiClient variables
	 */
	private boolean goToStartingCorner() throws Exception {
		//Saving coordinates of the starting corner to local variables to make the code more readable
		double startingX = 0, startingY = 0;
		switch (serverData.getStartingCorner()) {
		//Middle point of the respective corner tile
		case 0:
			startingX = Navigation.TILE_SIZE/2 ;
			startingY = Navigation.TILE_SIZE/2 ;
			break;
		case 1:
			startingX = EV3WifiClient.X_GRID_LINES * Navigation.TILE_SIZE - Navigation.TILE_SIZE/2 ;
			startingY = Navigation.TILE_SIZE/2 ;
			break;
		case 2:
			startingX = EV3WifiClient.X_GRID_LINES * Navigation.TILE_SIZE - Navigation.TILE_SIZE/2;
			startingY = EV3WifiClient.Y_GRID_LINES * Navigation.TILE_SIZE - Navigation.TILE_SIZE/2 ;
			break;
		case 3:
			startingX = Navigation.TILE_SIZE/2;
			startingY = EV3WifiClient.Y_GRID_LINES * Navigation.TILE_SIZE - Navigation.TILE_SIZE/2 ;
			break;
		}
		
		//Going back to the starting corner of its team color
		switch (serverData.getTeamColor()) {
		case GREEN:
			//Green team
			//Gets where the starting corner is with respect to the Green search zone (SG)
			//Helps avoid the SG zone when going back to the corner
			switch (serverData.getSide(Zone.SG, startingX, startingY)) {
			case CENTER:
			case NORTH:
			case SOUTH:
				// Since the zone are always rectangles,
				// the starting corner is always east or west
				// of the search zone
				throw new Exception("GoToStarting corner: \nerror with the green starting corner position");
				
			case EAST:
				// starting corner is east of SG zone
				//look if the robot is South of the starting corner
				if (odometer.getY() < startingY) {
					// robot is south of starting corner
					// Get where the robot is with respect to the Green search zone to avoid it
					switch (serverData.getSide(Zone.SG, odometer.getX(), odometer.getY())) {
					case CENTER:
						//robot is in the middle of the SG
						// rush for it
					case NORTH:
						// robot north of the search zone, which is south west of the starting corner
						// go east a bit
						goToSGUpperRight();
						break;
					case EAST:
						// robot east of the search zone, which is south west of the starting corner
						// rush for it
						break;
					case SOUTH:
						// robot south of the search zone, which is south west of the starting corner
						// go east a bit
						goToSGLowerRight();
						break;
					case WEST:
						// robot west of the search zone, which is south west of the starting corner
						// go north and then east
						goToSGUpperLeft();
						goToSGUpperRight();
						break;
					}

				} else {
					// robot is north of starting corner
					switch (serverData.getSide(Zone.SG, odometer.getX(), odometer.getY())) {
					case CENTER:
						// rush for it
					case NORTH:
						// robot north of the search zone, which is north west of the starting corner
						// go east a bit
						goToSGLowerRight();
						break;
					case EAST:
						// robot east of the search zone, which is north west of the starting corner
						// rush for it
						break;
					case SOUTH:
						// robot south of the search zone, which is north west of the starting corner
						// go east a bit
						goToSGUpperRight();
						break;
					case WEST:
						// robot west of the search zone, which is north west of the starting corner
						// go south then east
						goToSGLowerLeft();
						goToSGLowerRight();
						break;
					}
				}

				break;
			case WEST:
				// starting corner is west of SG zone
				//look if the robot is South of the starting corner
				if (odometer.getY() < startingY) {
					// robot is south of starting corner
					switch (serverData.getSide(Zone.SG, odometer.getX(), odometer.getY())) {
					case CENTER:
						// rush for it
					case NORTH:
						// robot north of the search zone, which is south east of the starting corner
						// go west a bit
						goToSGUpperLeft();
						break;
					case EAST:
						// robot east of the search zone, which is south east of the starting corner
						// go north then west
						goToSGUpperRight();
						goToSGUpperLeft();
						break;
					case SOUTH:
						// robot south of the search zone, which is south east of the starting corner
						// go west a bit
						goToSGLowerLeft();
						break;
					case WEST:
						// robot west of the search zone, which is south east of the starting corner
						// rush for it
						break;
					}

				} else {
					// robot is north of starting corner
					switch (serverData.getSide(Zone.SG, odometer.getX(), odometer.getY())) {
					case CENTER:
						// rush for it
					case NORTH:
						// robot north of the search zone, which is north east of the starting corner
						// go east a bit
						goToSGLowerLeft();
						break;
					case EAST:
						// robot east of the search zone, which is north east of the starting corner
						// go south then west
						goToSGLowerRight();
						goToSGLowerLeft();
						break;
					case SOUTH:
						// robot south of the search zone, which is north east of the starting corner
						// go west a but
						goToSGUpperLeft();
						break;
					case WEST:
						// robot west of the search zone, which is north east of the starting corner
						// rush for it
						break;
					}
				}
				break;
			}
			break;
		case RED:
			//Robot is in RED team
			// red starting corner
			//Gets where the starting corner is with respect to the RED search zone (SR)
			//Helps avoid the SR zone when going back to the corner
			switch (serverData.getSide(Zone.SR, startingX, startingY)) {
			case CENTER:
			case NORTH:
			case SOUTH:
				// Since the zone are always rectangles,
				// the starting corner is always east or west
				// of the search zone
				throw new Exception("GoToStarting corner: \nerror with the red starting corner position");
			case EAST:
				// starting corner is east of SR zone
				// Looks if robot is South of the starting corner
				if (odometer.getY() < startingY) {
					// robot is south of starting corner
					// Looks at on what side of the SR zone the robot is
					switch (serverData.getSide(Zone.SR, odometer.getX(), odometer.getY())) {
					case CENTER:
						// rush for it
					case NORTH:
						// robot north of the search zone, which is south west of the starting corner
						// go east a bit
						goToSRUpperRight();
						break;
					case EAST:
						// robot east of the search zone, which is south west of the starting corner
						// rush for it
						break;
					case SOUTH:
						// robot south of the search zone, which is south west of the starting corner
						// go east a bit
						goToSRLowerRight();
						break;
					case WEST:
						// robot west of the search zone, which is south west of the starting corner
						// go north then east
						goToSRUpperLeft();
						goToSRUpperRight();
						break;
					}

				} else {
					// robot is north of starting corner
					// Looks at on what side of the SR zone the robot is
					switch (serverData.getSide(Zone.SR, odometer.getX(), odometer.getY())) {
					case CENTER:
						// rush for it
					case NORTH:
						// robot north of the search zone, which is north west of the starting corner
						// go east a bit
						goToSRLowerRight();
						break;
					case EAST:
						// robot east of the search zone, which is north west of the starting corner
						// rush for it
						break;
					case SOUTH:
						// robot south of the search zone, which is north west of the starting corner
						// go east a bit
						goToSRUpperRight();
						break;
					case WEST:
						// robot west of the search zone, which is north west of the starting corner
						// go south then east
						goToSRLowerLeft();
						goToSRLowerRight();
						break;
					}
				}

				break;
			case WEST:
				// starting corner is west of SR zone
				// Looks if robot is South of the starting corner
				if (odometer.getY() < startingY) {
					// robot is south of starting corner
					// Looks at on what side of the SR zone the robot is
					switch (serverData.getSide(Zone.SR, odometer.getX(), odometer.getY())) {
					case CENTER:
						// rush for it
					case NORTH:
						// robot north of the search zone, which is south east of the starting corner
						// go west a bit
						goToSRUpperLeft();
						break;
					case EAST:
						// robot east of the search zone, which is south east of the starting corner
						// go north then west
						goToSRUpperRight();
						goToSRUpperLeft();
						break;
					case SOUTH:
						// robot south of the search zone, which is south east of the starting corner
						// go west a bit
						goToSRLowerLeft();
						break;
					case WEST:
						// robot west of the search zone, which is south east of the starting corner
						// rush for it
						break;
					}

				} else {
					// robot is north of starting corner
					switch (serverData.getSide(Zone.SR, odometer.getX(), odometer.getY())) {
					case CENTER:
						// rush for it
					case NORTH:
						// robot north of the search zone, which is north east of the starting corner
						// go east a bit
						goToSRLowerLeft();
						break;
					case EAST:
						// robot east of the search zone, which is north east of the starting corner
						// go south then west
						goToSRLowerRight();
						goToSRLowerLeft();
						break;
					case SOUTH:
						// robot south of the search zone, which is north east of the starting corner
						// go west a bit
						goToSRUpperLeft();
						break;
					case WEST:
						// robot west of the search zone, which is north east of the starting corner
						// rush for it
						break;
					}
				}
				break;
			}
			break;
		}
		// robot is finally able to go the its starting corner normally
		// Go ahead robot, be free
		navigation.travelTo(startingX, startingY);
		return true;
	}

	/**
	 * Makes the robot navigate to the lower left corner of the search zone in the
	 * red zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSRLowerLeft() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SR_LL_x) -0.5)* Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SR_LL_y) -0.5)* Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the lower right corner of the search zone in the
	 * red zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSRLowerRight() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SR_UR_x) +0.5) * Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SR_LL_y) -0.5)* Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the upper left corner of the search zone in the
	 * red zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSRUpperLeft() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SR_LL_x)-0.5) * Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SR_UR_y)+0.5) * Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the upper right corner of the search zone in the
	 * red zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSRUpperRight() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SR_UR_x) +0.5)* Navigation.TILE_SIZE,
				(serverData.getCoordParam(CoordParameter.SR_UR_y) +0.5)* Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the lower left corner of the search zone in the
	 * green zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSGLowerLeft() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SG_LL_x) -0.5) * Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SG_LL_y) -0.5)* Navigation.TILE_SIZE);
		return true;
	}

	/**
	 * Makes the robot navigate to the lower right corner of the search zone in the
	 * green zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSGLowerRight() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SG_UR_x) +0.5)* Navigation.TILE_SIZE  ,
				(serverData.getCoordParam(CoordParameter.SG_LL_y) -0.5) * Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the upper left corner of the search zone in the
	 * green zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSGUpperLeft() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SG_LL_x) -0.5)* Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SG_UR_y) +0.5)* Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the upper right corner of the search zone in the
	 * green zone The robot will be outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSGUpperRight() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SG_UR_x) +0.5) * Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SG_UR_y) +0.5)* Navigation.TILE_SIZE );
		return true;
	}
	
}