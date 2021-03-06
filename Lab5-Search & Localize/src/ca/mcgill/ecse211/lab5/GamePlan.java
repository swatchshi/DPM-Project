package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.EV3WifiClient.CoordParameter;
import ca.mcgill.ecse211.lab5.EV3WifiClient.Zone;
import ca.mcgill.ecse211.lab5.Navigation.Turn;
import ca.mcgill.ecse211.localizer.*;
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

/**
 * Class responsible for the game play. This class creates instances of a game,
 * which will make the proper procedure calls and decide the behavior of the
 * robot.
 * 
 * @author Xavier Pellemans
 * @author Thomas Bahen
 * @author Zhang Guangyi
 * @author Zhang Cara
 * @author Shi WenQi
 *
 */
public class GamePlan {
	
	/**
	 * Enum for the chosen robot 
	 * 
	 * SCREW_DESIGN: design with the expanding track and screw
	 * TANK: design with the tank tracks
	 * 
	 * To add more designs, first add its name here,
	 * then make sure to follow the instructions in the TrackExpansion class
	 */
	public enum Robot {
		SCREW_DESIGN, TANK
	}
	
	/**
	 * Enum describing the cardinal point. Used to describe the side of a region.
	 * NORTH: side with the highest y 
	 * EAST: side with the highest x 
	 * SOUTH: side with the lowest y 
	 * WEST: side with the lowest x 
	 * CENTER: in the middle
	 */
	public enum Direction {
		NORTH, EAST, SOUTH, WEST, CENTER
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
	 * Type of the robot used
	 * Choose from the enum Robot
	 */
	public static final Robot robot = Robot.TANK;
	
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
	 * Object in charge of detecting the color of the block ahead of the front 
	 * light sensor
	 */
	private ColorSensor cSensor;
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
	 * Boolean to determine if the robot will allow Odometer
	 * correction when running
	 */
	private boolean canOdometerCorrect=true;
	
	/**
	 * Creates an object of the GamePlan class. Initializes all instances needed in
	 * the game
	 * 
	 * @throws Exception
	 *             error with Odometer instances or with data server retrieval
	 */
	public GamePlan() throws Exception {
		//initializing sensors
		lSensor = new ColorSensor(lightSensor);
		ultraSensor = new UltrasonicSensor(ultraSSensor);
		cSensor=new ColorSensor(armSensor);
		gyroscope = new Gyroscope(gyroSensor);
		// track related object
		dynamicTrack = new TrackExpansion();
		dynamicTrack.setDesignConstants(robot); // sets the values for the chosen robot

		// Odometer related objects
		odometer = Odometer.getOdometer(leftMotor, rightMotor, dynamicTrack, gyroscope, dynamicTrack.getConfig());
		//odometryDisplay = new Display(lcd, ultraSensor, gyroscope);
		odoCorrect = new OdometerCorrection(lSensor, odometer, dynamicTrack);
		navigation = new Navigation(odometer, dynamicTrack);
		//procedure objects
		usLoc = new USLocalizer(odometer, navigation, ultraSensor);
		lightLoc = new LightLocalizer(navigation, dynamicTrack, lSensor, odometer, gyroscope);
		internalClock=new InternalClock();
		
		
		
		//enabling or disabling thread functions
		odoCorrect.setDoCorrection(false);
		odometer.setDoThetaCorrection(false);
		odometer.setEnablePrint(false);
		
		//retrieving server data
		serverData = new EV3WifiClient(); 
		
		//Starting odometer thread
		Thread odoThread = new Thread(odometer);
		odoThread.start(); //starting the count of wheel rotations
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
		//variables for calculating the track
		int lineCount=5, i=0;
		int leftMotorLastTachoCount=0, rightMotorLastTachoCount=0,leftMotorTachoCount, rightMotorTachoCount;
	    double distL, distR, track;
	    odometer.setEnablePrint(false);
	    //Rotational movement (360 degree rotation)
	    navigation.setRotateSpeed(Navigation.SLOW_ROTATE_SPEED);
	    navigation.rotate(Turn.CLOCK_WISE);
	    while(i<lineCount) { //counts the lines seen
	    	if(lSensor.lineDetected()) {
	    		i++;
	    	}
	    	if(i==1) { //if it is the first line seen
	    		//save the initial amount of rotation of the wheels
	    		leftMotorLastTachoCount = leftMotor.getTachoCount();
	    		rightMotorLastTachoCount = rightMotor.getTachoCount();
	    	}
	    }
	    navigation.stopMotors();
	    //save the final amount of wheel rotation
	    leftMotorTachoCount = leftMotor.getTachoCount();
	    rightMotorTachoCount = rightMotor.getTachoCount();
		      
	    //Compute the track
		distL=Math.PI*dynamicTrack.getWheelRad()*(leftMotorTachoCount-leftMotorLastTachoCount)/180; 	//convert left rotation to wheel displacement
		distR=Math.PI*dynamicTrack.getWheelRad()*(rightMotorTachoCount-rightMotorLastTachoCount)/180;	//convert right rotation to wheel displacement
		track=(distR-distL)/(2*Math.PI); //Calculating the track by knowing the rotation of the wheels
		lcd.drawString("Track: "+ track, 0, 6); //display the track on the screen
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
		int buttonID;
		//displays the color seen
		do {
			lcd.drawString(cSensor.getColorSeen().toString(), 0, 0);
			buttonID=Button.waitForAnyPress();
		}while(buttonID!=Button.ID_ESCAPE);
		lcd.clear();
		odometer.setEnablePrint(true); 
	}
	
	/**
	 * Drives the robot in squares 
	 * Waits for user input after the completion
	 * of the square.
	 * 
	 * -Escapes when the enter button is pressed
	 * -Increases the track value by 0.05 when up is pressed
	 * -Decreases the track value by 0.05 when down is pressed
	 * -Increases the wheel radius by 0.01 when right is pressed
	 * -Decreases the wheel radius by 0.01 when left is pressed
	 * 
	 * @param tiles Number of tiles per side
	 * @return True when the user ends the procedure
	 */
	public boolean squareDrive(int tiles) {
		int buttonID=0;
		lcd.clear();
		lcd.drawString("Initial track: "+dynamicTrack.getTrack(), 0, 1);
		lcd.drawString("Initial w-rad: "+dynamicTrack.getTrack(), 0, 2);
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
				//increase the track
				dynamicTrack.setTrack(dynamicTrack.getTrack()+0.05);
				lcd.drawString("New track: "+dynamicTrack.getTrack(), 0, 3);
			}else if(buttonID==Button.ID_DOWN) {
				//decrease the track
				dynamicTrack.setTrack(dynamicTrack.getTrack()-0.05);
				lcd.drawString("New track: "+dynamicTrack.getTrack(), 0, 3);
			}else if(buttonID==Button.ID_LEFT) {
				//decrease the wheel radius
				dynamicTrack.setWheelRad(dynamicTrack.getWheelRad()-0.01);
				lcd.drawString("New w-rad: "+dynamicTrack.getWheelRad(), 0, 4);
			}else if(buttonID==Button.ID_LEFT) {
				//increase the wheel radius
				dynamicTrack.setWheelRad(dynamicTrack.getWheelRad()+0.01);
				lcd.drawString("New w-rad: "+dynamicTrack.getWheelRad(), 0, 4);
			}
		}while(buttonID!=Button.ID_ENTER);
		return true;
	}
	
	

	/**
	 * Beginning of the game!
	 * Determines what Team color plan should be followed
	 * Also starts the timer for the game
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
	 * @throws Exception
	 *             When there is a problem with the data from the EV3WifiClass
	 */
	private void redPlan() throws Exception {
		
		//Localizing at the corner
		navigation.setForwardSpeed(Navigation.LOCALIZATION_SPEED);
				
		usLoc.doLocalization(); 
		usLoc=null; //garbage collect will automatically dispose it
		
		Sound.beepSequenceUp();
				
		//Light localizing at the closest crossing
		navigation.setEnableGyroscopeCorrection(true);
		lightLoc.crashLocalizer(serverData.getStartingCorner());
				
		//set parameters
		Sound.beepSequenceUp();
		Thread odoCorrectionThread = new Thread(odoCorrect);
		odoCorrectionThread.start();
		navigation.setForwardSpeed(Navigation.FORWARD_SPEED);
		odoCorrect.setDoCorrection(canOdometerCorrect);
		navigation.setEnableGyroscopeCorrection(true);
				
		//Going to the bridge
		goToBridge(getBridgeEntry());
		Sound.beepSequenceUp();
		crossBridge();
		Sound.beepSequenceUp();
				
				
		//Find the flag in the green search zone
		//Creates a FlagFinding instance and immediately asks it to find the block
		//given the search zone to look inside, the side of the search zone the robot will start at,
		//the lower left corner and upper right corner line coordinates of the search zone
		// and the color of the flag desired
		(new FlagFinding(dynamicTrack, cSensor, ultraSensor, internalClock)).findBlock(getSearchStartSide(Zone.SG, directionSwitch(getTunnelEntry())),
				serverData.getCoordParam(CoordParameter.SG_LL_x), serverData.getCoordParam(CoordParameter.SG_LL_y),
				serverData.getCoordParam(CoordParameter.SG_UR_x), serverData.getCoordParam(CoordParameter.SG_UR_y), 
				serverData.getFlagColor());
		
				
				
				
		//Going to the tunnel
		goToBridge(getTunnelEntry());
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
		usLoc=null; //garbage collect will automatically dispose it
		
		Sound.beepSequenceUp();
		
		//Light localizing at the closest crossing
		navigation.setEnableGyroscopeCorrection(true);
		lightLoc.crashLocalizer(serverData.getStartingCorner());
		
		//set parameters
		Sound.beepSequenceUp();
		Thread odoCorrectionThread = new Thread(odoCorrect);
		odoCorrectionThread.start();
		navigation.setForwardSpeed(Navigation.FORWARD_SPEED);
		odoCorrect.setDoCorrection(canOdometerCorrect);
		navigation.setEnableGyroscopeCorrection(true);
		
		//Going to the tunnel
		goToTunnel(getTunnelEntry());
		Sound.beepSequenceUp();
		crossTunnel();
		Sound.beepSequenceUp();
		
		
		//Find the flag in the red search zone
		//Creates a FlagFinding instance and immediately asks it to find the block
		//given the search zone to look inside, the side of the search zone the robot will start at,
		//the lower left corner and upper right corner line coordinates of the search zone
		// and the color of the flag desired
		
		(new FlagFinding(dynamicTrack, new ColorSensor(armSensor), ultraSensor, internalClock)).findBlock(Direction.EAST,
				serverData.getCoordParam(CoordParameter.SR_LL_x), serverData.getCoordParam(CoordParameter.SR_LL_y),
				serverData.getCoordParam(CoordParameter.SR_UR_x), serverData.getCoordParam(CoordParameter.SR_UR_y), 
				serverData.getFlagColor());
		
		
		//Going to the bridge
		localizeBeforeBridge(getBridgeEntry());
		Sound.beepSequenceUp();
		crossBridge();
		Sound.beepSequenceUp();
		
		//finish 
		goToStartingCorner();
		Sound.beepSequenceUp();
	}

	
	/**
	 * Procedure to cross the bridge
	 * Takes into account the length of the bridge
	 * Will try to end up in the middle of the tile
	 * after the bridge
	 * 
	 * @return True when the bridge has been crossed
	 * @throws Exception
	 *             if the specified entry point is incorrect
	 */
	private boolean crossBridge() throws Exception { // expansion method, travel directly
		odoCorrect.setDoCorrection(false);
		navigation.travel(Navigation.TILE_SIZE * (1 + serverData.getBridgeWidth(getBridgeEntry())));
		// travels the width of the bridge plus an extra tile
		odoCorrect.setDoCorrection(canOdometerCorrect);
		return true;
	}

	/**
	 * Procedure to cross the tunnel
	 * Takes into account the length of the tunnel
	 * Will try to end up in the middle of the tile
	 * after the tunnel
	 * 
	 * @return True when the tunnel has been crossed
	 * @throws Exception
	 *             if the specified entry point is incorrect
	 */
	private boolean crossTunnel() throws Exception {
		odoCorrect.setDoCorrection(false);
		navigation.travel(Navigation.TILE_SIZE * (1 + serverData.getTunnelWidth(getTunnelEntry())));
		// travels the width of the tunnel plus an extra tile
		odoCorrect.setDoCorrection(canOdometerCorrect);
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
		switch(serverData.getZone((upperRightX+lowerLeftX)/2, upperRightY+Navigation.TILE_SIZE/2)) {
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
			switch(serverData.getZone(upperRightX+Navigation.TILE_SIZE/2, (upperRightY+lowerLeftY)/2)) {
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
		switch(serverData.getZone((upperRightX+lowerLeftX)/2, upperRightY+Navigation.TILE_SIZE/2)) {
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
			switch(serverData.getZone(upperRightX+Navigation.TILE_SIZE/2,(upperRightY+lowerLeftY)/2)) {
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
				//if closer to the south side
				flagSearchStartSide=Direction.SOUTH;
				break;
			default:
				//else go look at your side
				flagSearchStartSide=serverData.getSide(searchZone, odometer.getX(), odometer.getY());
				break;
			}
			break;
		case EAST:
			//looks where the robot is with respect to the search zone desired
			switch(serverData.getSide(searchZone, odometer.getX(), odometer.getY())) {
			case CENTER:
			case NORTH:
				//if closer to the west side
				flagSearchStartSide=Direction.WEST;
				break;
			default:
				//else go look at your side
				flagSearchStartSide=serverData.getSide(searchZone, odometer.getX(), odometer.getY());
				break;
			}
			break;
		case SOUTH:
			//looks where the robot is with respect to the search zone desired
			switch(serverData.getSide(searchZone, odometer.getX(), odometer.getY())) {
			case CENTER:
			case EAST:
				//if closer to the north side
				flagSearchStartSide=Direction.NORTH;
				break;
			default:
				//else go look at your side
				flagSearchStartSide=serverData.getSide(searchZone, odometer.getX(), odometer.getY());
				break;
			}
			break;
		case WEST:
			//looks where the robot is with respect to the search zone desired
			switch(serverData.getSide(searchZone, odometer.getX(), odometer.getY())) {
			case CENTER:
			case SOUTH:
				//if closer to the east side
				flagSearchStartSide=Direction.EAST;
				break;
			default:
				//else go look at your side
				flagSearchStartSide=serverData.getSide(searchZone, odometer.getX(), odometer.getY());
				break;
			}
			break;
		default: flagSearchStartSide=Direction.SOUTH; //will start the search on the south side of the zone
		}
		return flagSearchStartSide;
	}
	
	/** 
	 * Prepares the robot to cross the bridge by
	 * going to the entry tile and localizing before crossing the bridge
	 * 
	 * Will get to the tile in front of the entry of the bridge,
	 * detect the borders of the tile which are parallel to the bridge
	 * and place the robot in its center, facing the bridge.
	 * 
	 * Does not avoid the search zone.
	 * 
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
		
		//local variables for localizing
		double first, second;
		
		//Direction of the bridge entry
		switch(direction) {
		case NORTH:
			//Entry of bridge is in the North part of the bridge
			//Localizes in the tile North of it
			GamePlan.lcd.drawString("North loc", 0, 7);
			odoCorrect.setDoCorrection(false);
			
			//Determines if there is a danger of falling in the water when localizing
			//by looking if the tile west of the tile just north of the bridge is water 
			if(serverData.getZone((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE)==Zone.WATER) {
				navigation.travelTo((lowerLeftXLine+0.55)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE);
				navigation.turnTo(90); //look east
			}else {
				navigation.travelTo((lowerLeftXLine+0.45)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE);
				navigation.turnTo(270); //look west
			}
			
			//localize
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors();//detected the first line
			first=odometer.getX(); 
			
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors();//detected the second line
			second=odometer.getX(); 
			
			//go to the middle of the tile
			navigation.travel(Math.abs(first-second)/2+dynamicTrack.getLightSensorDistance()); 
			navigation.turnTo(180);
			
			
			break;
		case SOUTH:
			//Entry of bridge is in the South part of the bridge
			//Localizes in the tile South of it
			GamePlan.lcd.drawString("South loc", 0, 7);
			odoCorrect.setDoCorrection(false);
			
			//Determines if there is a danger of falling in the water when localizing
			//by looking if the tile west of the tile just south of the bridge is water 
			if(serverData.getZone((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (lowerLeftYLine-0.5)*Navigation.TILE_SIZE)==Zone.WATER) {
				navigation.travelTo((lowerLeftXLine+0.55)*Navigation.TILE_SIZE, (lowerLeftYLine-0.5)*Navigation.TILE_SIZE);
				navigation.turnTo(90); //look east
			}else {
				navigation.travelTo((lowerLeftXLine+0.45)*Navigation.TILE_SIZE, (lowerLeftYLine-0.5)*Navigation.TILE_SIZE);
				navigation.turnTo(270); //look west
			}
			//localize
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors();
			first=odometer.getX(); //first line detected
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //second line detected
			second=odometer.getX();
			//go to the middle of the tile
			navigation.travel(Math.abs(first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(0);
			break;
		case EAST:
			//Entry of bridge is in the East part of the bridge
			//Localizes in the tile East of it
			GamePlan.lcd.drawString("EAST loc", 0, 7);
			odoCorrect.setDoCorrection(false);
			
			//Determines if there is a danger of falling in the water when localizing
			//by looking if the tile north of the tile just east of the bridge is water 
			if(serverData.getZone((upperRightXLine+0.5)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE)==Zone.WATER) {
				navigation.travelTo((upperRightXLine+0.5)*Navigation.TILE_SIZE, (upperRightYLine-0.55)*Navigation.TILE_SIZE);
				navigation.turnTo(180); //look south
			}else {
				navigation.travelTo((upperRightXLine+0.5)*Navigation.TILE_SIZE, (upperRightYLine-0.45)*Navigation.TILE_SIZE);
				navigation.turnTo(0); //look north
			}
			//localize
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //first line detected
			first=odometer.getY();
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //second line detected
			second=odometer.getY();
			//go to the middle of the tile
			navigation.travel(Math.abs(first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(270);
			break;
		case WEST:
			//Entry of bridge is in the West part of the bridge
			//Localizes in the tile West of it
			GamePlan.lcd.drawString("West loc", 0, 7);
			odoCorrect.setDoCorrection(false);
			
			//Determines if there is a danger of falling in the water when localizing
			//by looking if the tile north of the tile just west of the bridge is water 
			if(serverData.getZone((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE)==Zone.WATER) {
				navigation.travelTo((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (lowerLeftYLine+0.45)*Navigation.TILE_SIZE);
				navigation.turnTo(180); //look south
			}else {
				navigation.travelTo((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (lowerLeftYLine+0.55)*Navigation.TILE_SIZE);
				navigation.turnTo(0); //look north
			}
			//localize
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //first line detected
			first=odometer.getY();
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors();//second line detected
			second=odometer.getY();
			navigation.travel(Math.abs(first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(90);
			break;
		case CENTER:
			//do nothing
			break;
		}
	}
	
	/**
	 * Prepares the robot to cross the tunnel by
	 * going to the entry tile and localizing before crossing the tunnel
	 * 
	 * Will get to the tile in front of the entry of the tunnel,
	 * detect the borders of the tile which are parallel to the tunnel
	 * and place the robot in its center, facing the tunnel.
	 * 
	 * Does not avoid the search zone.
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
		
		//local variables for localizing
		double first, second;
		
		//direction of the tunnel entry
		switch(direction) {
		case NORTH:
			//Entry of tunnel is in the North part of the tunnel
			//Localizes in the tile North of it
			GamePlan.lcd.drawString("North loc", 0, 5);
			odoCorrect.setDoCorrection(false);
			
			//Determines if there is a danger of falling in the water when localizing
			//by looking if the tile west of the tile just north of the tunnel is water 
			if(serverData.getZone((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE)==Zone.WATER) {
				navigation.travelTo((lowerLeftXLine+0.55)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE);
				navigation.turnTo(90); //look east
			}else {
				navigation.travelTo((lowerLeftXLine+0.45)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE);
				navigation.turnTo(270); //look west
			}
			
			//localize
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //first line detected
			first=odometer.getX();
			
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //second line detected
			second=odometer.getX();
			//go in the middle of the tile
			navigation.travel(Math.abs(first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(180);
			
			break;
		case SOUTH:
			//Entry of tunnel is in the South part of the tunnel
			//Localizes in the tile South of it
			GamePlan.lcd.drawString("South loc", 0, 5);
			odoCorrect.setDoCorrection(false);
			
			//Determines if there is a danger of falling in the water when localizing
			//by looking if the tile west of the tile just south of the tunnel is water 
			if(serverData.getZone((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (lowerLeftYLine-0.5)*Navigation.TILE_SIZE)==Zone.WATER) {
				navigation.travelTo((lowerLeftXLine+0.55)*Navigation.TILE_SIZE, (lowerLeftYLine-0.5)*Navigation.TILE_SIZE);
				navigation.turnTo(90); //look east
			}else {
				navigation.travelTo((lowerLeftXLine+0.45)*Navigation.TILE_SIZE, (lowerLeftYLine-0.5)*Navigation.TILE_SIZE);
				navigation.turnTo(270); //look west
			}
			
			//localize
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //first line detected
			first=odometer.getX();
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //second line detected
			second=odometer.getX();
			//go to the middle of the tile
			navigation.travel(Math.abs(first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(0);
			break;
		case EAST:
			//Entry of tunnel is in the East part of the tunnel
			//Localizes in the tile East of it
			GamePlan.lcd.drawString("East loc", 0, 5);
			odoCorrect.setDoCorrection(false);
			navigation.travelTo((upperRightXLine+0.5)*Navigation.TILE_SIZE, (upperRightYLine-0.55)*Navigation.TILE_SIZE);
			
			//Determines if there is a danger of falling in the water when localizing
			//by looking if the tile north of the tile just east of the tunnel is water 
			if(serverData.getZone((upperRightXLine+0.5)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE)==Zone.WATER) {
				navigation.travelTo((upperRightXLine+0.5)*Navigation.TILE_SIZE, (upperRightYLine-0.55)*Navigation.TILE_SIZE);
				navigation.turnTo(180); //look south
			}else {
				navigation.travelTo((upperRightXLine+0.5)*Navigation.TILE_SIZE, (upperRightYLine-0.45)*Navigation.TILE_SIZE);
				navigation.turnTo(0); //look north
			}
			
			//localize
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //first line detected
			first=odometer.getY();
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //second line detected
			second=odometer.getY();
			//go to the middle of the tile
			navigation.travel(Math.abs(first-second)/2+dynamicTrack.getLightSensorDistance());
			navigation.turnTo(270);
			break;
		case WEST:
			//Entry of tunnel is in the West part of the tunnel
			//Localizes in the tile West of it
			GamePlan.lcd.drawString("West loc", 0, 5);
			odoCorrect.setDoCorrection(false);
			
			//Determines if there is a danger of falling in the water when localizing
			//by looking if the tile north of the tile just west of the tunnel is water 
			if(serverData.getZone((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (upperRightYLine+0.5)*Navigation.TILE_SIZE)==Zone.WATER) {
				navigation.travelTo((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (lowerLeftYLine+0.45)*Navigation.TILE_SIZE);
				navigation.turnTo(180); //look south
			}else {
				navigation.travelTo((lowerLeftXLine-0.5)*Navigation.TILE_SIZE, (lowerLeftYLine+0.55)*Navigation.TILE_SIZE);
				navigation.turnTo(0); //look north
			}
			
			//localize
			navigation.travelForward();
			
			navigation.setEnableGyroscopeCorrection(true);
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //first line detected
			first=odometer.getY();
			navigation.backUp(Navigation.TILE_SIZE/2);
			navigation.travelBackward();
			while(!lSensor.lineDetected());
			navigation.stopMotors(); //second line detected
			second=odometer.getY();
			//go to the middle of the tile
			navigation.travel(Math.abs(first-second)/2+dynamicTrack.getLightSensorDistance());
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
	 * WARNING: this method contains a lot of cases which are all the cases
	 * 			possible for the configuration of the entry point of the bridge
	 * 			the search zone in the red zone and the position of the robot at 
	 * 			that time.
	 * 			Keep in mind that it finds the shortest path to avoid the search zone,
	 * 			goes around by going to its corners: goToSRLowerLeft() ,
	 * 			goToSRLowerRight() , goToSRUpperLeft() , goToSRUpperRight() 
	 * 
	 * 			After, it will call the localizeBeforeBridge(direction) method
	 * 			
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
					GamePlan.lcd.drawString("North SOUTH NORTH", 0, 6);
					goToSRUpperLeft();
					goToSRLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the north bridge entry
					// rush for it
					GamePlan.lcd.drawString("North SOUTH SOUTH", 0, 6);
					break;
				case EAST:
					// robot is east of the search zone, which is north of the north bridge entry
					// go south by the east bound
					GamePlan.lcd.drawString("North SOUTH EAST", 0, 6);
					goToSRLowerRight();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the north bridge entry
					// go south by the west bound
					GamePlan.lcd.drawString("North SOUTH WEST", 0, 6);
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
					GamePlan.lcd.drawString("North EAST NORTH", 0, 6);
					goToSRUpperRight();
					goToSRLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the north bridge entry
					// go west
					GamePlan.lcd.drawString("North EAST SOUTH", 0, 6);
					goToSRLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the north bridge entry
					// rush for it
					GamePlan.lcd.drawString("North EAST EAST", 0, 6);
					break;
				case WEST:
					// robot is west of the search zone, which is west of the north bridge entry
					// go south by the west bound and then east
					GamePlan.lcd.drawString("North EAST WEST", 0, 6);
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
					GamePlan.lcd.drawString("North WEST NORTH", 0, 6);
					goToSRUpperLeft();
					goToSRLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the north bridge entry
					// go west
					GamePlan.lcd.drawString("North WEST SOUTH", 0, 6);
					goToSRLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the north bridge entry
					// go south by the east bound and then west
					GamePlan.lcd.drawString("North WEST EAST", 0, 6);
					goToSRLowerRight();
					goToSRLowerLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is east of the north bridge entry
					// rush for it
					GamePlan.lcd.drawString("North WEST WEST", 0, 6);
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
					GamePlan.lcd.drawString("East SOUTH NORTH", 0, 6);
					goToSRUpperRight();
					goToSRLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the east bridge entry
					// go east a bit
					GamePlan.lcd.drawString("East SOUTH SOUTH", 0, 6);
					goToSRLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is north of the east bridge entry
					// go south by the east bound
					GamePlan.lcd.drawString("East SOUTH EAST", 0, 6);
					goToSRLowerRight();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the east bridge entry
					// go south by the west bound and then east
					GamePlan.lcd.drawString("East SOUTH WEST", 0, 6);
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
					GamePlan.lcd.drawString("East NORTH NORTH", 0, 6);
					goToSRLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the east bridge entry
					// go north by the east bound
					GamePlan.lcd.drawString("East NORTH SOUTH", 0, 6);
					goToSRLowerRight();
					goToSRUpperRight();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the east bridge entry
					// go north using east bound
					GamePlan.lcd.drawString("East NORTH EAST", 0, 6);
					goToSRUpperRight();
					break;
				case WEST:
					// robot is west of the search zone, which is south of the east bridge entry
					// go north by the west bound and then east
					GamePlan.lcd.drawString("East NORTH WEST", 0, 6);
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
					GamePlan.lcd.drawString("East WEST NORTH", 0, 6);
					goToSRUpperLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the east bridge entry
					// go west
					GamePlan.lcd.drawString("East WEST SOUTH", 0, 6);
					goToSRLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the east bridge entry
					GamePlan.lcd.drawString("East WEST EAST", 0, 6);
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
					GamePlan.lcd.drawString("East WEST WEST", 0, 6);
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
					GamePlan.lcd.drawString("West SOUTH NORTH", 0, 6);
					goToSRUpperLeft();
					goToSRLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the west bridge entry
					// go west a bit
					GamePlan.lcd.drawString("West SOUTH SOUTH", 0, 6);
					goToSRLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is north of the west bridge entry
					// go south by the east bound and then west
					GamePlan.lcd.drawString("West SOUTH EAST", 0, 6);
					goToSRLowerRight();
					goToSRLowerLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the west bridge entry
					// go south by the west bound
					GamePlan.lcd.drawString("West SOUTH WEST", 0, 6);
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
					GamePlan.lcd.drawString("West NORTH NORTH", 0, 6);
					goToSRLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the west bridge entry
					// go north by the west bound
					GamePlan.lcd.drawString("West NORTH SOUTH", 0, 6);
					goToSRLowerLeft();
					goToSRUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the west bridge entry
					// go for it
					GamePlan.lcd.drawString("West NORTH EAST", 0, 6);
					break;
				case WEST:
					// robot is west of the search zone, which is south of the west bridge entry
					// go north by the west bound
					GamePlan.lcd.drawString("West NORTH WEST", 0, 6);
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
					GamePlan.lcd.drawString("West EAST NORTH", 0, 6);
					goToSRUpperRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the west bridge entry
					// go east
					GamePlan.lcd.drawString("West EAST SOUTH", 0, 6);
					goToSRLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the west bridge entry
					// rush for it
					GamePlan.lcd.drawString("West EAST EAST", 0, 6);
					break;
				case WEST:
					// robot is west of the search zone, which is west of the west bridge entry
					GamePlan.lcd.drawString("West EAST WEST", 0, 6);
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
					GamePlan.lcd.drawString("South North NORTH", 0, 6);
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the south bridge entry
					// go north using the west bound
					GamePlan.lcd.drawString("South North SOUTH", 0, 6);
					goToSRLowerLeft();
					goToSRUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the south bridge entry
					// go north by the east bound
					GamePlan.lcd.drawString("South North EAST", 0, 6);
					goToSRUpperRight();
					break;
				case WEST:
					// robot is west of the search zone, which is south of the south bridge entry
					// go north by the west bound
					GamePlan.lcd.drawString("South North WEST", 0, 6);
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
					GamePlan.lcd.drawString("South EAST NORTH", 0, 6);
					goToSRUpperRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the south bridge entry
					// go north using the east bound
					GamePlan.lcd.drawString("South EAST SOUTH", 0, 6);
					goToSRLowerRight();
					goToSRUpperRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the south bridge entry
					// rush for it
					GamePlan.lcd.drawString("South EAST EAST", 0, 6);
					break;
				case WEST:
					// robot is west of the search zone, which is west of the south bridge entry
					// go north by the west bound and then east
					GamePlan.lcd.drawString("South EAST WEST", 0, 6);
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
					GamePlan.lcd.drawString("South WEST NORTH", 0, 6);
					goToSRUpperLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the south bridge entry
					// go north using the west bound
					GamePlan.lcd.drawString("South WEST SOUTH", 0, 6);
					goToSRLowerLeft();
					goToSRUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the south bridge entry
					// go north by the east bound and then west
					GamePlan.lcd.drawString("South WEST EAST", 0, 6);
					goToSRUpperRight();
					goToSRUpperLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is east of the south bridge entry
					// rush for it
					GamePlan.lcd.drawString("South WEST WEST", 0, 6);
					break;
				}
				break;
			}
			break;
		case CENTER:
			//cannot avoid SR to the go to center of bridge
			//so don't do anything
			break;
		}
		//go to the bridge and localize
		localizeBeforeBridge(direction);
	}

	/**
	 * Procedure method to avoid the search zone and water areas to go to the
	 * specified side of the tunnel.
	 * 
	 * WARNING: this method contains a lot of cases which are all the cases
	 * 			possible for the configuration of the entry point of the tunne;
	 * 			the search zone in the green zone and the position of the robot at 
	 * 			that time.
	 * 			Keep in mind that it finds the shortest path to avoid the search zone,
	 * 			goes around by going to its corners: goToSGLowerLeft() ,
	 * 			goToSGLowerRight() , goToSGUpperLeft() , goToSGUpperRight() 
	 * 
	 * 			After, it will call the localizeBeforeTunnel(direction) method
	 * 			
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
					// goes south around by the east bound
					GamePlan.lcd.drawString("North SOUTH NORTH", 0, 4);
					goToSGLowerRight();
					goToSGLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the north tunnel entry
					// rush for it
					GamePlan.lcd.drawString("North SOUTH SOUTH", 0, 4);
					break;
				case EAST:
					// robot is east of the search zone, which is north of the north tunnel entry
					// go south by the east bound
					GamePlan.lcd.drawString("North SOUTH EAST", 0, 4);
					Sound.beepSequenceUp();
					Sound.beepSequence();
					goToSGLowerRight();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the north tunnel entry
					// go south by the west bound
					GamePlan.lcd.drawString("North SOUTH WEST", 0, 4);
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
					// goes south around by the west bound
					GamePlan.lcd.drawString("North EAST NORTH", 0, 4);
					goToSGLowerLeft();
					goToSGLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the north tunnel entry
					// go west
					GamePlan.lcd.drawString("North EAST SOUTH", 0, 4);
					goToSGLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the north tunnel entry
					// rush for it
					GamePlan.lcd.drawString("North EAST EAST", 0, 4);
					break;
				case WEST:
					// robot is west of the search zone, which is west of the north tunnel entry
					// go south by the west bound and then east
					GamePlan.lcd.drawString("North EAST WEST", 0, 4);
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
					// goes south around by the east bound
					GamePlan.lcd.drawString("North WEST NORTH", 0, 4);
					goToSGLowerRight();
					goToSGLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the north tunnel entry
					// go west
					GamePlan.lcd.drawString("North WEST SOUTH", 0, 4);
					goToSGLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the north tunnel entry
					// go south by the east bound and then west
					GamePlan.lcd.drawString("North WEST EAST", 0, 4);
					goToSGLowerRight();
					goToSGLowerLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is east of the north tunnel entry
					// rush for it
					GamePlan.lcd.drawString("North WEST WEST", 0, 4);
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
					GamePlan.lcd.drawString("EAST SOUTH NORTH", 0, 4);
					goToSGUpperRight();
					goToSGLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the east tunnel entry
					// go east a bit
					GamePlan.lcd.drawString("EAST SOUTH SOUTH", 0, 4);
					goToSGLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is north of the east tunnel entry
					// go south by the east bound
					GamePlan.lcd.drawString("EAST SOUTH EAST", 0, 4);
					goToSGLowerRight();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the east tunnel entry
					// go south by the west bound and then east
					GamePlan.lcd.drawString("EAST SOUTH WEST", 0, 4);
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
					GamePlan.lcd.drawString("EAST NORTH NORTH", 0, 4);
					goToSGLowerRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the east tunnel entry
					// go north by the east bound
					GamePlan.lcd.drawString("EAST NORTH SOUTH", 0, 4);
					goToSGLowerRight();
					goToSGUpperRight();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the east tunnel entry
					// go north using east bound
					GamePlan.lcd.drawString("EAST SOUTH EAST", 0, 4);
					goToSGUpperRight();
					break;
				case WEST:
					// robot is west of the search zone, which is south of the east tunnel entry
					// go north by the west bound and then east
					GamePlan.lcd.drawString("EAST SOUTH WEST", 0, 4);
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
					GamePlan.lcd.drawString("EAST WEST NORTH", 0, 4);
					goToSGUpperLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the east tunnel entry
					// go west
					GamePlan.lcd.drawString("EAST WEST SOUTH", 0, 4);
					goToSGLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the east tunnel entry
					GamePlan.lcd.drawString("EAST WEST EAST", 0, 4);
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
					GamePlan.lcd.drawString("EAST WEST WEST", 0, 4);
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
					GamePlan.lcd.drawString("WEST SOUTH NORTH", 0, 4);
					goToSGUpperLeft();
					goToSGLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is north of the west tunnel entry
					// go west a bit
					GamePlan.lcd.drawString("WEST SOUTH SOUTH", 0, 4);
					goToSGLowerLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is north of the west tunnel entry
					// go south by the east bound and then west
					GamePlan.lcd.drawString("WEST SOUTH EAST", 0, 4);
					goToSGLowerRight();
					goToSGLowerLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is north of the west tunnel entry
					// go south by the west bound
					GamePlan.lcd.drawString("WEST SOUTH WEST", 0, 4);
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
					GamePlan.lcd.drawString("WEST NORTH NORTH", 0, 4);
					goToSGLowerLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the west tunnel entry
					// go north by the west bound
					GamePlan.lcd.drawString("WEST NORTH SOUTH", 0, 4);
					goToSGLowerLeft();
					goToSGUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the west tunnel entry
					// go for it
					GamePlan.lcd.drawString("WEST NORTH EAST", 0, 4);
					break;
				case WEST:
					// robot is west of the search zone, which is south of the west tunnel entry
					// go north by the west bound
					GamePlan.lcd.drawString("WEST NORTH WEST", 0, 4);
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
					GamePlan.lcd.drawString("WEST EAST NORTH", 0, 4);
					goToSGUpperRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the west tunnel entry
					// go east
					GamePlan.lcd.drawString("WEST EAST SOUTH", 0, 4);
					goToSGLowerRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the west tunnel entry
					// rush for it
					GamePlan.lcd.drawString("WEST EAST EAST", 0, 4);
					break;
				case WEST:
					// robot is west of the search zone, which is west of the west tunnel entry
					GamePlan.lcd.drawString("WEST EAST WEST", 0, 4);
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
					GamePlan.lcd.drawString("SOUTH NORTH NORTH", 0, 4);
					break;
				case SOUTH:
					// robot is south of the search zone, which is south of the south tunnel entry
					// go north using the west bound
					GamePlan.lcd.drawString("SOUTH NORTH SOUTH", 0, 4);
					goToSGLowerLeft();
					goToSGUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is south of the south tunnel entry
					// go north by the east bound
					GamePlan.lcd.drawString("SOUTH NORTH EAST", 0, 4);
					goToSGUpperRight();
					break;
				case WEST:
					// robot is west of the search zone, which is south of the south tunnel entry
					// go north by the west bound
					GamePlan.lcd.drawString("SOUTH NORTH WEST", 0, 4);
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
					GamePlan.lcd.drawString("SOUTH EAST NORTH", 0, 4);
					goToSGUpperRight();
					break;
				case SOUTH:
					// robot is south of the search zone, which is west of the south tunnel entry
					// go north using the east bound
					GamePlan.lcd.drawString("SOUTH EAST SOUTH", 0, 4);
					goToSGLowerRight();
					goToSGUpperRight();
					break;
				case EAST:
					// robot is east of the search zone, which is west of the south tunnel entry
					// rush for it
					GamePlan.lcd.drawString("SOUTH EAST EAST", 0, 4);
					break;
				case WEST:
					// robot is west of the search zone, which is west of the south tunnel entry
					// go north by the west bound and then east
					GamePlan.lcd.drawString("SOUTH EAST WEST", 0, 4);
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
					GamePlan.lcd.drawString("SOUTH WEST NORTH", 0, 4);
					goToSGUpperLeft();
					break;
				case SOUTH:
					// robot is south of the search zone, which is east of the south tunnel entry
					// go north using the west bound
					GamePlan.lcd.drawString("SOUTH WEST SOUTH", 0, 4);
					goToSGLowerLeft();
					goToSGUpperLeft();
					break;
				case EAST:
					// robot is east of the search zone, which is east of the south tunnel entry
					// go north by the east bound and then west
					GamePlan.lcd.drawString("SOUTH WEST EAST", 0, 4);
					goToSGUpperRight();
					goToSGUpperLeft();
					break;
				case WEST:
					// robot is west of the search zone, which is east of the south tunnel entry
					// rush for it
					GamePlan.lcd.drawString("SOUTH WEST WEST", 0, 4);
					break;
				}
				break;
			}
			break;
		case CENTER:
			//cannot avoid SR to the go to center of bridge
			//so don't do anything
			break;
		}
		//finally localize and go to the tunnel
		localizeBeforeTunnel(direction);
	}

	/**
	 * Procedure method to avoid the search zone and water areas and go to the
	 * starting corner.
	 * 
	 * WARNING: this method contains a lot of cases which are all the cases
	 * 			possible for the configuration of the search zone in the 
	 * 			zone of the robot and the position of the robot at 
	 * 			that time.
	 * 			Keep in mind that it finds the shortest path to avoid the search zone
	 * 			of its team color, goes around by going to its corners:
	 * 			RED: goToSRLowerLeft() ,
	 * 			goToSRLowerRight() , goToSRUpperLeft() , goToSRUpperRight() 
	 * 
	 * 			GREEN:
	 * 			goToSGLowerLeft() ,
	 * 			goToSGLowerRight() , goToSGUpperLeft() , goToSGUpperRight() 
	 * 
	 * 			After, it will directly travel to where it started (in its corner)
	 * 			
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
	 * red zone The robot will be a quarter of a tile outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSRLowerLeft() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SR_LL_x) -0.25)* Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SR_LL_y) -0.25)* Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the lower right corner of the search zone in the
	 * red zone The robot will be a quarter of a tile outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSRLowerRight() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SR_UR_x) +0.25) * Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SR_LL_y) -0.25)* Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the upper left corner of the search zone in the
	 * red zone The robot will be a quarter of a tile outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSRUpperLeft() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SR_LL_x)-0.25) * Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SR_UR_y)+0.25) * Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the upper right corner of the search zone in the
	 * red zone The robot will be a quarter of a tile outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSRUpperRight() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SR_UR_x) +0.25)* Navigation.TILE_SIZE,
				(serverData.getCoordParam(CoordParameter.SR_UR_y) +0.25)* Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the lower left corner of the search zone in the
	 * green zone The robot will be a quarter of a tile outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSGLowerLeft() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SG_LL_x) -0.25) * Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SG_LL_y) -0.25)* Navigation.TILE_SIZE);
		return true;
	}

	/**
	 * Makes the robot navigate to the lower right corner of the search zone in the
	 * green zone The robot will be a quarter of a tile outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSGLowerRight() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SG_UR_x) +0.25)* Navigation.TILE_SIZE  ,
				(serverData.getCoordParam(CoordParameter.SG_LL_y) -0.25) * Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the upper left corner of the search zone in the
	 * green zone The robot will be a quarter of a tile outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSGUpperLeft() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SG_LL_x) -0.25)* Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SG_UR_y) +0.25)* Navigation.TILE_SIZE );
		return true;
	}

	/**
	 * Makes the robot navigate to the upper right corner of the search zone in the
	 * green zone The robot will be a quarter of a tile outside of the zone
	 * 
	 * @return True when reached
	 */
	private boolean goToSGUpperRight() {
		navigation.travelTo(
				(serverData.getCoordParam(CoordParameter.SG_UR_x) +0.25) * Navigation.TILE_SIZE ,
				(serverData.getCoordParam(CoordParameter.SG_UR_y) +0.25)* Navigation.TILE_SIZE );
		return true;
	}
	
}