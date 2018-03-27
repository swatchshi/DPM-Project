package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.EV3WifiClient.CoordParameter;
import ca.mcgill.ecse211.lab5.EV3WifiClient.Zone;
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
	 * Object to help with how the Light sensor at the head of the robot detects
	 * colors
	 */
	private ColorSensor cSensor;
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
	 * Object in charge of displaying values periodically
	 */
	private Display odometryDisplay;
	/**
	 * Object in charge of correcting the trajectory
	 */
	private OdometerCorrection odoCorrect;
	public Direction direction;

	/**
	 * Creates an object of the GamePlan class. Initializes all instances needed in
	 * the game
	 * 
	 * @throws Exception
	 *             error with Odometer instances or with data server retrieval
	 */
	public GamePlan() throws Exception {

		cSensor = new ColorSensor(armSensor);
		lSensor = new ColorSensor(lightSensor);
		ultraSensor = new UltrasonicSensor(ultraSSensor);
		gyroscope = new Gyroscope(gyroSensor);
		// track related object
		dynamicTrack = new TrackExpansion();
		dynamicTrack.setDesignConstants(robot); // sets the values for the chosen robot

		// Odometer related objects
		odometer = Odometer.getOdometer(leftMotor, rightMotor, dynamicTrack, gyroscope, CONFIG);
		odometryDisplay = new Display(lcd, ultraSensor, gyroscope);
		odoCorrect = new OdometerCorrection(lSensor, odometer, dynamicTrack);

		navigation = new Navigation(odometer, dynamicTrack, CONFIG);
		serverData = new EV3WifiClient(); //////////////////////////////////////////// uncomment to enable data
											//////////////////////////////////////////// retrieval

		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();
		Thread odoCorrectionThread = new Thread(odoCorrect);
		odoCorrectionThread.start();
		odoCorrect.setDoCorrection(false);
		odometer.setDoThetaCorrection(false);
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
	 * Procedure to determine what Team color plan should be followed
	 * 
	 * @throws Exception
	 *             Exception thrown if the robot is not playing
	 */
	public void play() throws Exception {
		
		switch (serverData.getTeamColor()) {
		case RED:

			redPlan();
			break;
		case GREEN:
			greenPlan();
			break;
		}
	
		// TODO victory tune
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
		int corner=serverData.getStartingCorner();

		
		
		USLocalizer usLoc = new USLocalizer(odometer, navigation, ultraSensor);
		
		usLoc.doLocalization(0); 
		navigation.turnTo(0);
		
		
		Sound.beepSequenceUp();
		
		LightLocalizer lightLoc = new LightLocalizer(navigation, dynamicTrack, lSensor, odometer, gyroscope);
		lightLoc.lightloc(corner);
		///////////////////////////////////////////////////////////
		Sound.beep();
		
		//System.exit(0);
		/////////////////////////////////////////////////////////////////////////////
		goToBridge(getBridgeEntry());
		
		
		crossBridge();
		Sound.beepSequenceUp();

		
		//goToTunnel(getTunnelEntry());
		goToTunnel(directionSwitch(getBridgeEntry()));
		crossTunnel();
		
		
		goToStartingCorner();
		System.exit(0);
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
		int corner=serverData.getStartingCorner();

		
		
		USLocalizer usLoc = new USLocalizer(odometer, navigation, ultraSensor);
		
		usLoc.doLocalization(0); 
		navigation.turnTo(0);
		
		
		Sound.beepSequenceUp();
		
		LightLocalizer lightLoc = new LightLocalizer(navigation, dynamicTrack, lSensor, odometer, gyroscope);
		lightLoc.lightloc(corner);
		///////////////////////////////////////////////////////////
		Sound.beepSequenceUp();
		Button.waitForAnyPress();
		/////////////////////////////////////////////////////////////////////////////
		goToTunnel(getTunnelEntry());
		Button.waitForAnyPress();
		crossTunnel();
		Button.waitForAnyPress();
		Sound.beepSequenceUp();
		
		goToBridge(directionSwitch(getTunnelEntry()));
		Button.waitForAnyPress();
		crossBridge();
		Button.waitForAnyPress();
		
		odometer.correctAngle();
		goToStartingCorner();
	}

	/**
	 * Procedure to cross the bridge
	 * @return 
	 * 
	 * @return True when the bridge has been crossed
	 * @throws Exception
	 *             if the specified entry point is incorrect
	 */
	
	public Direction directionSwitch(Direction direction) {
		this.direction = direction;
		if (direction == Direction.NORTH) {
			return Direction.SOUTH;
		}
		else if (direction == Direction.SOUTH) {
			return Direction.NORTH;			
		}
		else if (direction == Direction.WEST) {
			return Direction.EAST;
		}
		else if (direction == Direction.EAST) {
			return Direction.WEST;
		}
		else {
			Sound.buzz();
			Sound.buzz();
			return null;
		}
		
	}
	
	
	
	
	
	private boolean crossBridge() throws Exception { // expansion method, travel directly
		navigation.travel(navigation.TILE_SIZE * (1 + serverData.getBridgeWidth(getBridgeEntry())));
		// travels the width of the bridge plus an extra tile
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
		navigation.travel(navigation.TILE_SIZE * (1 + serverData.getTunnelWidth(getTunnelEntry())));
		// travels the width of the tunnel plus an extra tile
		return true;
	}

	/**
	 * Finds the entry side the robot has to take in order to cross the bridge
	 * Returns Direction.CENTER if unable to find the entrance.
	 * 
	 * @return The Direction (side) which the robot needs to enter the bridge
	 * 
	 * @throws Exception
	 *             Exception thrown if the robot is not playing or a problem with the zone
	 */
	private Direction getBridgeEntry() throws Exception {
		double lowerLeftX = serverData.getCoordParam(CoordParameter.BR_LL_x) * Navigation.TILE_SIZE;
		double lowerLeftY = serverData.getCoordParam(CoordParameter.BR_LL_y) * Navigation.TILE_SIZE;
		double upperRightX = serverData.getCoordParam(CoordParameter.BR_UR_x) * Navigation.TILE_SIZE;
		double upperRightY = serverData.getCoordParam(CoordParameter.BR_UR_y) * Navigation.TILE_SIZE;
		
		
		//look North of the bridge, to see what zone is there, desired zone is the RED zone
		switch(serverData.getZone(lowerLeftX + (upperRightX-lowerLeftX)/2, upperRightY+Navigation.TILE_SIZE/2)) {
		case BRIDGE:
		case TUNNEL:
			throw new Exception("Error in the zone for bridge entry");
		case GREEN:
		case SG:
			//entry in RED zone is in the south because GREEN entry is in the North
			return Direction.SOUTH;
		case RED:
		case SR:
			//entry in RED zone is in the North
			return Direction.NORTH;
		case WATER:
			//look EAST of the bridge, to see what zone is there
			switch(serverData.getZone(upperRightX+Navigation.TILE_SIZE/2, lowerLeftY+ (upperRightY-lowerLeftY)/2)) {
			case BRIDGE:
			case TUNNEL:
			case WATER:
				throw new Exception("Error in the zone for bridge entry");
			case GREEN:
			case SG:
				//EAST entry is the GREEN entry point
				return Direction.WEST;
			case RED:
			case SR:
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
		double lowerLeftX = serverData.getCoordParam(CoordParameter.TN_LL_x) * Navigation.TILE_SIZE;
		double lowerLeftY = serverData.getCoordParam(CoordParameter.TN_LL_y) * Navigation.TILE_SIZE;
		double upperRightX = serverData.getCoordParam(CoordParameter.TN_UR_x) * Navigation.TILE_SIZE;
		double upperRightY = serverData.getCoordParam(CoordParameter.TN_UR_y) * Navigation.TILE_SIZE;
		
		
		//look North of the tunnel, to see what zone is there, desired zone is the GREEN zone
		switch(serverData.getZone(lowerLeftX + (upperRightX-lowerLeftX)/2, upperRightY+Navigation.TILE_SIZE/2)) {
		case BRIDGE:
		case TUNNEL:
			throw new Exception("Error in the zone for tunnel entry");
		case GREEN:
		case SG:
			//entry in GREEN zone is in the North 
			return Direction.NORTH;
		case RED:
		case SR:
			//entry in GREEN zone is in the South because RED entry is in the North
			return Direction.SOUTH;
		case WATER:
			//look EAST of the tunnel, to see what zone is there
			switch(serverData.getZone(upperRightX+Navigation.TILE_SIZE/2, lowerLeftY+ (upperRightY-lowerLeftY)/2)) {
			case BRIDGE:
			case TUNNEL:
			case WATER:
				throw new Exception("Error in the zone for tunnel entry");
			case GREEN:
			case SG:
				//EAST entry is the GREEN entry point
				return Direction.EAST;
			case RED:
			case SR:
				return Direction.WEST;
			}
			break;
		}
		//default
		return Direction.CENTER;
	}

	/**
	 * Procedure method to avoid the search zone and water areas and go to the
	 * specified side of the bridge.
	 * 
	 * @param direction
	 *            Direction (side) of the bridge entry
	 * @return True when reached
	 * @throws Exception
	 *             When there is a problem with the data from the EV3WifiClass or
	 *             with the entry point
	 */
	private void goToBridge(Direction direction) throws Exception {
		double lowerLeftX = serverData.getCoordParam(CoordParameter.BR_LL_x) * Navigation.TILE_SIZE;
		double lowerLeftY = serverData.getCoordParam(CoordParameter.BR_LL_y) * Navigation.TILE_SIZE;
		double upperRightX = serverData.getCoordParam(CoordParameter.BR_UR_x) * Navigation.TILE_SIZE;
		double upperRightY = serverData.getCoordParam(CoordParameter.BR_UR_y) * Navigation.TILE_SIZE;

		switch (direction) {
		// Entrance is in the North
		case NORTH:
			// entrance of the bridge is on the North side
			// avoid SR
			switch (serverData.getSide(EV3WifiClient.Zone.SR, lowerLeftX + (upperRightX - lowerLeftX) / 2,
					upperRightY)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case NORTH:
				// the search zone is south of the north bridge entry
				// very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to bridge entrance");
			case SOUTH:
				// the search zone is north of the north bridge entry
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

			// finally rush to North entrance of the bridge
			navigation.travelTo(lowerLeftX + (upperRightX - lowerLeftX) / 2, upperRightY + Navigation.TILE_SIZE / 2);
			navigation.turnTo(180);
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
			// finally rush to East entrance of the bridge
			navigation.travelTo(upperRightX + Navigation.TILE_SIZE / 2, lowerLeftY + (upperRightY - lowerLeftY) / 2);
			navigation.turnTo(270);
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
			// finally rush to West entrance of the bridge
			navigation.travelTo(lowerLeftX - Navigation.TILE_SIZE / 2, lowerLeftY + (upperRightY - lowerLeftY) / 2);
			navigation.turnTo(90);
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

			// finally rush to South entrance of the bridge
			navigation.travelTo(lowerLeftX + (upperRightX - lowerLeftX) / 2, lowerLeftY - Navigation.TILE_SIZE / 2);
			navigation.turnTo(0);
			break;
		}
	}

	/**
	 * Procedure method to avoid the search zone and water areas and go to the
	 * specified side of the tunnel.
	 * 
	 * @param direction
	 *            Direction (side) of the tunnel entry
	 * @return True when reached
	 * @throws Exception
	 *             When there is a problem with the data from the EV3WifiClass or
	 *             with the entry point
	 */
	private void goToTunnel(Direction direction) throws Exception {
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

			// finally rush to North entrance of the tunnel
			navigation.travelTo(lowerLeftX + (upperRightX - lowerLeftX) / 2, upperRightY + Navigation.TILE_SIZE / 2);
			navigation.turnTo(180);
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
			// finally rush to East entrance of the tunnel
			navigation.travelTo(upperRightX + Navigation.TILE_SIZE / 2, lowerLeftY + (upperRightY - lowerLeftY) / 2);
			navigation.turnTo(270);
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
			// finally rush to West entrance of the tunnel
			navigation.travelTo(lowerLeftX - Navigation.TILE_SIZE / 2, lowerLeftY + (upperRightY - lowerLeftY) / 2);
			navigation.turnTo(90);
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

			// finally rush to South entrance of the tunnel
			navigation.travelTo(lowerLeftX + (upperRightX - lowerLeftX) / 2, lowerLeftY - Navigation.TILE_SIZE / 2);
			navigation.turnTo(0);
			break;
		}

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
		double startingX = 0, startingY = 0;
		switch (serverData.getStartingCorner()) {
		case 0:
			startingX = Navigation.TILE_SIZE ;//
			startingY = Navigation.TILE_SIZE ;
			break;
		case 1:
			startingX = EV3WifiClient.X_GRID_LINES * Navigation.TILE_SIZE - Navigation.TILE_SIZE ;
			startingY = Navigation.TILE_SIZE ;
			break;
		case 2:
			startingX = EV3WifiClient.X_GRID_LINES * Navigation.TILE_SIZE - Navigation.TILE_SIZE;
			startingY = EV3WifiClient.Y_GRID_LINES * Navigation.TILE_SIZE - Navigation.TILE_SIZE ;
			break;
		case 3:
			startingX = Navigation.TILE_SIZE;
			startingY = EV3WifiClient.Y_GRID_LINES * Navigation.TILE_SIZE - Navigation.TILE_SIZE ;
			break;
		}
		switch (serverData.getTeamColor()) {
		case GREEN:
			switch (serverData.getSide(Zone.SG, startingX, startingY)) {
			case CENTER:
				throw new Exception("Starting corner in center of search zone");
				// Since rectangle, always east or west
			case EAST:
				// starting corner is east of SG zone
				if (odometer.getY() < startingY) {
					// robot is south of starting corner
					switch (serverData.getSide(Zone.SG, odometer.getX(), odometer.getY())) {
					case CENTER:
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
						goToSGLowerRight();
						break;
					case WEST:
						// robot west of the search zone, which is south west of the starting corner
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
						goToSGUpperRight();
						break;
					case WEST:
						// robot west of the search zone, which is north west of the starting corner
						goToSGLowerLeft();
						goToSGLowerRight();
						break;
					}
				}

				break;
			case WEST:
				// starting corner is west of SG zone
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
						// rush for it
						break;
					case SOUTH:
						// robot south of the search zone, which is south east of the starting corner
						goToSGLowerLeft();
						break;
					case WEST:
						// robot west of the search zone, which is south east of the starting corner
						goToSGUpperRight();
						goToSGUpperLeft();
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
						// rush for it
						break;
					case SOUTH:
						// robot south of the search zone, which is north east of the starting corner
						goToSGUpperLeft();
						break;
					case WEST:
						// robot west of the search zone, which is north east of the starting corner
						goToSGLowerRight();
						goToSGLowerLeft();
						break;
					}
				}

				break;
			}
			break;
		case RED:
			// red starting corner
			switch (serverData.getSide(Zone.SR, startingX, startingY)) {
			case CENTER:
				throw new Exception("Starting corner in center of search zone");
				// Since rectangle, always east or west
			case EAST:
				// starting corner is east of SR zone
				if (odometer.getY() < startingY) {
					// robot is south of starting corner
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
						goToSRLowerRight();
						break;
					case WEST:
						// robot west of the search zone, which is south west of the starting corner
						goToSRUpperLeft();
						goToSRUpperRight();
						break;
					}

				} else {
					// robot is north of starting corner
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
						goToSRUpperRight();
						break;
					case WEST:
						// robot west of the search zone, which is north west of the starting corner
						goToSRLowerLeft();
						goToSRLowerRight();
						break;
					}
				}

				break;
			case WEST:
				// starting corner is west of SR zone
				if (odometer.getY() < startingY) {
					// robot is south of starting corner
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
						// rush for it
						break;
					case SOUTH:
						// robot south of the search zone, which is south east of the starting corner
						goToSRLowerLeft();
						break;
					case WEST:
						// robot west of the search zone, which is south east of the starting corner
						goToSRUpperRight();
						goToSRUpperLeft();
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
						// rush for it
						break;
					case SOUTH:
						// robot south of the search zone, which is north east of the starting corner
						goToSRUpperLeft();
						break;
					case WEST:
						// robot west of the search zone, which is north east of the starting corner
						goToSRLowerRight();
						goToSRLowerLeft();
						break;
					}
				}

				break;
			}
			break;
		}
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
				serverData.getCoordParam(CoordParameter.SR_LL_x) * Navigation.TILE_SIZE - dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SR_LL_y) * Navigation.TILE_SIZE - dynamicTrack.getTrack());
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
				serverData.getCoordParam(CoordParameter.SR_UR_x) * Navigation.TILE_SIZE + dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SR_LL_y) * Navigation.TILE_SIZE - dynamicTrack.getTrack());
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
				serverData.getCoordParam(CoordParameter.SR_LL_x) * Navigation.TILE_SIZE - dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SR_UR_y) * Navigation.TILE_SIZE + dynamicTrack.getTrack());
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
				serverData.getCoordParam(CoordParameter.SR_UR_x) * Navigation.TILE_SIZE + dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SR_UR_y) * Navigation.TILE_SIZE + dynamicTrack.getTrack());
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
				serverData.getCoordParam(CoordParameter.SG_LL_x) * Navigation.TILE_SIZE - dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SG_LL_y) * Navigation.TILE_SIZE - dynamicTrack.getTrack());
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
				serverData.getCoordParam(CoordParameter.SG_UR_x) * Navigation.TILE_SIZE + dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SG_LL_y) * Navigation.TILE_SIZE - dynamicTrack.getTrack());
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
				serverData.getCoordParam(CoordParameter.SG_LL_x) * Navigation.TILE_SIZE - dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SG_UR_y) * Navigation.TILE_SIZE + dynamicTrack.getTrack());
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
				serverData.getCoordParam(CoordParameter.SG_UR_x) * Navigation.TILE_SIZE + dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SG_UR_y) * Navigation.TILE_SIZE + dynamicTrack.getTrack());
		return true;
	}

}