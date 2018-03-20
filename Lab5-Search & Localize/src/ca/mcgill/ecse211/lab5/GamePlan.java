package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.EV3WifiClient.CoordParameter;
import ca.mcgill.ecse211.localizer.*;
import ca.mcgill.ecse211.odometer.*;
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
	 * CENTER: in the middle
	 */
	public enum Direction {
		NORTH, EAST, SOUTH, WEST, CENTER
	}

	private Odometer odometer;
	private Navigation navigation;
	private TrackExpansion dynamicTrack;
	private ColorSensor cSensor;
	private ColorSensor lSensor;
	private UltrasonicSensor ultraSensor;
	private Gyroscope gyroscope;
	private EV3WifiClient serverData;
	private Display odometryDisplay;
	private OdometerCorrection odoCorrect;
	
    private boolean player;  //green = true, red = false;
	
	
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
		gyroscope = new Gyroscope(gyroSensor);
		//track related object
		dynamicTrack = new TrackExpansion();
		dynamicTrack.setDesignConstants(robot); //sets the values for the chosen robot
		
		// Odometer related objects
		odometer = Odometer.getOdometer(leftMotor, rightMotor, dynamicTrack, gyroscope, CONFIG);
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
	 * @throws Exception Exception thrown if the robot is not playing
	 */
	public void play() throws Exception {
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();
		Thread odoCorrectionThread=new Thread(odoCorrect);
		odoCorrectionThread.start();
		
		
		
		//testing slot
		
		
		
		/*switch(serverData.getTeamColor()) {
		case RED:
			redPlan();
			break;
		case GREEN:
			greenPlan();
			break;
		}
		*/
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
			lightLoc.doLocalization(1, 1, 0);
			break;
		case 1:
			lightLoc.doLocalization(EV3WifiClient.X_GRID_LINES-1, 1, 1);
			break;
		case 2:
			lightLoc.doLocalization(EV3WifiClient.X_GRID_LINES-1, serverData.Y_GRID_LINES-1, 2);
			break;
		case 3:
			lightLoc.doLocalization(1, EV3WifiClient.Y_GRID_LINES-1, 3);
			break;
		}
		goToBridge(getBridgeEntry());
		crossBridge();
		
		//TODO: look for flag
		
		
		goToTunnel(getTunnelEntry());
		crossTunnel();
		goToStartingCorner();
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
	 * 
	 * @throws Exception When there is a problem with the data from the EV3WifiClass
	 */
	private void greenPlan() throws Exception {
		USLocalizer usLoc=new USLocalizer(odometer, navigation, ultraSensor);
		usLoc.doLocalization(serverData.getStartingCorner());
		LightLocalizer lightLoc=new LightLocalizer(navigation, dynamicTrack, lSensor, odometer, CONFIG);
		switch(serverData.getStartingCorner()) {
		case 0:
			lightLoc.doLocalization(1, 1, 0);
			break;
		case 1:
			lightLoc.doLocalization(EV3WifiClient.X_GRID_LINES-1, 1, 1);
			break;
		case 2:
			lightLoc.doLocalization(EV3WifiClient.X_GRID_LINES-1, serverData.Y_GRID_LINES-1, 2);
			break;
		case 3:
			lightLoc.doLocalization(1, EV3WifiClient.Y_GRID_LINES-1, 3);
			break;
		}
		
		
		
		goToTunnel(getTunnelEntry());
		crossTunnel();
		
		//TODO: look for flag
		
		goToBridge(getBridgeEntry());
		crossBridge();
		
		goToStartingCorner();
	}

	/**
	 * Procedure to cross the bridge
	 * 
	 * @return True when the bridge has been crossed
	 */
	private boolean crossBridge() {   //expansion method, travel directly
		navigation.travel(navigation.TILE_SIZE*2);
	      leftMotor.stop(true);
	      rightMotor.stop(false);
		
		// TODO call the procedure for bridge traversal
		return true;
	}

	/**
	 * Procedure to cross the tunnel
	 * 
	 * @return True when the tunnel has been crossed
	 */
	private boolean crossTunnel() {
	      navigation.travel(navigation.TILE_SIZE*2);
	      leftMotor.stop(true);
	      rightMotor.stop(false);
		// TODO call the procedure for tunnel traversal
		return true;
	}

	
	/**
	 * Finds the entry side the robot 
	 * has to take in order to cross the bridge
	 * 
	 * @return The Direction (side) which the 
	 * 			robot needs to enter the bridge
	 * 
	 * @throws Exception Exception thrown if the robot is not playing
	 */
	private Direction getBridgeEntry() throws Exception {
		
		if(player == true) {//green
			return Direction.SOUTH;
		}
		else if(player == false) {//red
			return Direction.NORTH;
		}
		return null;
	}

	/**
	 * Finds the entry side the robot 
	 * has to take in order to cross the tunnel
	 * 
	 * @return The Direction (side) which the 
	 * 			robot needs to enter the tunnel
	 * @throws Exception When there is a problem with the data from the EV3WifiClass
	 */
	private Direction getTunnelEntry() throws Exception {
		switch(serverData.getTeamColor()) {
		case RED:
			//Need to find the green entrance
			break;
		case GREEN:
			//Need to find the green entrance
			break;
		}
		return Direction.EAST;
	}
	
	
	/**
	 * Procedure method to avoid the search zone and water areas 
	 * and go to the specified side of the bridge.
	 * 
	 * @param direction Direction (side) of the bridge entry
	 * @return True when reached
	 * @throws Exception When there is a problem with the data from the EV3WifiClass or with the entry point
	 */
	private void goToBridge(Direction direction) throws Exception {
		double lowerLeftX=serverData.getCoordParam(CoordParameter.BR_LL_x)*Navigation.TILE_SIZE;
		double lowerLeftY=serverData.getCoordParam(CoordParameter.BR_LL_y)*Navigation.TILE_SIZE;
		double upperRightX=serverData.getCoordParam(CoordParameter.BR_UR_x)*Navigation.TILE_SIZE;
		double upperRightY=serverData.getCoordParam(CoordParameter.BR_UR_y)*Navigation.TILE_SIZE;
		
		switch(direction) {
		//Entrance is in the North	
		case NORTH:
			//entrance of the bridge is on the North side
			//avoid SR
			switch(serverData.getSide(EV3WifiClient.Zone.SR, lowerLeftX+(upperRightX-lowerLeftX)/2, upperRightY)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case NORTH:
				//the search zone is south of the north bridge entry
				//very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to bridge entrance");
			case SOUTH:
				//the search zone is north of the north bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is north of the north bridge entry
					//goes south around by the west bound
					goToSRUpperLeft();
					goToSRLowerLeft();
					break;
				case SOUTH:
					//robot is south of the search zone, which is north of the north bridge entry
					//rush for it
					break;
				case EAST:
					//robot is east of the search zone, which is north of the north bridge entry
					//go south by the east bound
					goToSRLowerRight();
					break;
				case WEST:
					//robot is west of the search zone, which is north of the north bridge entry
					//go south by the west bound
					goToSRLowerLeft();
					break;
				}
				break;
			case EAST:
				//the search zone is west of the north bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is west of the north bridge entry
					//goes south around by the east bound
					goToSRUpperRight();
					goToSRLowerRight();
					break;
				case SOUTH:
					//robot is south of the search zone, which is west of the north bridge entry
					//go west
					goToSRLowerRight();
					break;
				case EAST:
					//robot is east of the search zone, which is west of the north bridge entry
					//rush for it
					break;
				case WEST:
					//robot is west of the search zone, which is west of the north bridge entry
					//go south by the west bound and then east
					goToSRLowerLeft();
					goToSRLowerRight();
					break;
				}
				break;
			case WEST:
				//the search zone is east of the north bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is east of the north bridge entry
					//goes south around by the west bound
					goToSRUpperLeft();
					goToSRLowerLeft();
					break;
				case SOUTH:
					//robot is south of the search zone, which is east of the north bridge entry
					//go west
					goToSRLowerLeft();
					break;
				case EAST:
					//robot is east of the search zone, which is east of the north bridge entry
					//go south by the east bound and then west
					goToSRLowerRight();
					goToSRLowerLeft();
					break;
				case WEST:
					//robot is west of the search zone, which is east of the north bridge entry
					//rush for it
					break;
				}
				break;
			}
			
			//finally rush to North entrance of the bridge
			navigation.travelTo(lowerLeftX+(upperRightX-lowerLeftX)/2, upperRightY+Navigation.TILE_SIZE/2);
			navigation.turnTo(180);
			break;
			
			
		//Entrance is in the East	
		case EAST:
			//entrance of the bridge is on the East side
			//avoid SR
			switch(serverData.getSide(EV3WifiClient.Zone.SR, upperRightX, lowerLeftY+(upperRightY-lowerLeftY)/2)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case EAST:
				//the search zone is west of the east bridge entry
				//very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to bridge entrance");
			case SOUTH:
				//the search zone is north of the east bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is north of the east bridge entry
					//goes south around by the east bound
					goToSRUpperRight();
					goToSRLowerRight();
					break;
				case SOUTH:
					//robot is south of the search zone, which is north of the east bridge entry
					//go east a bit
					goToSRLowerRight();
					break;
				case EAST:
					//robot is east of the search zone, which is north of the east bridge entry
					//go south by the east bound
					goToSRLowerRight();
					break;
				case WEST:
					//robot is west of the search zone, which is north of the east bridge entry
					//go south by the west bound and then east
					goToSRLowerLeft();
					goToSRLowerRight();
					break;
				}
				break;
			case NORTH:
				//the search zone is south of the east bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is south of the east bridge entry
					//go east a bit
					goToSRLowerRight();
					break;
				case SOUTH:
					//robot is south of the search zone, which is south of the east bridge entry
					//go north by the east bound
					goToSRLowerRight();
					goToSRUpperRight();
					break;
				case EAST:
					//robot is east of the search zone, which is south of the east bridge entry
					//go north using east bound
					goToSRUpperRight();
					break;
				case WEST:
					//robot is west of the search zone, which is south of the east bridge entry
					//go north by the west bound and then east
					goToSRUpperLeft();
					goToSRUpperRight();
					break;
				}
				break;
			case WEST:
				//the search zone is east of the east bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is east of the east bridge entry
					//goes west
					goToSRUpperLeft();
					break;
				case SOUTH:
					//robot is south of the search zone, which is east of the east bridge entry
					//go west
					goToSRLowerLeft();
					break;
				case EAST:
					//robot is east of the search zone, which is east of the east bridge entry
					
					if(lowerLeftY<odometer.getY()) {
						//go south by the east bound and then west
						goToSRLowerRight();
						goToSRLowerLeft();
					}else {
						goToSRUpperRight();
						goToSRUpperLeft();
					}
					break;
				case WEST:
					//robot is west of the search zone, which is east of the east bridge entry
					//rush for it
					break;
				}
				break;
			}
			//finally rush to East entrance of the bridge
			navigation.travelTo(upperRightX+Navigation.TILE_SIZE/2, lowerLeftY+(upperRightY-lowerLeftY)/2);
			navigation.turnTo(270);
			break;
		
		//Entrance is in the West	
		case WEST:
			//entrance of the bridge is on the West side
			//avoid SR
			switch(serverData.getSide(EV3WifiClient.Zone.SR, lowerLeftX, lowerLeftY+(upperRightY-lowerLeftY)/2)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case WEST:
				//the search zone is east of the west bridge entry
				//very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to bridge entrance");
			case SOUTH:
				//the search zone is north of the west bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is north of the west bridge entry
					//goes south around by the west bound
					goToSRUpperLeft();
					goToSRLowerLeft();
					break;
				case SOUTH:
					//robot is south of the search zone, which is north of the west bridge entry
					//go west a bit
					goToSRLowerLeft();
					break;
				case EAST:
					//robot is east of the search zone, which is north of the west bridge entry
					//go south by the east bound and then west
					goToSRLowerRight();
					goToSRLowerLeft();
					break;
				case WEST:
					//robot is west of the search zone, which is north of the west bridge entry
					//go south by the west bound
					goToSRLowerLeft();
					break;
				}
				break;
			case NORTH:
				//the search zone is south of the west bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is south of the west bridge entry
					//go west a bit
					goToSRLowerLeft();
					break;
				case SOUTH:
					//robot is south of the search zone, which is south of the west bridge entry
					//go north by the west bound
					goToSRLowerLeft();
					goToSRUpperLeft();
					break;
				case EAST:
					//robot is east of the search zone, which is south of the west bridge entry
					//go for it
					break;
				case WEST:
					//robot is west of the search zone, which is south of the west bridge entry
					//go north by the west bound
					goToSRUpperLeft();
					break;
				}
				break;
			case EAST:
				//the search zone is west of the west bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is west of the west bridge entry
					//goes east
					goToSRUpperRight();
					break;
				case SOUTH:
					//robot is south of the search zone, which is west of the west bridge entry
					//go east
					goToSRLowerRight();
					break;
				case EAST:
					//robot is east of the search zone, which is west of the west bridge entry
					//rush for it
					
					break;
				case WEST:
					//robot is west of the search zone, which is west of the west bridge entry
					if(lowerLeftY<odometer.getY()) {
						//go south by the west bound and then east
						goToSRLowerLeft();
						goToSRLowerRight();
					}else {
						goToSRUpperLeft();
						goToSRUpperRight();
					}
					break;
				}
				break;
			}
			//finally rush to West entrance of the bridge
			navigation.travelTo(lowerLeftX-Navigation.TILE_SIZE/2, lowerLeftY+(upperRightY-lowerLeftY)/2);
			navigation.turnTo(90);
			break;
			
			
		//Entrance is in the south	
		case SOUTH:
			//entrance of the bridge is on the South side
			//avoid SR
			switch(serverData.getSide(EV3WifiClient.Zone.SR, lowerLeftX+(upperRightX-lowerLeftX)/2, lowerLeftY)) {
			case CENTER:
				throw new Exception("Brigde entry in search zone");
			case SOUTH:
				//the search zone is north of the south bridge entry
				//very unlikely considering rectangular zones
				throw new Exception("No good trajectory to go to bridge entrance");
			case NORTH:
				//the search zone is south of the south bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//robot in center of search zone
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is south of the south bridge entry
					//rush for it
					break;
				case SOUTH:
					//robot is south of the search zone, which is south of the south bridge entry
					//go north using the west bound
					goToSRLowerLeft();
					goToSRUpperLeft();
					break;
				case EAST:
					//robot is east of the search zone, which is south of the south bridge entry
					//go north by the east bound
					goToSRUpperRight();
					break;
				case WEST:
					//robot is west of the search zone, which is south of the south bridge entry
					//go north by the west bound
					goToSRUpperLeft();
					break;
				}
				break;
			case EAST:
				//the search zone is west of the south bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is west of the south bridge entry
					//go east a bit
					goToSRUpperRight();
					break;
				case SOUTH:
					//robot is south of the search zone, which is west of the south bridge entry
					//go north using the east bound
					goToSRLowerRight();
					goToSRUpperRight();
					break;
				case EAST:
					//robot is east of the search zone, which is west of the south bridge entry
					//rush for it
					break;
				case WEST:
					//robot is west of the search zone, which is west of the south bridge entry
					//go north by the west bound and then east
					goToSRUpperLeft();
					goToSRUpperRight();
					break;
				}
				break;
			case WEST:
				//the search zone is east of the south bridge entry
				switch(serverData.getSide(EV3WifiClient.Zone.SR, odometer.getX(), odometer.getY())) {
				case CENTER:
					//rush for it
					break;
				case NORTH:
					//robot is north of the search zone, which is east of the south bridge entry
					//go west a bit
					goToSRUpperLeft();
					break;
				case SOUTH:
					//robot is south of the search zone, which is east of the south bridge entry
					//go north using the west bound
					goToSRLowerLeft();
					goToSRUpperLeft();
					break;
				case EAST:
					//robot is east of the search zone, which is east of the south bridge entry
					//go north by the east bound and then west
					goToSRUpperRight();
					goToSRUpperLeft();
					break;
				case WEST:
					//robot is west of the search zone, which is east of the south bridge entry
					//rush for it
					break;
				}
				break;
			}
			
			//finally rush to South entrance of the bridge
			navigation.travelTo(lowerLeftX+(upperRightX-lowerLeftX)/2, lowerLeftY-Navigation.TILE_SIZE/2);
			navigation.turnTo(0);
			break;
		}
	}
	
	/**
	 * Procedure method to avoid the search zone and water areas 
	 * and go to the specified side of the tunnel.
	 * 
	 * @param direction Direction (side) of the tunnel entry
	 * @return True when reached
	 * @throws Exception When there is a problem with the data from the EV3WifiClass or with the entry point
	 */
	private void goToTunnel(Direction direction) throws Exception {
		switch(direction) {
		case NORTH:
			navigation.travelTo(serverData.getCoordParam(CoordParameter.TN_UR_x)*Navigation.TILE_SIZE ,
					serverData.getCoordParam(CoordParameter.TN_UR_y)*Navigation.TILE_SIZE);
			
			navigation.turnTo(180); //faces south
			break;
		case EAST:
			navigation.travelTo(serverData.getCoordParam(CoordParameter.TN_UR_x)*Navigation.TILE_SIZE ,
					serverData.getCoordParam(CoordParameter.TN_UR_y)*Navigation.TILE_SIZE);

			navigation.turnTo(270); //faces west
			break;
		case SOUTH:
			navigation.travelTo(serverData.getCoordParam(CoordParameter.TN_LL_x)*Navigation.TILE_SIZE ,
					serverData.getCoordParam(CoordParameter.TN_LL_y)*Navigation.TILE_SIZE);
			navigation.turnTo(0); //faces north
			break;
		case WEST:
			navigation.travelTo(serverData.getCoordParam(CoordParameter.TN_LL_x)*Navigation.TILE_SIZE ,
					serverData.getCoordParam(CoordParameter.TN_LL_y)*Navigation.TILE_SIZE);
			
			navigation.turnTo(90); //faces east
			break;
			
		case CENTER:
			//entrance cannot be in the middle
			throw new Exception("Problem with the entry point of the tunnel");
		}
		
	}
	
	/**
	 * Procedure method to avoid the search zone and water areas 
	 * and go to the starting corner.
	 * 
	 * @return True when reached
	 */
	private boolean goToStartingCorner() {
		
		return true;
	}
	
	
	//////////////////////////////////////////////////////////
	private boolean goToSRLowerLeft() {
		navigation.travelTo(serverData.getCoordParam(CoordParameter.SR_LL_x)*Navigation.TILE_SIZE - dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SR_LL_y)*Navigation.TILE_SIZE - dynamicTrack.getTrack());
		return true;
	}
	private boolean goToSRLowerRight() {
		navigation.travelTo(serverData.getCoordParam(CoordParameter.SR_UR_x)*Navigation.TILE_SIZE + dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SR_LL_y)*Navigation.TILE_SIZE - dynamicTrack.getTrack());
		return true;
	}
	private boolean goToSRUpperLeft() {
		navigation.travelTo(serverData.getCoordParam(CoordParameter.SR_LL_x)*Navigation.TILE_SIZE - dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SR_UR_y)*Navigation.TILE_SIZE + dynamicTrack.getTrack());
		return true;
	}
	private boolean goToSRUpperRight() {
		navigation.travelTo(serverData.getCoordParam(CoordParameter.SR_UR_x)*Navigation.TILE_SIZE + dynamicTrack.getTrack(),
				serverData.getCoordParam(CoordParameter.SR_UR_y)*Navigation.TILE_SIZE + dynamicTrack.getTrack());
		return true;
	}
	
	/*
	private boolean goToSGLowerLeft() {
		return true;
	}
	private boolean goToSGLowerRight() {
		return true;
	}
	private boolean goToSGUpperLeft() {
		return true;
	}
	private boolean goToSGUpperRight() {
		return true;
	}
	*/
}