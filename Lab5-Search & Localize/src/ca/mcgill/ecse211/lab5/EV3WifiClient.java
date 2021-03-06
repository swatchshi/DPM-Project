package ca.mcgill.ecse211.lab5;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.localizer.ColorSensor;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 * Class using WifiConnection to communicate with a server and receive data
 * concerning the competition such as the starting corner the robot is placed
 * in.
 * 
 * There are two variables you **MUST** set manually before trying to use this
 * code.
 * 
 * 1. SERVER_IP: The IP address of the computer running the server application.
 * This will be your own laptop, until the beta beta demo or competition where
 * this is the TA or professor's laptop. In that case, set the IP to
 * 192.168.2.3.
 * 
 * 2. TEAM_NUMBER: your project team number
 * 
 * Note: We System.out.println() instead of LCD printing so that full debug
 * output (e.g. the very long string containing the transmission) can be read on
 * the screen OR a remote console such as the EV3Control program via Bluetooth
 * or WiFi. You can disable printing from the WiFi code via
 * ENABLE_DEBUG_WIFI_PRINT (below).
 * 
 * @author Michael Smith, Tharsan Ponnampalam, Xavier Pellemans
 *
 */
public class EV3WifiClient {
	/**
	 * Enumeration of the game parameters referring to a coordinate component
	 * Red_LL_x, Red_LL_y: lower left grid coordinates of the Red zone with respect
	 * to the 0,0
	 * 
	 * Red_UR_x, Red_UR_y: upper right grid coordinates of the Red zone with respect
	 * to the 0,0
	 * 
	 * Green_LL_x, Green_LL_y: lower left grid coordinates of the Green zone with
	 * respect to the 0,0
	 * 
	 * Green_UR_x, Green_UR_y: upper right grid coordinates of the Green zone with
	 * respect to the 0,0
	 * 
	 * SR_LL_x, SR_LL_y: lower left grid coordinates of the Red search zone with
	 * respect to the 0,0
	 * 
	 * SR_UR_x, SR_UR_y: upper right grid coordinates of the Red search zone with
	 * respect to the 0,0
	 * 
	 * SG_LL_x, SG_LL_y: lower left grid coordinates of the Green search zone with
	 * respect to the 0,0
	 * 
	 * SG_UR_x, SG_UR_y: upper right grid coordinates of the Green search zone with
	 * respect to the 0,0
	 * 
	 * BR_LL_x, BR_LL_y: lower left grid coordinates of the bridge with respect to
	 * the 0,0
	 * 
	 * BR_UR_x, BR_UR_y: upper right grid coordinates of the bridge zone with
	 * respect to the 0,0
	 * 
	 * TN_LL_x, TN_LL_y: lower left grid coordinates of the tunnel with respect to
	 * the 0,0
	 * 
	 * TN_UR_x, TN_UR_y: upper right grid coordinates of the tunnel with respect to
	 * the 0,0
	 */
	public enum CoordParameter {
		Red_LL_x, Red_LL_y, 
		Red_UR_x, Red_UR_y, 
		Green_LL_x, Green_LL_y, 
		Green_UR_x, Green_UR_y, 
		SR_LL_x, SR_LL_y, 
		SR_UR_x, SR_UR_y, 
		SG_LL_x, SG_LL_y, 
		SG_UR_x, SG_UR_y, 
		BR_LL_x, BR_LL_y, 
		BR_UR_x, BR_UR_y, 
		TN_LL_x, TN_LL_y, 
		TN_UR_x, TN_UR_y,
	}

	/**
	 * Enumeration for the parameters not related to the coordinate system OG: flag
	 * of the green team OR: flag of the red team RedCorner: Starting corner of the
	 * red team GreenCorner: Starting corner of the green team RedTeam: Team number
	 * of the red team GreenTeam: Team number of the green team
	 */
	public enum QualParameter {
		OG, OR, RedCorner, GreenCorner, RedTeam, GreenTeam
	}

	/**
	 * Enumeration for the team colors RED: red team GREEN: green team
	 */
	public enum TeamColor {
		RED, GREEN
	}

	/**
	 * Enumeration for the different zones on the map RED: red team zone GREEN:
	 * green team zone WATER: water area (no man's land) BRIDGE: zone of the bridge
	 * TUNNEL: zone of the tunnel SG: search zone in the green zone SR: search zone
	 * in the red zone
	 */
	public enum Zone {
		RED, GREEN, WATER, BRIDGE, TUNNEL, SG, SR
	}

	// ** Set these as appropriate for your team and current situation **
	private static final String SERVER_IP = "192.168.2.3"; // put your ipv4 here (go to cmd and write ipconfig)
	// "192.168.2.3"
	private static final int TEAM_NUMBER = 1; // Best team ever, will definitely win the competition
	public static final int X_GRID_LINES = 12; // according to predefined convention for x
	public static final int Y_GRID_LINES = 12; // according to predefined convention for y

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;
	@SuppressWarnings("rawtypes")
	private Map data;
	private TeamColor teamColor;

	/**
	 * Constructor of the wifi client Waits to receive the values from the EV3
	 * server and stores them in a map
	 * 
	 * @throws Exception
	 *             if it can't connect to the server (e.g. wrong IP address, server
	 *             not running on laptop, not connected to WiFi router, etc.). It
	 *             will also throw an exception if it connects but receives
	 *             corrupted data or a message from the server saying something went
	 *             wrong
	 */
	public EV3WifiClient() throws Exception {
		System.out.println("                            00");

		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might occur

		/*
		 * getData() will connect to the server and wait until the user/TA presses the
		 * "Start" button in the GUI on their laptop with the data filled in. Once it's
		 * waiting, you can kill it by pressing the upper left hand corner button
		 * (back/escape) on the EV3. getData() will throw exceptions if it can't connect
		 * to the server (e.g. wrong IP address, server not running on laptop, not
		 * connected to WiFi router, etc.). It will also throw an exception if it
		 * connects but receives corrupted data or a message from the server saying
		 * something went wrong. For example, if TEAM_NUMBER is set to 1 above but the
		 * server expects teams 17 and 5, this robot will receive a message saying an
		 * invalid team number was specified and getData() will throw an exception
		 * letting you know.
		 */

		data = conn.getData();

		
		/*
		 * This method is there to warn the user about error in the server data.
		 * The warning can be discarded by simply pressing a button on the EV3
		 */
		try {
			validateData();  
		}catch(Exception exc) {
			Sound.buzz();
			System.err.println(exc.getMessage());
			Button.waitForAnyPress();
			GamePlan.lcd.clear();
		}


	}

	/**
	 * Gets the color of this Robot's team according to the team number and the
	 * values received from the server
	 * 
	 * @return The color of the robot's team (type enum TeamColor)
	 * 
	 * @throws Exception
	 *             Throws an a general exception with a personalized message telling
	 *             the robot is not in any team
	 */
	public TeamColor getTeamColor() throws Exception {
		if (teamColor != null) { // if team color is already known
			return teamColor;
		} else {
			int RedTeam = ((Long) data.get("RedTeam")).intValue();
			int GreenTeam = ((Long) data.get("GreenTeam")).intValue();
			if (RedTeam == TEAM_NUMBER) {
				teamColor = TeamColor.RED;
				return teamColor; // Red team
			} else if (GreenTeam == TEAM_NUMBER) {
				teamColor = TeamColor.GREEN;
				return teamColor; // Green Team
			} else {
				throw new Exception("Team is not playing.\n Get off the field");
			}
		}
	}

	/**
	 * Gets the desired coordinate parameter representing the desired grid
	 * coordinate specified to this method.
	 * 
	 * This method only accepts name of variables contained in the
	 *         CoordParameter enum in this class Here is the description of each:
	 * 
	 *         Red_LL_x, Red_LL_y: lower left grid coordinates of the Red zone with
	 *         respect to the 0,0
	 * 
	 *         Red_UR_x, Red_UR_y: upper right grid coordinates of the Red zone with
	 *         respect to the 0,0
	 * 
	 *         Green_LL_x, Green_LL_y: lower left grid coordinates of the Green zone
	 *         with respect to the 0,0
	 * 
	 *         Green_UR_x, Green_UR_y: upper right grid coordinates of the Green
	 *         zone with respect to the 0,0
	 * 
	 *         SR_LL_x, SR_LL_y: lower left grid coordinates of the Red search zone
	 *         with respect to the 0,0 (is in the red zone)
	 * 
	 *         SR_UR_x, SR_UR_y: upper right grid coordinates of the Red search zone
	 *         with respect to the 0,0 (is in the red zone)
	 * 
	 *         SG_LL_x, SG_LL_y: lower left grid coordinates of the Green search
	 *         zone with respect to the 0,0 (is in the green zone)
	 * 
	 *         SG_UR_x, SG_UR_y: upper right grid coordinates of the Green search
	 *         zone with respect to the 0,0 (is in the green zone)
	 * 
	 *         BR_LL_x, BR_LL_y: lower left grid coordinates of the bridge with
	 *         respect to the 0,0
	 * 
	 *         BR_UR_x, BR_UR_y: upper right grid coordinates of the bridge zone
	 *         with respect to the 0,0
	 * 
	 *         TN_LL_x, TN_LL_y: lower left grid coordinates of the tunnel with
	 *         respect to the 0,0
	 * 
	 *         TN_UR_x, TN_UR_y: upper right grid coordinates of the tunnel with
	 *         respect to the 0,0
	 * 
	 * 
	 * @param the CoordParameter desired
	 *            
	 * @return the desired coordinate parameter (as an int)
	 * 
	 *         
	 */
	public int getCoordParam(CoordParameter param) {
		return ((Long) data.get(param.toString())).intValue();
	}

	/**
	 * Gets the starting corner of this robot based on it's team color
	 * 
	 * @return the starting corner between 0 and 3 0: lower left 1: lower right 2:
	 *         upper right 3: upper left
	 * @throws Exception
	 *             Throws this general exception with a personalized message when
	 *             the robot is not in one of the two teams, thus should not start
	 *             in a corner
	 */
	public int getStartingCorner() throws Exception {
		TeamColor teamColor = getTeamColor();
		if (teamColor == TeamColor.RED) {
			// Red team
			return ((Long) data.get(QualParameter.RedCorner.toString())).intValue();
		} else {
			// Green team
			return ((Long) data.get(QualParameter.GreenCorner.toString())).intValue();
		}
	}

	/**
	 * Gets the block color of the flag which the robot has to find It is usually
	 * the oponent's flag.
	 * 
	 * @return the desired BlockColor (enum in ColorSensor class) Possible values
	 *         are listed as follows: RED (1) BLUE (2) YELLOW (3) WHITE (4)
	 * 
	 * @throws Exception
	 *             Throws this general exception with a personalized message when
	 *             the robot is not in one of the two teams, thus should not start
	 *             in a corner, or if the color of the oponent's flag is not defined
	 *             correctly
	 */
	public ColorSensor.BlockColor getFlagColor() throws Exception {
		TeamColor teamColor = getTeamColor();
		if (teamColor == TeamColor.RED) {
			// Red team
			// looks for flag of green team
			switch (((Long) data.get(QualParameter.OG.toString())).intValue()) {
			case 1:
				// Red
				return ColorSensor.BlockColor.RED;
			case 2:
				// Blue
				return ColorSensor.BlockColor.BLUE;
			case 3:
				// Yellow
				return ColorSensor.BlockColor.YELLOW;
			case 4:
				// White
				return ColorSensor.BlockColor.WHITE;
			default:
				throw new Exception("Unspecified/Erroneous green flag color");
			}
		} else {
			// Green team
			// looks for flag of red team
			switch (((Long) data.get(QualParameter.OR.toString())).intValue()) {
			case 1:
				// Red
				return ColorSensor.BlockColor.RED;
			case 2:
				// Blue
				return ColorSensor.BlockColor.BLUE;
			case 3:
				// Yellow
				return ColorSensor.BlockColor.YELLOW;
			case 4:
				// White
				return ColorSensor.BlockColor.WHITE;
			default:
				throw new Exception("Unspecified/Erroneous red flag color");
			}
		}
	}

	/**
	 * Gets the zone which contains the specified line coordinates Default zone is
	 * WATER
	 * 
	 * @param x
	 *            Line number in x (0, 1, ..., X_GRID_LINES)
	 * @param y
	 *            Line number in y (0, 1, ..., Y_GRID_LINES)
	 * @return The Zone where the coordinates are
	 */
	public Zone getZone(int x, int y) {
		return getZone(x * Navigation.TILE_SIZE, y * Navigation.TILE_SIZE);
	}

	/**
	 * Gets the zone which contains the specified odometer coordinates. Default zone
	 * is WATER
	 * 
	 * @param x
	 *            Coordinate in x (as a double)
	 * @param y
	 *            Coordinate in y (as a double)
	 * @return The Zone where the coordinates are
	 */
	public Zone getZone(double x, double y) {
		for (Zone zone : Zone.values()) {
			if (isInZone(x, y, zone)) {// Checks every zone until it finds the right zone
				return zone;
			}
		}
		return Zone.WATER; // default value
	}

	/**
	 * Gets if the specified line coordinates are in the specified zone.
	 * 
	 * @param targetX
	 *            Line number in x (0, 1, ..., X_GRID_LINES)
	 * @param targetY
	 *            Line number in y (0, 1, ..., Y_GRID_LINES)
	 * @param zone
	 *            Zone which needs to be checked.
	 * @return True if the coordinates are in the zone.
	 */
	public boolean isInZone(int targetX, int targetY, Zone zone) {
		return isInZone(targetX * Navigation.TILE_SIZE, targetY * Navigation.TILE_SIZE, zone);
	}

	/**
	 * Gets if the specified odometer coordinates are in the specified zone.
	 * 
	 * @param targetX
	 *            Coordinate in x (as a double)
	 * @param targetY
	 *            Coordinate in y (as a double)
	 * @param zone
	 *            Zone which needs to be checked
	 * @return True if the coordinates are in the zone
	 */
	public boolean isInZone(double targetX, double targetY, Zone zone) {
		switch (zone) {
		case RED:
			if (targetX <= getCoordParam(CoordParameter.Red_UR_x) * Navigation.TILE_SIZE
					&& targetX >= getCoordParam(CoordParameter.Red_LL_x) * Navigation.TILE_SIZE
					&& targetY <= getCoordParam(CoordParameter.Red_UR_y) * Navigation.TILE_SIZE
					&& targetY >= getCoordParam(CoordParameter.Red_LL_y) * Navigation.TILE_SIZE) {
				if (!isInZone(targetX, targetY, Zone.SR)) { // if the robot is not in the search zone in red zone
					return true;
				}
			}
			break;
		case GREEN:
			if (targetX <= getCoordParam(CoordParameter.Green_UR_x) * Navigation.TILE_SIZE
					&& targetX >= getCoordParam(CoordParameter.Green_LL_x) * Navigation.TILE_SIZE
					&& targetY <= getCoordParam(CoordParameter.Green_UR_y) * Navigation.TILE_SIZE
					&& targetY >= getCoordParam(CoordParameter.Green_LL_y) * Navigation.TILE_SIZE) {
				if (!isInZone(targetX, targetY, Zone.SG)) { // if the robot is not in the search zone in green zone
					return true;
				}
			}
			break;
		case SG:
			if (targetX <= getCoordParam(CoordParameter.SG_UR_x) * Navigation.TILE_SIZE
					&& targetX >= getCoordParam(CoordParameter.SG_LL_x) * Navigation.TILE_SIZE
					&& targetY <= getCoordParam(CoordParameter.SG_UR_y) * Navigation.TILE_SIZE
					&& targetY >= getCoordParam(CoordParameter.SG_LL_y) * Navigation.TILE_SIZE) {
				return true;
			}
			break;
		case SR:
			if (targetX <= getCoordParam(CoordParameter.SR_UR_x) * Navigation.TILE_SIZE
					&& targetX >= getCoordParam(CoordParameter.SR_LL_x) * Navigation.TILE_SIZE
					&& targetY <= getCoordParam(CoordParameter.SR_UR_y) * Navigation.TILE_SIZE
					&& targetY >= getCoordParam(CoordParameter.SR_LL_y) * Navigation.TILE_SIZE) {
				return true;
			}
			break;
		case BRIDGE:
			if (targetX < getCoordParam(CoordParameter.BR_UR_x) * Navigation.TILE_SIZE
					&& targetX > getCoordParam(CoordParameter.BR_LL_x) * Navigation.TILE_SIZE
					&& targetY < getCoordParam(CoordParameter.BR_UR_y) * Navigation.TILE_SIZE
					&& targetY > getCoordParam(CoordParameter.BR_LL_y) * Navigation.TILE_SIZE) {
				return true;
			}
			break;
		case TUNNEL:
			if (targetX < getCoordParam(CoordParameter.TN_UR_x) * Navigation.TILE_SIZE
					&& targetX > getCoordParam(CoordParameter.TN_LL_x) * Navigation.TILE_SIZE
					&& targetY < getCoordParam(CoordParameter.TN_UR_y) * Navigation.TILE_SIZE
					&& targetY > getCoordParam(CoordParameter.TN_LL_y) * Navigation.TILE_SIZE) {
				return true;
			}
			break;
		case WATER:
			// if is not in any of the zones above
			if (!isInZone(targetX, targetY, Zone.GREEN) && !isInZone(targetX, targetY, Zone.RED)
					&& !isInZone(targetX, targetY, Zone.TUNNEL) && !isInZone(targetX, targetY, Zone.BRIDGE)) {
				return true;
			}
			break;

		}
		return false;
	}

	/**
	 * Gets on what side the specified coordinates are compared to the zone.
	 * East and West sides are larger than the North and South sides.
	 * 
	 * @param zone
	 *            Zone specified
	 * @param x
	 *            X absolute coordinate (in cm)
	 * @param y
	 *            Y absolute coordinate (in cm)
	 * @return The side closest to the point (EAST and WEST are predominant)
	 */
	public GamePlan.Direction getSide(Zone zone, double x, double y) {
		/*
		 * 			|	North	  |
		 * 			 _____________
		 * 			|  	Zone	  |
		 * 			|  			  |
		 * 	West	|  	Center	  |		East
		 * 			|  			  |
		 *          |_____________|
		 * 
		 * 			|	South	  |
		 */
		
		double lowerLeftX = 0, lowerLeftY = 0, upperRightX = 0, upperRightY = 0;
		switch (zone) {
		case WATER:
			//not relevant
			break;
		case BRIDGE:
			lowerLeftX = getCoordParam(CoordParameter.BR_LL_x)*Navigation.TILE_SIZE;
			lowerLeftY = getCoordParam(CoordParameter.BR_LL_y)*Navigation.TILE_SIZE;
			upperRightX = getCoordParam(CoordParameter.BR_UR_x)*Navigation.TILE_SIZE;
			upperRightY = getCoordParam(CoordParameter.BR_UR_y)*Navigation.TILE_SIZE;
			break;
		case TUNNEL:
			lowerLeftX = getCoordParam(CoordParameter.TN_LL_x)*Navigation.TILE_SIZE;
			lowerLeftY = getCoordParam(CoordParameter.TN_LL_y)*Navigation.TILE_SIZE;
			upperRightX = getCoordParam(CoordParameter.TN_UR_x)*Navigation.TILE_SIZE;
			upperRightY = getCoordParam(CoordParameter.TN_UR_y)*Navigation.TILE_SIZE;
			break;
		case GREEN:
			lowerLeftX = getCoordParam(CoordParameter.Green_LL_x)*Navigation.TILE_SIZE;
			lowerLeftY = getCoordParam(CoordParameter.Green_LL_y)*Navigation.TILE_SIZE;
			upperRightX = getCoordParam(CoordParameter.Green_UR_x)*Navigation.TILE_SIZE;
			upperRightY = getCoordParam(CoordParameter.Green_UR_y)*Navigation.TILE_SIZE;
			break;
		case RED:
			lowerLeftX = getCoordParam(CoordParameter.Red_LL_x)*Navigation.TILE_SIZE;
			lowerLeftY = getCoordParam(CoordParameter.Red_LL_y)*Navigation.TILE_SIZE;
			upperRightX = getCoordParam(CoordParameter.Red_UR_x)*Navigation.TILE_SIZE;
			upperRightY = getCoordParam(CoordParameter.Red_UR_y)*Navigation.TILE_SIZE;
			break;
		case SG:
			lowerLeftX = getCoordParam(CoordParameter.SG_LL_x)*Navigation.TILE_SIZE;
			lowerLeftY = getCoordParam(CoordParameter.SG_LL_y)*Navigation.TILE_SIZE;
			upperRightX = getCoordParam(CoordParameter.SG_UR_x)*Navigation.TILE_SIZE;
			upperRightY = getCoordParam(CoordParameter.SG_UR_y)*Navigation.TILE_SIZE;
			break;
		case SR:
			lowerLeftX = getCoordParam(CoordParameter.SR_LL_x)*Navigation.TILE_SIZE;
			lowerLeftY = getCoordParam(CoordParameter.SR_LL_y)*Navigation.TILE_SIZE;
			upperRightX = getCoordParam(CoordParameter.SR_UR_x)*Navigation.TILE_SIZE;
			upperRightY = getCoordParam(CoordParameter.SR_UR_y)*Navigation.TILE_SIZE;
			break;
		}
		if (x <= lowerLeftX) {
			return GamePlan.Direction.WEST;
		} else if (x < upperRightX) {
			if (y <= lowerLeftY) {
				return GamePlan.Direction.SOUTH;
			} else if (y >= upperRightY) {
				return GamePlan.Direction.NORTH;
			} else {
				// center of search zone
				return GamePlan.Direction.CENTER;
			}
		} else {
			return GamePlan.Direction.EAST;
		}
	}

	/**
	 * Gets the tile width of the tunnel in line with the entry side (in grid line).
	 * 
	 * @param entry
	 *            Side of the entrance to the tunnel
	 * @return The width in this axis (in grid lines)
	 * @throws Exception
	 *             If the entry point specified is erroneous
	 */
	public int getTunnelWidth(GamePlan.Direction entry) throws Exception {
		switch (entry) {
		default:
		case CENTER:
			throw new Exception("getTunnelWidth(): no such entry point");
		case NORTH:
		case SOUTH:
			return getCoordParam(CoordParameter.TN_UR_y) - getCoordParam(CoordParameter.TN_LL_y);
		case EAST:
		case WEST:
			return getCoordParam(CoordParameter.TN_UR_x) - getCoordParam(CoordParameter.TN_LL_x);
		}
	}

	/**
	 * Gets the tile width of the tunnel in line with the entry side (in grid line).
	 * 
	 * @param entry
	 *            Side of the entrance to the tunnel
	 * @return The width in this axis (in grid lines)
	 * @throws Exception
	 *             If the entry point specified is erroneous
	 */
	public int getBridgeWidth(GamePlan.Direction entry) throws Exception {
		switch (entry) {
		default:
		case CENTER:
			throw new Exception("getBridgeWidth(): no such entry point");
		case NORTH:
		case SOUTH:
			return getCoordParam(CoordParameter.BR_UR_y) - getCoordParam(CoordParameter.BR_LL_y);
		case EAST:
		case WEST:
			return getCoordParam(CoordParameter.BR_UR_x) - getCoordParam(CoordParameter.BR_LL_x);
		}
	}

	/**
	 * Method for the validation of the server user input Checks every parameter and
	 * makes sure it is plausible
	 * 
	 * @throws Exception
	 *             throws a generalized Exception containing specific message on
	 *             what parameter is not correct
	 */
	private void validateData() throws Exception {
		int param; // place holder for the data checked
		// Coordinate parameters
		for (int i = 0; i < CoordParameter.values().length; i++) {
			switch (CoordParameter.values()[i]) {
			case BR_LL_x:
			case BR_UR_x:
			case TN_LL_x:
			case TN_UR_x:
			case Green_LL_x:
			case Green_UR_x:
			case Red_LL_x:
			case Red_UR_x:
			case SG_LL_x:
			case SG_UR_x:
			case SR_LL_x:
			case SR_UR_x:
				param = ((Long) data.get(CoordParameter.values()[i].toString())).intValue();
				if (param < 0 || param > X_GRID_LINES) {
					throw new Exception("Parameter " + CoordParameter.values()[i].toString() + " out of bounds");
				}
				break;

			case BR_LL_y:
			case BR_UR_y:
			case TN_LL_y:
			case TN_UR_y:
			case Green_LL_y:
			case Green_UR_y:
			case Red_LL_y:
			case Red_UR_y:
			case SG_LL_y:
			case SG_UR_y:
			case SR_LL_y:
			case SR_UR_y:
				param = ((Long) data.get(CoordParameter.values()[i].toString())).intValue();
				if (param < 0 || param > Y_GRID_LINES) {
					throw new Exception("Parameter " + CoordParameter.values()[i].toString() + " out of bounds");
				}
				break;
			}
		}

		// Qualitative parameters
		for (int i = 0; i < QualParameter.values().length; i++) {
			switch (QualParameter.values()[i]) {
			case GreenTeam:
			case RedTeam:
				param = ((Long) data.get(QualParameter.values()[i].toString())).intValue();
				if (param < 1 || param > 20) {
					throw new Exception("Parameter " + QualParameter.values()[i].toString() + " out of bounds");
				}
				break;
			case GreenCorner:
			case RedCorner:
				param = ((Long) data.get(QualParameter.values()[i].toString())).intValue();

				if (param < 0 || param > 3) {
					throw new Exception("Parameter " + QualParameter.values()[i].toString() + " out of bounds");
				}
				break;
			case OG:
			case OR:
				param = ((Long) data.get(QualParameter.values()[i].toString())).intValue();
				if (param < 1 || param > 4) {
					throw new Exception("Parameter " + QualParameter.values()[i].toString() + " out of bounds");
				}
				break;
			}
		}
		// Bridge in x
		param = ((Long) data.get(CoordParameter.BR_UR_x.toString())).intValue()
				- ((Long) data.get(CoordParameter.BR_LL_x.toString())).intValue();
		if (param < 1 || param > 2) {
			throw new Exception("Bridge length in x out of bounds");
		}

		// Bridge in y
		param = ((Long) data.get(CoordParameter.BR_UR_y.toString())).intValue()
				- ((Long) data.get(CoordParameter.BR_LL_y.toString())).intValue();
		if (param < 1 || param > 2) {
			throw new Exception("Bridge length in y out of bounds");
		}

		// Tunnel in x
		param = ((Long) data.get(CoordParameter.TN_UR_x.toString())).intValue()
				- ((Long) data.get(CoordParameter.TN_LL_x.toString())).intValue();
		if (param < 1 || param > 2) {
			throw new Exception("Tunnel length in x out of bounds");
		}

		// Tunnel in y
		param = ((Long) data.get(CoordParameter.TN_UR_y.toString())).intValue()
				- ((Long) data.get(CoordParameter.TN_LL_y.toString())).intValue();
		if (param < 1 || param > 2) {
			throw new Exception("Tunnel length in y out of bounds");
		}

		// Green zone in x
		param = ((Long) data.get(CoordParameter.Green_UR_x.toString())).intValue()
				- ((Long) data.get(CoordParameter.Green_LL_x.toString())).intValue();
		if (param < 2 || param > 10) {
			throw new Exception("Green zone length in x out of bounds");
		}

		// Green zone in y
		param = ((Long) data.get(CoordParameter.Green_UR_y.toString())).intValue()
				- ((Long) data.get(CoordParameter.Green_LL_y.toString())).intValue();
		if (param < 2 || param > 10) {
			throw new Exception("Green zone length in y out of bounds");
		}
		// Red zone in x
		param = ((Long) data.get(CoordParameter.Red_UR_x.toString())).intValue()
				- ((Long) data.get(CoordParameter.Red_LL_x.toString())).intValue();
		if (param < 2 || param > 10) {
			throw new Exception("Red zone length in x out of bounds");
		}

		// Red zone in y
		param = ((Long) data.get(CoordParameter.Red_UR_y.toString())).intValue()
				- ((Long) data.get(CoordParameter.Red_LL_y.toString())).intValue();
		if (param < 2 || param > 10) {
			throw new Exception("Red zone length in y out of bounds");
		}

		// Green search zone in x
		param = ((Long) data.get(CoordParameter.SG_UR_x.toString())).intValue()
				- ((Long) data.get(CoordParameter.SG_LL_x.toString())).intValue();
		if (param < 2 || param > 10) {
			System.out.println(param);
			throw new Exception("Green search zone length in x out of bounds");

		}

		// Green search zone in y
		param = ((Long) data.get(CoordParameter.SG_UR_y.toString())).intValue()
				- ((Long) data.get(CoordParameter.SG_LL_y.toString())).intValue();
		System.out.println("abc" + param);
		if (param < 2 || param > 10) {

			throw new Exception("Green search zone length in y out of bounds");
		}
		// Red search zone in x
		param = ((Long) data.get(CoordParameter.SR_UR_x.toString())).intValue()
				- ((Long) data.get(CoordParameter.SR_LL_x.toString())).intValue();
		if (param < 2 || param > 10) {
			throw new Exception("Red search zone length in x out of bounds");
		}

		// Red search zone in y
		param = ((Long) data.get(CoordParameter.SR_UR_y.toString())).intValue()
				- ((Long) data.get(CoordParameter.SR_LL_y.toString())).intValue();
		if (param < 2 || param > 10) {
			throw new Exception("Red search zone length in y out of bounds");
		}
	}

}
