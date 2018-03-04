package ca.mcgill.ecse211.lab5;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.localizer.ColorSensor;

/**
 * Example class using WifiConnection to communicate with a server and receive
 * data concerning the competition such as the starting corner the robot is
 * placed in.
 * 
 * Keep in mind that this class is an **example** of how to use the WiFi code;
 * you must use the WifiConnection class yourself in your own code as
 * appropriate. In this example, we simply show how to get and process different
 * types of data.
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
	public enum CoordParameter {
		Red_LL_x, Red_LL_y, Red_UR_x, Red_UR_y, Green_LL_x, Green_LL_y, Green_UR_x, Green_UR_y,
		SR_LL_x, SR_LL_y, SR_UR_x, SR_UR_y, SG_LL_x, SG_LL_y, SG_UR_x, SG_UR_y,
		BR_LL_x, BR_LL_y, BR_UR_x, BR_UR_y, TN_LL_x, TN_LL_y, TN_UR_x, TN_UR_y,
	}

	public enum QualParameter {
		OG, OR, RedCorner, GreenCorner, RedTeam, GreenTeam
	}

	public enum TeamColor {
		RED, GREEN
	}

	// ** Set these as appropriate for your team and current situation **
	private static final String SERVER_IP = "192.168.1.104"; // put your ipv4 here (go to cmd and write ipconfig)
	// "192.168.2.3"
	private static final int TEAM_NUMBER = 1; // Best team ever, will definitely win the competition

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;
	@SuppressWarnings("rawtypes")
	private Map data;
	private TeamColor teamColor;

	
	/**
	 * Constructor of the wifi client
	 * Waits to receive the values from the EV3 server 
	 * and stores them in a map
	 * 
	 * @throws Exception if it can't connect
	 * to the server (e.g. wrong IP address, server not running on laptop, not
	 * connected to WiFi router, etc.). It will also throw an exception if it
	 * connects but receives corrupted data or a message from the server saying
	 * something went wrong
	 */
	public EV3WifiClient() throws Exception{
		System.out.println("Running..");

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
	 * coordinate specified to this method
	 * 
	 * @param the CoordParameter desired
	 * @return the desired coordinate parameter (as an int)
	 * 
	 * This method only accepts name of variables contained in the CoordParameter
	 * enum in this class Here is the description of each: 
	 * 
	 * Red_LL_x, Red_LL_y: lower left grid coordinates of the Red zone with respect to the 0,0
	 * 
	 * Red_UR_x, Red_UR_y: upper right grid coordinates of the Red zone with respect to the 0,0
	 * 
	 * Green_LL_x, Green_LL_y: lower left grid coordinates of the Green zone with respect to the 0,0
	 * 
	 * Green_UR_x, Green_UR_y: upper right grid coordinates of the Green zone with respect to the 0,0
	 * 
	 * SR_LL_x,  SR_LL_y: lower left grid coordinates of the Red search zone with respect to the 0,0
	 * 
	 * SR_UR_x, SR_UR_y: upper right grid coordinates of the Red search zone with respect to the 0,0
	 * 
	 * SG_LL_x, SG_LL_y: lower left grid coordinates of the Green search zone with respect to the 0,0
	 * 
	 * SG_UR_x, SG_UR_y: upper right grid coordinates of the Green search zone with respect to the 0,0
	 * 
	 * BR_LL_x, BR_LL_y: lower left grid coordinates of the bridge with respect to the 0,0
	 * 
	 * BR_UR_x, BR_UR_y: upper right grid coordinates of the bridge zone with respect to the 0,0
	 * 
	 * TN_LL_x, TN_LL_y: lower left grid coordinates of the tunnel with respect to the 0,0
	 * 
	 * TN_UR_x, TN_UR_y: upper right grid coordinates of the tunnel with respect to the 0,0
	 * 
	 */
	public int getCoordParam(CoordParameter param) {
		return ((Long) data.get(param.toString())).intValue();
	}
	
	
	/**
	 * Gets the starting corner of this robot based on it's team color
	 * 
	 * @return the starting corner between 0 and 3
	 * 0: lower left
	 * 1: lower right
	 * 2: upper right
	 * 3: upper left
	 * @throws Exception Throws this general exception with a personalized message
	 * 					when the robot is not in one of the two teams, thus should not
	 * 					start in a corner
	 */
	public int getStartingCorner() throws Exception {
		TeamColor teamColor=getTeamColor();
		if(teamColor==teamColor.RED) {
			//Red team
			return ((Long) data.get(QualParameter.RedCorner)).intValue();
		}else {
			//Green team
			return ((Long) data.get(QualParameter.GreenCorner)).intValue();
		}
	}
	
	
	/**
	 * Gets the block color of the flag which the robot has to find
	 * It is usually the oponent's flag
	 * 
	 * @return the desired BlockColor (enum in ColorSensor class)
	 * Possible values are listed as follows:
	 * RED		(1)
	 * BLUE		(2)
	 * YELLOW	(3)
	 * WHITE	(4)
	 * 
	 * @throws Exception Throws this general exception with a personalized message
	 * 					when the robot is not in one of the two teams, thus should not
	 * 					start in a corner, or if the color of the oponent's flag is not
	 * 					defined correctly
	 */
	public ColorSensor.BlockColor getFlagColor() throws Exception{
		TeamColor teamColor=getTeamColor();
		if(teamColor==teamColor.RED) {
			//Red team
			//looks for flag of green team
			switch(((Long) data.get(QualParameter.OG)).intValue()) {
				case 1:
					//Red
					return ColorSensor.BlockColor.RED;
				case 2:
					//Blue
					return ColorSensor.BlockColor.BLUE;
				case 3:
					//Yellow
					return ColorSensor.BlockColor.YELLOW;
				case 4:
					//White
					return ColorSensor.BlockColor.WHITE;
				default:
					throw new Exception("Unspecified/Erroneous green flag color");
			}
		}else {
			//Green team
			//looks for flag of red team
			switch(((Long) data.get(QualParameter.OR)).intValue()) {
				case 1:
					//Red
					return ColorSensor.BlockColor.RED;
				case 2:
					//Blue
					return ColorSensor.BlockColor.BLUE;
				case 3:
					//Yellow
					return ColorSensor.BlockColor.YELLOW;
				case 4:
					//White
					return ColorSensor.BlockColor.WHITE;
				default:
					throw new Exception("Unspecified/Erroneous red flag color");
			}
		}
	}
}
