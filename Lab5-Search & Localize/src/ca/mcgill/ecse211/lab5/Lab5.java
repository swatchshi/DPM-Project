package ca.mcgill.ecse211.lab5;

/**
 * imports
 */
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab5.EV3WifiClient.CoordParameter;
import ca.mcgill.ecse211.lab5.GamePlan.RobotConfig;
import ca.mcgill.ecse211.localizer.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;


/**
 * Main class of the Final Project
 * 
 * @author Xavier Pellemans 260775554
 * 		   Thomas Bahen 260675971
 * 		   Guangyi Zhang 260708622
 * 		   Cara Zhang
 * 		   WenQi Shi
 *
 */
public class Lab5 {



	/**
	 * Main method of the program. Creates the initial menu displayed on EV3,
	 * Manages the choices made by the user, Creates instances of the main procedure
	 * classes, Give options to tune the robot's systems And starts the game when
	 * the user is ready. Catches any error thrown by the program.
	 * 
	 * @param args
	 *            Compiler arguments
	 * @throws Exception
	 */
	public static void main(String[] args) throws Exception {
		try {


			GamePlan game = new GamePlan();
			game.play();
;			
		} catch (OdometerExceptions exc) {
			// instance error, do nothing
		}

	}

}
