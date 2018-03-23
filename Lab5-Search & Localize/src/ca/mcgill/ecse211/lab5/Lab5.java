package ca.mcgill.ecse211.lab5;

/**
 * imports
 */
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab5.EV3WifiClient.CoordParameter;
import ca.mcgill.ecse211.localizer.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * Main class of the Final Project
 * 
 * @author Xavier Pellemans 260775554
 * @author Thomas Bahen 260675971
 *
 */
public class Lab5 {
	

	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
	
	/**
	 * Main method of the program. Creates the initial menu displayed on EV3, 
	 * Manages the choices made by the user, 
	 * Creates instances of the main procedure classes,
	 * Give options to tune the robot's systems 
	 * And starts the game when the user is ready.
	 * Catches any error thrown by the program.
	 * 
	 * @param args Compiler arguments
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		int buttonChoice;

	
			
			
			GamePlan game=new GamePlan();
			Button.waitForAnyPress();
			game.play();
			Button.waitForAnyPress();
			System.exit(0);
			
//			
//			try {
//				System.out.println("please press any button");
//				Button.waitForAnyPress();
//			EV3WifiClient serverData=new EV3WifiClient();
//
//			System.out.println("severdata complete");
//			Button.waitForAnyPress();
//			
//			System.out.println("Red_LL_x" + serverData.getCoordParam(CoordParameter.Red_LL_x));
//			System.out.println("Red_LL_y" + serverData.getCoordParam(CoordParameter.Red_LL_y));
//			System.out.println("BR_LL_x" + serverData.getCoordParam(CoordParameter.BR_LL_x));
//			System.out.println("TN_LL_x" + serverData.getCoordParam(CoordParameter.TN_LL_x));
//			
//			Button.waitForAnyPress();
//			
//			}catch(Exception e) {
//				 System.err.println("Error: main " + e.getMessage());
//				 Button.waitForAnyPress();
//			}
//			
//			
/*			
		
			
			
			do {
				// clear the display
				lcd.clear();

				// ask the user whether the motors should drive in a square or float
				lcd.drawString("< Left | Right >", 0, 0);
				lcd.drawString("       |        ", 0, 1);
				lcd.drawString(" Screw | Start  ", 0, 2);
				lcd.drawString(" Fix   | Game   ", 0, 3);
				lcd.drawString("       |        ", 0, 4);

				buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
				
				if (buttonChoice == Button.ID_LEFT) {
					
					game.trackAdjust();
					
				}
			} while (buttonChoice != Button.ID_RIGHT);
			
			
			game.play();
			
			
				
			
		} catch (OdometerExceptions exc) {
			// instance error, do nothing
		}*/
	}
}
