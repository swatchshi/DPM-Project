package ca.mcgill.ecse211.lab5;

/**
 * imports
 */
import ca.mcgill.ecse211.odometer.*;
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
	
	public static void main(String[] args) {
		int buttonChoice;
		try {
			
			
			GamePlan game=new GamePlan(lcd);

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
			} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

			if (buttonChoice == Button.ID_LEFT) {
				
				
				
				game.trackAdjust();
				
				
				
				/*
				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();

				

				// shows color seen by the color sensor
				lcd.clear();
				FlagFinding flagFinder = new FlagFinding(dynamicTrack, cSensor, ultraSensor, blockWanted, SR_LLx, SR_LLy, SR_URx,
						SR_URy);

				Delay.msDelay(1000);

				while (true) {

					System.out.println(cSensor.getColorSeen());
					Delay.msDelay(1000);

				}
				*/

			} else {
				/*

				// Start odometer and display threads
				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();
				Navigation navigation = new Navigation(odometer, dynamicTrack, CONFIG);
				FlagFinding flagFinder = new FlagFinding(dynamicTrack, cSensor, ultraSensor, blockWanted, SR_LLx, SR_LLy, SR_URx, SR_URy);

				// Localization
				
				USLocalizer usLoc = new USLocalizer(odometer, navigation, ultraSensor);
				usLoc.doLocalization();

				LightLocalizer lightLoc = new LightLocalizer(navigation, lSensor, odometer, CONFIG);
				//////////////////////////////////////////////////////////////////////////////////////////////////
				lightLoc.doLocalization(1, 1, 0);

				// Block finding
				flagFinder.findBlock();
				flagFinder.beepSequence(1);
				System.exit(0);
				*/
			}

		} catch (OdometerExceptions exc) {
			// instance error, do nothing
		} catch (Exception e) {
			// Other error
			Sound.buzz();
			System.err.println("Error: " + e.getMessage());
		}
	}
}
