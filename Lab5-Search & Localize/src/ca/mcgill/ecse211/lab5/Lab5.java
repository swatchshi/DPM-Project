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
 * Main class of the Localizer Labs
 * 
 * @author Xavier Pellemans 260775554
 * @author Thomas Bahen 260675971
 *
 */
public class Lab5 {
	public enum RobotConfig {
		TRACTION, PROPULSION
	}

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	public static final EV3MediumRegulatedMotor usSensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	public static final EV3MediumRegulatedMotor trackExpansionMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	public static final EV3ColorSensor armSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	public static final SensorModes ultraSSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));

	public static final RobotConfig CONFIG = RobotConfig.PROPULSION;
	

	

	public static final int SR_LLx = 1;
	public static final int SR_LLy = 2;
	public static final int SR_URx = 3;
	public static final int SR_URy = 3;
	public static final int SG_LLx = 4;
	public static final int SG_LLy = 6;
	public static final int SG_URx = 6;
	public static final int SG_URy = 7;
	public static final int RZ_LLx = 0;
	public static final int RZ_LLy = 0;
	public static final int RZ_URx = 4;
	public static final int RZ_URy = 4;
	public static final int GZ_LLx = 2;
	public static final int GZ_LLy = 5;
	public static final int GZ_URx = 8;
	public static final int GZ_URy = 8;
	public static final int BR_LLx = 3;
	public static final int BR_LLy = 4;
	public static final int BR_URx = 4;
	public static final int BR_URy = 5;
	public static final int TN_LLx = 2;
	public static final int TN_LLy = 4;
	public static final int TN_URx = 3;
	public static final int TN_URy = 5;
	public static final ColorSensor.BlockColor blockWanted = ColorSensor.BlockColor.RED;

	public static void main(String[] args) {
		int buttonChoice;
		try {
			// US related objects
			@SuppressWarnings("resource") // Because we don't bother to close this resource
			SampleProvider us = ultraSSensor.getMode("Distance"); // usDistance provides samples from
																	// this instance
			float[] usData = new float[us.sampleSize()]; // usData is the buffer in which data are
			ColorSensor cSensor = new ColorSensor(armSensor);
			ColorSensor lSensor = new ColorSensor(lightSensor);
			UltrasonicSensor ultraSensor = new UltrasonicSensor(us, usData);

			// Odometer related objects
			TrackExpansion dynamicTrack=new TrackExpansion();
			Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, dynamicTrack , CONFIG);
			Display odometryDisplay = new Display(lcd, ultraSensor);

			do {
				// clear the display
				lcd.clear();

				// ask the user whether the motors should drive in a square or float
				lcd.drawString("< Left | Right >", 0, 0);
				lcd.drawString("       |        ", 0, 1);
				lcd.drawString(" Red   | Green  ", 0, 2);
				lcd.drawString(" Team  | Team   ", 0, 3);
				lcd.drawString("       |        ", 0, 4);

				buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
			} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

			if (buttonChoice == Button.ID_LEFT) {
				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();

				Navigation navigation = new Navigation(odometer, dynamicTrack, CONFIG);

				// shows color seen by the color sensor
				lcd.clear();
				FlagFinding flagFinder = new FlagFinding(dynamicTrack, cSensor, ultraSensor, blockWanted, SR_LLx, SR_LLy, SR_URx,
						SR_URy);

				Delay.msDelay(1000);

				while (true) {

					System.out.println(cSensor.getColorSeen());
					Delay.msDelay(1000);

				}

			} else {
				// clear the display
				lcd.clear();

				// ask the user which start corner is used
				lcd.drawString("  ^^ CORNER 0  ^^   ", 0, 0);
				lcd.drawString(" < _______________ >", 0, 1);
				lcd.drawString("< CORNER  | CORNER >", 0, 2);
				lcd.drawString(" <____1___|____2___>", 0, 3);
				lcd.drawString("  vv CORNER 3  vv   ", 0, 4);

				buttonChoice = Button.waitForAnyPress(); // Record choice (left,right press)

				// localization choice
				int corner = 0;

				switch (buttonChoice) {

				case Button.ID_UP:
					corner = 0;
					break;
				case Button.ID_LEFT:
					corner = 1;
					break;
				case Button.ID_RIGHT:
					corner = 2;
					break;
				case Button.ID_DOWN:
					corner = 3;
					break;
				default:
					System.exit(0);
					break;
				}

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
