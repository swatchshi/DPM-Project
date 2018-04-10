package ca.mcgill.ecse211.lab5;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.localizer.UltrasonicSensor;
import ca.mcgill.ecse211.odometer.Gyroscope;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to display the content of the odometer variables (x, y,
 * Theta), ultrasonic distance and gyroscope angle.
 */
public class Display {

	private Odometer odo;
	private Gyroscope gyroscope;
	private TextLCD lcd;
	private double[] position;
	private static Display display;

	/**
	 * Constructor of the Display
	 * 
	 * @param lcd
	 *            The screen used to display
	 * @param gyroscope
	 *            Gyroscope used
	 * @throws OdometerExceptions
	 *             if there is a problem with Odometer instances
	 */
	public Display(TextLCD lcd, Gyroscope gyroscope) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.lcd = lcd;
		this.gyroscope = gyroscope;
		display = this;
	}

	/**
	 * Gets the instance of the display used (only one at a time)
	 * 
	 * @param lcd
	 *            the lcd used
	 * @param gyroscope
	 *            the gyroscope used
	 * @return the display instance
	 * @throws OdometerExceptions
	 *             if error when creating instances of Odometer
	 */
	public synchronized static Display getDisplay(TextLCD lcd, Gyroscope gyroscope) throws OdometerExceptions {

		if (display != null) { // Return existing object
			return display;
		} else { // create object and return it
			display = new Display(lcd, gyroscope);
			return display;
		}
	}

	/**
	 * Threading method for the display of odometer variables
	 */
	public void printData() {

		lcd.clear();

		// Retrieve x, y and Theta information
		position = odo.getXYT();
		try {

			// Print x,y, and theta information

			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
			lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
			lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
			lcd.drawString("G: " + gyroscope.getAngle(), 0, 3);
			lcd.drawString("O: " + gyroscope.getOffset(), 0, 4);

		} catch (Exception e) {
		}

	}

}
