package ca.mcgill.ecse211.lab5;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.localizer.UltrasonicSensor;
import ca.mcgill.ecse211.odometer.Gyroscope;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta), ultrasonic distance
 * and gyroscope angle.
 */
public class Display implements Runnable {

  private Odometer odo;
  private UltrasonicSensor us;
  private Gyroscope gyroscope;
  private TextLCD lcd;
  private double[] position;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * Constructor of the Display
   * 
   * @param lcd The screen used to display
   * @param us The UltrasonicSensor which will fetch the data
   * @param gyroscope Gyroscope used
   * @throws OdometerExceptions if there is a problem with Odometer instances
   */
  public Display(TextLCD lcd, UltrasonicSensor us, Gyroscope gyroscope) throws OdometerExceptions {
    odo = Odometer.getOdometer();
    this.lcd = lcd;
    this.us=us;
    this.gyroscope=gyroscope;
  }

  /**
   * This is the overloaded class constructor
   * 
   * @param lcd The screen used to display
   * @param us The UltrasonicSensor which will fetch the data
   * @param timeout The maximal time for display
   * @throws OdometerExceptions
   */
  
  public Display(TextLCD lcd, UltrasonicSensor us, long timeout) throws OdometerExceptions {
    odo = Odometer.getOdometer();
    this.timeout = timeout;
    this.lcd = lcd;
    this.us=us;
  }

  /**
   * Threading method for the display of odometer variables
   */
  public void run() {
    
    lcd.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = odo.getXYT();
      try {
      
      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      lcd.drawString("U: " + us.getDistance(), 0, 3);
      lcd.drawString("G: " + gyroscope.getAngle(), 0, 4);
      
      }catch(Exception e) {}
      
      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }

}
