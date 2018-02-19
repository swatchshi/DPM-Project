package ca.mcgill.ecse211.lab5;


/**
 * imports
 */
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab5.*;
import ca.mcgill.ecse211.localizer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


/**
 * Main class of the Localizer Lab
 * @author Xavier Pellemans 260775554
 * @author Thomas Bahen 260675971
 *
 */
public class Lab5 {
	public enum RobotConfig{
		TRACTION,
		PROPULSION
	}
	
	// Motor Objects, and Robot related parameters
	  public static final EV3LargeRegulatedMotor leftMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	  public static final EV3LargeRegulatedMotor rightMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	 // public static final EV3MediumRegulatedMotor sensorMotor =
	//	      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	  public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	  public static final EV3ColorSensor lightSensor=new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	  public static final SensorModes ultraSSensor=new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
	  
	  public static final RobotConfig CONFIG=RobotConfig.PROPULSION;
	  public static final int X_GRID_LINES=8;
	  public static final int Y_GRID_LINES=8;
	  public static final int LOWER_LEFT=1;
	  public static final int UPPER_RIGHT=3;
	  //free space between wheels: 13.7 cm
	  //wheel width 2.2 (EACH)
	  //wheel diameter: 4.4
	  public static final double WHEEL_RAD = 2.2;
	  public static final double TRACK = 16.0;  //adjust from 13.7 to 18.1
	  
	public static void main(String[] args) {
		int buttonChoice;
	    try {
		    //US related objects
		    @SuppressWarnings("resource") // Because we don't bother to close this resource
		    SampleProvider us = ultraSSensor.getMode("Distance"); // usDistance provides samples from
		                                                              // this instance
		    float[] usData = new float[us.sampleSize()]; // usData is the buffer in which data are
		    ColorSensor cSensor=new ColorSensor(lightSensor);
		    UltrasonicSensor ultraSensor=new UltrasonicSensor(ultraSSensor, usData);
	
		    // Odometer related objects
		    rightMotor.synchronizeWith(new EV3LargeRegulatedMotor[] { leftMotor });  //to sync the wheels
		    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD, CONFIG); 
		    OdometerCorrection odoCorrection=new OdometerCorrection(cSensor, odometer);
		    Display odometryDisplay = new Display(lcd); // No need to change
	
	
		    do {
		      // clear the display
		      lcd.clear();
	
		      // ask the user whether the motors should drive in a square or float
		      lcd.drawString("< Left | Right >", 0, 0);
		      lcd.drawString("       |        ", 0, 1);
		      lcd.drawString(" Float | Local- ", 0, 2);
		      lcd.drawString("motors | ization", 0, 3);
		      lcd.drawString("       |        ", 0, 4);
	
		      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
	
		    if (buttonChoice == Button.ID_LEFT) {
		      // Float the motors
		      leftMotor.forward();
		      leftMotor.flt();
		      rightMotor.forward();
		      rightMotor.flt();
	
		      // Display changes in position as wheels are (manually) moved
		      
		      Thread odoThread = new Thread(odometer);
		      odoThread.start();
		      Thread odoDisplayThread = new Thread(odometryDisplay);
		      odoDisplayThread.start();
		     
		     
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
	
		     //localization choice
		      int corner=0;
		      
		      switch(buttonChoice) {
		      
			      case Button.ID_UP:
			    	  corner=0;
			      		break;
			      	case Button.ID_LEFT:
			      		corner=1;
			      		break;
			      	case Button.ID_RIGHT:
			      		corner=2;
			      		break;
			      	case Button.ID_DOWN:
			      		corner=3;
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
		      Thread odoCorThread=new Thread(odoCorrection);
		      //odoCorThread.start();
		      
		      Navigation navigation=new Navigation(odometer, CONFIG, odoCorThread);
		      
		      ///////////////////////////////////////// CHANGE LAB PROCEDURE HERE ///////////////////////////////////////
		      							navigation.goToPoint(2, 2);
		      
									      //Start US localization
										 
										  USLocalizer usLoc=new USLocalizer(odometer, navigation, ultraSensor, corner);
										  usLoc.doLocalization();
										  
										  //wait for Button press
										  if(usLoc.isFinished())
											  Button.waitForAnyPress();
										  
										 
									   
										  //Start Light localization
										 
										  LightLocalizer lightLoc=new LightLocalizer(navigation, cSensor, odometer, CONFIG, corner);
										  lightLoc.doLocalization();
										  
										  //Wait until done
										  if(lightLoc.isFinished())
											  Button.waitForAnyPress();
											  System.exit(0);	 
									      }
		    
		  }catch(OdometerExceptions exc) {
			  //instance error, do nothing
		  }	  
	}

}
