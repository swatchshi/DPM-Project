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
		   // UltrasonicPoller usPoller = null;													// returned
	
		    // Odometer related objects
		    rightMotor.synchronizeWith(new EV3LargeRegulatedMotor[] { leftMotor });  //to sync the wheels
		    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD, CONFIG); 
		    
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
		      lcd.drawString("    ^^^^ CORNER 0 ^^^^   ", 0, 0);
		      lcd.drawString("  < __________________ > ", 0, 1);
		      lcd.drawString(" < CORNER 1 | CORNER 2 >", 0, 2);
		      lcd.drawString("  <_________|__________> ", 0, 3);
		      lcd.drawString("    vvvv CORNER 3 vvvv   ", 0, 4);
	
		      buttonChoice = Button.waitForAnyPress(); // Record choice (left,right press)
	
		     //localization choice
		      
		      switch(buttonChoice) {
		      
		      case Button.ID_UP:
		    	  
		    	  //TODO: CORNER 0
		      		
		      		break;
		      	case Button.ID_LEFT:
		      		
		      	//TODO: CORNER 1
		      		break;
		      	case Button.ID_RIGHT:
		      		
		      	//TODO: CORNER 2
		      		
		      		break;
		      	case Button.ID_DOWN:
		      		
		      	//TODO: CORNER 3
		      		
		      		break;
		      	default: System.exit(0);
		      		break;
		      }
		      
		      // Start odometer and display threads
		      Thread odoThread = new Thread(odometer);
		      odoThread.start();
		      Thread odoDisplayThread = new Thread(odometryDisplay);
		      odoDisplayThread.start();
		      Navigation navigation=new Navigation(odometer, CONFIG);
		      
		      ///////////////////////////////////////// CHANGE LAB PROCEDURE HERE ///////////////////////////////////////
		      
									      //Start US localization
										  UltrasonicSensor ultraSensor=new UltrasonicSensor(ultraSSensor, usData);
										  USLocalizer usLoc=new USLocalizer(odometer, navigation, ultraSensor);
										  usLoc.doLocalization();
										  
										  //wait for Button press
										  if(usLoc.isFinished())
											  Button.waitForAnyPress();
										  
										 
									   
										  //Start Light localization
										  ColorSensor cSensor=new ColorSensor(lightSensor);
										  LightLocalizer lightLoc=new LightLocalizer(navigation, cSensor, odometer, CONFIG);
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
