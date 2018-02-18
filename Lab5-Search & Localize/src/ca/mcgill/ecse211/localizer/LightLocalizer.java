package ca.mcgill.ecse211.localizer;

import ca.mcgill.ecse211.lab5.*;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.utility.Delay;

/**
 * Class for handling the light localization procedure
 * 
 * @author Xavier Pellemans
 * @author Thomas Bahen
 */
public class LightLocalizer {
	
		/**
		 * Variable for light localization
		 */
		private static final double LIGHT_SENSOR_DISTANCE = 6; //from 4 to 8
		private final Navigation navigation;
		private final ColorSensor lightSensor;
		private final Odometer odo;
		private final Lab5.RobotConfig config;
		private boolean lightLocalizerDone; //tells when the localization is over
		private int corner;
		

		/**
		 * Create a lightLocalizer object
		 * 
		 * @param navigation Navigation used
		 * @param lightSensor EV3ColorSensor used
		 * @param odo Odometer used
		 * @param config The Lab5.RobotConfig, i.e. the wheel positioning
		 */
		public LightLocalizer(Navigation navigation, ColorSensor lightSensor, Odometer odo, Lab5.RobotConfig config, int corner) {
			this.odo = odo;
			this.navigation = navigation;
			this.lightSensor = lightSensor;
			this.lightLocalizerDone=false;
			this.config=config;
			this.corner=corner;
		}

		
		/**
		 * Perform localization
		 */
		public void doLocalization() {
			double x=0,y=0,dTheta=0, thetaY, thetaX;
			double[] lineAngles= new double[4];
			// travel to location
			// cross a line and stop
			Sound.beep();
			navigation.turnTo((45-corner*90)%360);
			navigation.travelForward();
			if(lightSensor.lineCrossed()) { //wait to cross a line
				navigation.stopMotors();
			}
			
			switch(config) {
				case PROPULSION:
					//travel to "center" the robot on the cross
					navigation.travel(LIGHT_SENSOR_DISTANCE*Math.cos(Math.PI/4), LIGHT_SENSOR_DISTANCE*Math.cos(Math.PI/4));
					
					//Read in the angles		
					for(int lineCounter=0; lineCounter<4; lineCounter++) {
						
						navigation.rotate(Navigation.Turn.CLOCK_WISE);//Rotate
						
						Delay.msDelay(500); //wait for initialization to make sure not initializing on the line
						
						if (lightSensor.lineCrossed()) { //wait to cross a line
							navigation.stopMotors(); //stop
							lineAngles[lineCounter] = odo.getTheta(); //positive x, negative y, negative x, positive y
						}
					}
					
					//Angle and position calculation
					thetaY = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[1]),Math.toRadians(lineAngles[3]));
					x = LIGHT_SENSOR_DISTANCE * Math.cos(thetaY / 2);
					
					thetaX = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[0]), Math.toRadians(lineAngles[2]));
					y = LIGHT_SENSOR_DISTANCE * Math.cos(thetaX / 2);
					
					dTheta = Math.toDegrees(thetaY) / 2 + 270 - lineAngles[3];
					break;
				case TRACTION:
					//travel back to "center" the robot on the cross
					navigation.travel(-LIGHT_SENSOR_DISTANCE*Math.cos(Math.PI/4), -LIGHT_SENSOR_DISTANCE*Math.cos(Math.PI/4));
					
					//Read in the angles
					for(int lineCounter=0; lineCounter<4; lineCounter++) {
						
						navigation.rotate(Navigation.Turn.CLOCK_WISE);//Rotate
						
						Delay.msDelay(500); //wait for initialization to make sure not initializing on the line
						
						if (lightSensor.lineCrossed()) { //wait to cross a line
							navigation.stopMotors(); //stop
							lineAngles[lineCounter] = odo.getTheta(); //negative x, positive y, positve x, negative y
						}
					}
					
					//Angle and position calculation
					thetaY = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[1]),Math.toRadians(lineAngles[3]));
					x = - LIGHT_SENSOR_DISTANCE * Math.cos(thetaY / 2);
					
					thetaX = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[0]), Math.toRadians(lineAngles[2]));
					y = - LIGHT_SENSOR_DISTANCE * Math.cos(thetaX / 2);
					
					dTheta = Math.toDegrees(thetaY) / 2 + 270- lineAngles[3]; ////////////////////////////////////////////////////////////COULD BE WRONG HERE
					break;
					
			}
			
			odo.setXYT(x, y, odo.getTheta() + dTheta);
			
			//Finally go to origin
			navigation.goToPoint(0, 0);
			navigation.turnTo(0);
			lightLocalizerDone=true;
		}	

		
		
		/**
		 * Gets if the localization is done
		 * @return if the localization has been made
		 */
		public boolean isFinished() {
			return lightLocalizerDone;
		}
}
