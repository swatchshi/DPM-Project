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
		public static final double LIGHT_SENSOR_DISTANCE = 6; //from 4 to 8
		private final Navigation navigation;
		private final ColorSensor lightSensor;
		private final Odometer odo;
		private final Lab5.RobotConfig config;
		private boolean lightLocalizerDone; //tells when the localization is over
		

		/**
		 * Create a lightLocalizer object
		 * 
		 * @param navigation Navigation used
		 * @param lightSensor EV3ColorSensor used
		 * @param odo Odometer used
		 * @param config The Lab5.RobotConfig, i.e. the wheel positioning
		 */
		public LightLocalizer(Navigation navigation, ColorSensor lightSensor, Odometer odo, Lab5.RobotConfig config) {
			this.odo = odo;
			this.navigation = navigation;
			this.lightSensor = lightSensor;
			this.lightLocalizerDone=false;
			this.config=config;
		}

		
		/**
		 * Perform localization
		 */
		public void doLocalization(int corner) {
			Sound.beep();
			navigation.turnTo(45);
			navigation.travelForward();
			if(lightSensor.lineCrossed()) { //wait to cross a line
				navigation.stopMotors();
			}
			
			//travel to "center" the robot on the cross
			navigation.travel(LIGHT_SENSOR_DISTANCE*Math.cos(Math.PI/4), LIGHT_SENSOR_DISTANCE*Math.cos(Math.PI/4));
			
			
			
			//Read in the angles
			double[] lineAngles = new double[4];
			
			for(int lineCounter=0; lineCounter<4; lineCounter++) {
				
				navigation.rotate(Navigation.Turn.CLOCK_WISE);//Rotate
				
				Delay.msDelay(500); //wait for initialization to make sure not initializing on the line
				
				if (lightSensor.lineCrossed()) { //wait to cross a line
					navigation.stopMotors(); //stop
					lineAngles[lineCounter] = odo.getTheta(); //positive x, negative y, negative x, positive y
				}
			}
			
			//Angle and position calculation
			double thetaY = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[1]),Math.toRadians(lineAngles[3]));
			double x = LIGHT_SENSOR_DISTANCE * Math.cos(thetaY / 2)+Navigation.TILE_SIZE;
			
			double ThetaX = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[0]), Math.toRadians(lineAngles[2]));
			double y = LIGHT_SENSOR_DISTANCE * Math.cos(ThetaX / 2)+Navigation.TILE_SIZE;
			
			double dTheta = Math.toDegrees(thetaY) / 2 + 280 - lineAngles[3]; //was always off by 5 degrees when adding 270 degrees
			
			odo.setXYT(x, y, odo.getTheta() + dTheta);
			
			//Finally go to origin
			navigation.goToPoint(1, 1);
			navigation.turnTo(0);
			
			Delay.msDelay(30);
		
			switch(corner) {
				//true position calculation
				case 3:
					odo.setXYT(Navigation.TILE_SIZE, Lab5.Y_GRID_LINES*Navigation.TILE_SIZE, odo.getTheta()-90*corner); //point (1, Ymax, 90)
					break;
				case 2:
					odo.setXYT(Lab5.X_GRID_LINES*Navigation.TILE_SIZE, Lab5.Y_GRID_LINES*Navigation.TILE_SIZE, odo.getTheta()-90*corner); //point (Xmax, Ymax, 180)
					break;
				case 1:
					odo.setXYT(Lab5.X_GRID_LINES*Navigation.TILE_SIZE, Navigation.TILE_SIZE, odo.getTheta()-90*corner); //point (Xmax, 1, 270)
					break;
				case 0:
				default:
					//do nothing
					break;
			}
			Sound.beep();
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
