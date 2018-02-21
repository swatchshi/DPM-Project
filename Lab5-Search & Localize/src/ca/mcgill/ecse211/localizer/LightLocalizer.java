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
			double x=0,y=0,dTheta=0, thetaY, thetaX;
			double[] lineAngles= new double[4];
			// travel to location
			// cross a line and stop
			Sound.beep();
			navigation.turnTo(45); //points to middle of map
			navigation.travelForward();
			if(lightSensor.lineCrossed()) { //wait to cross a line
				navigation.stopMotors();
			}
			//travel to "center" the robot on the cross
			navigation.travel(LIGHT_SENSOR_DISTANCE*Math.cos(Math.PI/4), LIGHT_SENSOR_DISTANCE*Math.cos(Math.PI/4));
			switch(config) {
				case PROPULSION:
					
					//Read in the angles		
					for(int lineCounter=0; lineCounter<4; lineCounter++) {
						
						navigation.rotate(Navigation.Turn.CLOCK_WISE);//Rotate
						
						Delay.msDelay(500); //wait for initialization to make sure not initializing on the line
						
						if (lightSensor.lineCrossed()) { //wait to cross a line
							navigation.stopMotors(); //stop
							lineAngles[lineCounter] = odo.getTheta(); //positive x[0], negative y[1], negative x[2], positive y [3] --relative to temporary 0 degrees
						}
					}
					
					thetaY = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[1]),Math.toRadians(lineAngles[3]));
					x =  LIGHT_SENSOR_DISTANCE * Math.cos(thetaY / 2) + Navigation.TILE_SIZE;
						
					thetaX = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[0]), Math.toRadians(lineAngles[2]));
					y =  LIGHT_SENSOR_DISTANCE * Math.cos(thetaX / 2) + Navigation.TILE_SIZE;
					
				
					dTheta = Math.toDegrees(thetaY) / 2 + 270 - lineAngles[3]; ////////////////////////////////////////////////////////////COULD BE WRONG HERE
					break;
				
				case TRACTION:
					//travel back to "center" the robot on the cross
					navigation.travel(LIGHT_SENSOR_DISTANCE*Math.cos(Math.PI/4), LIGHT_SENSOR_DISTANCE*Math.cos(Math.PI/4));
					
					//Read in the angles
					for(int lineCounter=0; lineCounter<4; lineCounter++) {
						
						navigation.rotate(Navigation.Turn.CLOCK_WISE);//Rotate
						
						Delay.msDelay(500); //wait for initialization to make sure not initializing on the line
						
						if (lightSensor.lineCrossed()) { //wait to cross a line
							navigation.stopMotors(); //stop
							lineAngles[lineCounter] = odo.getTheta(); //negative x, positive y, positve x, negative y
						}
					}
					
				
					thetaY = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[0]),Math.toRadians(lineAngles[2]));
					x = - LIGHT_SENSOR_DISTANCE * Math.cos(thetaY / 2) + Navigation.TILE_SIZE;
						
					thetaX = USLocalizer.getDiffAngle(Math.toRadians(lineAngles[1]), Math.toRadians(lineAngles[3]));
					y = - LIGHT_SENSOR_DISTANCE * Math.cos(thetaX / 2) + Navigation.TILE_SIZE;
					
					
					dTheta = Math.toDegrees(thetaY) / 2 + 270 - lineAngles[3]; ////////////////////////////////////////////////////////////COULD BE WRONG HERE
					break;
			}
			odo.setXYT(x, y, odo.getTheta() + dTheta);
			Sound.beep();
			navigation.goToPoint(1, 1);  //does localizer like it's in the corner 0
			navigation.turnTo(0); //turns to temporary 0
			
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
