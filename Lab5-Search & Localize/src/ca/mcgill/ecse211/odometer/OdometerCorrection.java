package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab5.Navigation;
import ca.mcgill.ecse211.lab5.TrackExpansion;
import ca.mcgill.ecse211.localizer.ColorSensor;
import ca.mcgill.ecse211.localizer.LightLocalizer;

/**
 * Class for correcting the odometer values using a light sensor from the ColorSensor class
 * Corrects the odometer values when crossing a line.
 * @author Xavier Pellemans
 *
 */
public class OdometerCorrection extends Thread{
	public final double ERROR_THRESHOLD;
	private final ColorSensor lightSensor;
	private final TrackExpansion dynamicTrack;
	private Odometer odo;
	
	/**
	 * Constructor of the OdometerCorrection class.
	 * Initialises the odometer correction.
	 * 
	 * @param sensor ColorSensor in red mode.
	 * @param odo Odometer used by the robot's navigation system.
	 */
	public OdometerCorrection(ColorSensor sensor, Odometer odo, TrackExpansion dynamicTrack) {
		this.lightSensor=sensor;
		this.odo=odo;
		this.dynamicTrack=dynamicTrack;
		this.ERROR_THRESHOLD=dynamicTrack.getLightSensorDistance()*1.2; //to make sure no error occurs at crossings
																		//normally is times 1
	}
	
	
	
	/**
	 * Run method of the OdometerCorrection
	 * Calls the lineCrossed() in ColorSensor
	 * Waits for a line to be read
	 * Corrects if close to line, but not if close to crossing of lines
	 * Runs on a while(true) loop--->.wait() to stop and .notify() to resume
	 */
	@Override
	public void run() {
		double x,y,theta,lineX, lineY;
		while(true){
			if(lightSensor.lineCrossed()) { //does not return until a line is crossed
				x=odo.getX();
				y=odo.getY();
				theta=odo.getTheta(); 
				//Takes the light sensor displacement into account
				lineX=Math.round(x+dynamicTrack.getLightSensorDistance()*Math.sin(Math.toRadians(theta))/Navigation.TILE_SIZE);
				lineY=Math.round(y+dynamicTrack.getLightSensorDistance()*Math.cos(Math.toRadians(theta))/Navigation.TILE_SIZE);
				if(Math.abs(x-lineX*Navigation.TILE_SIZE)<ERROR_THRESHOLD && Math.abs(y-lineY*Navigation.TILE_SIZE)<ERROR_THRESHOLD) {
					//do nothing because close to crossing
				} else if(Math.abs(x-lineX*Navigation.TILE_SIZE)<ERROR_THRESHOLD) {
					//close enough to x grid line
					odo.setX(lineX*Navigation.TILE_SIZE-dynamicTrack.getLightSensorDistance()*Math.sin(Math.toRadians(theta))); //correct X
				}else if(Math.abs(y-lineY*Navigation.TILE_SIZE)<ERROR_THRESHOLD) {
					//close enough to y grid line
					odo.setY(lineY*Navigation.TILE_SIZE-dynamicTrack.getLightSensorDistance()*Math.cos(Math.toRadians(theta))); //correct Y
				}
			}
		}
	}
	

}