package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab5.Navigation;
import ca.mcgill.ecse211.localizer.ColorSensor;
import ca.mcgill.ecse211.localizer.LightLocalizer;

public class OdometerCorrection extends Thread{
	public final double ERROR_THRESHOLD;
	private final ColorSensor lightSensor;
	private Odometer odo;
	
	
	public OdometerCorrection(ColorSensor sensor, Odometer odo) {
		this.lightSensor=sensor;
		this.odo=odo;
		this.ERROR_THRESHOLD=LightLocalizer.LIGHT_SENSOR_DISTANCE;
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
			if(lightSensor.lineCrossed()) {
				x=odo.getX();
				y=odo.getY();
				theta=odo.getTheta(); 
				//Takes the light sensor displacement into account
				lineX=Math.round(x+LightLocalizer.LIGHT_SENSOR_DISTANCE*Math.sin(Math.toRadians(theta))/Navigation.TILE_SIZE);
				lineY=Math.round(y+LightLocalizer.LIGHT_SENSOR_DISTANCE*Math.cos(Math.toRadians(theta))/Navigation.TILE_SIZE);
				if(Math.abs(x-lineX*Navigation.TILE_SIZE)<ERROR_THRESHOLD && Math.abs(y-lineY*Navigation.TILE_SIZE)<ERROR_THRESHOLD) {
					//do nothing because close to crossing
				} else if(Math.abs(x-lineX*Navigation.TILE_SIZE)<ERROR_THRESHOLD) {
					//close enough to x grid line
					odo.setX(lineX*Navigation.TILE_SIZE-LightLocalizer.LIGHT_SENSOR_DISTANCE*Math.sin(Math.toRadians(theta))); //correct X
				}else if(Math.abs(y-lineY*Navigation.TILE_SIZE)<ERROR_THRESHOLD) {
					//close enough to y grid line
					odo.setY(lineY*Navigation.TILE_SIZE-LightLocalizer.LIGHT_SENSOR_DISTANCE*Math.cos(Math.toRadians(theta))); //correct Y
				}
			}
		}
	}
	

}