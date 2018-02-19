package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab5.Navigation;
import ca.mcgill.ecse211.localizer.ColorSensor;

public class OdometerCorrection extends Thread{
	public final int ERROR_THRESHOLD=2;
	private final ColorSensor lightSensor;
	private Odometer odo;
	
	
	public OdometerCorrection(ColorSensor sensor, Odometer odo) {
		this.lightSensor=sensor;
		this.odo=odo;
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
		double x,y,lineX, lineY;
		while(true){
			if(lightSensor.lineCrossed()) {
				x=odo.getX();
				y=odo.getY();
				lineX=Math.round(x/Navigation.TILE_SIZE);
				lineY=Math.round(y/Navigation.TILE_SIZE);
				if(Math.abs(x-lineX*Navigation.TILE_SIZE)<ERROR_THRESHOLD && Math.abs(y-lineY*Navigation.TILE_SIZE)<ERROR_THRESHOLD) {
					//do nothing because close to crossing
				} else if(Math.abs(x-lineX*Navigation.TILE_SIZE)<ERROR_THRESHOLD) {
					odo.setX(lineX*Navigation.TILE_SIZE); //correct X
				}else if(Math.abs(y-lineY*Navigation.TILE_SIZE)<ERROR_THRESHOLD) {
					odo.setY(lineY*Navigation.TILE_SIZE); //correct Y
				}
			}
			
		}
	}
	

}