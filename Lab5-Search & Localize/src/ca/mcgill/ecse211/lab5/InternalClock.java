package ca.mcgill.ecse211.lab5;


/**
 * Class responsible to hold and manage time related variables
 * such as the total time elapsed and the time duration of the search.
 * @author Xavier Pellemans
 *
 */
public class InternalClock {
	
	/**
	 * Variable to hold the starting time of the game
	 */
	private long clockStart;
	
	/**
	 * Variable to hold the starting time of the game
	 */
	private long searchStart;
	
	/**
	 * Maximum allowable time for the robot to complete the course
	 */
	public static final long MAX_ELAPSED_TIME = 300000; // 5 min is 300 000 ms
	
	/**
	 * Maximum allowable time for the robot to complete the course
	 */
	public static final long MAX_SEARCH_TIME = Math.round(MAX_ELAPSED_TIME/2); // half of the total time
	
	/**
	 * Constructor of the internal clock for the game
	 */
	public InternalClock() {
		//nothing to initialize
	}
	
	/**
	 * Starts the clock by saving the starting time
	 */
	public void startClock() {
		clockStart=System.currentTimeMillis();
	}
	
	/**
	 * Starts the searching clock by saving the starting time of the search
	 */
	public void startSearchClock() {
		searchStart=System.currentTimeMillis();
	}

	/**
	 * Gets the time elapsed since play() has been called (in ms)
	 * 
	 * @return The time passed in milliseconds
	 */
	public long getTimeElapsed() {
		return (System.currentTimeMillis()-clockStart);
	}
	
	/**
	 * Gets the time elapsed since the flag finding has been started (in ms)
	 * 
	 * @return The time passed in milliseconds since the start of the search
	 */
	public long getSearchTimeElapsed() {
		return (System.currentTimeMillis()-searchStart);
	}
	
	/**
	 * Gets if the time elapsed since play() has exceeded
	 * the total allowable time for the game.
	 * 
	 * @return True if the time limit has been reached
	 */
	public boolean isTimeUp() {
		if(System.currentTimeMillis()-clockStart>=MAX_ELAPSED_TIME) {
			return true;
		}else {
			return false;
		}
	}
	
	/**
	 * Gets if the search time elapsed since the start of the search
	 * has exceeded the total allowable time for the search.
	 * 
	 * @return True if the time limit has been reached
	 */
	public boolean isSearchTimeUp() {
		if(System.currentTimeMillis()-searchStart>=MAX_SEARCH_TIME) {
			return true;
		}else {
			return false;
		}
	}
}
