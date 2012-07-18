package org.team751.inav;

import org.team751.framework.RobotTask;

/**
 * A task that handles inertial navigation to estimate the position of the robot
 * on the field relative to its starting location. <br />
 * <br />
 * This class uses a coordinate system in which the robot starts at (0, 0)
 * facing up/north.
 * 
 * <pre>
 * 						    ^ Positive Y-axis
 * 							|
 * 						_________
 * 						|   ^   |
 * 	<- Negative X-axis	| Robot |  Positive X-axis ->
 * 						|_______|
 * 							|
 * 							Negative Y-axis
 * </pre>
 * 
 * Note that the directions are defined only in reference to
 * the heading of the robot when it starts up. The coordinate
 * system will not change when the robot turns.
 * 
 * @author Sam Crow
 * 
 */
public class InertialNavigationTask extends RobotTask {

	/**
	 * The robot's X-axis location in meters. The positive X axis is to the
	 * right of the robot's starting position.
	 */
	private volatile double x;

	/**
	 * The robot's Y-axis location in meters. The positive Y axis is forward
	 * of the robot's starting position.
	 */
	private volatile double y;
	
	/**
	 * The robot's forward speed in meters per second.
	 */
	private volatile double speed;
	
	/**
	 * The system that provides heading/acceleration data for navigation.
	 */
	private INavDataProvider robot;

}
