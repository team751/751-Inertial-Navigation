package org.team751.inav;

import org.team751.framework.RobotTask;

import edu.wpi.first.wpilibj.Timer;

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
 * Note that the directions are defined only in reference to the heading of the
 * robot when it starts up. The coordinate system will not change when the robot
 * turns.
 * 
 * @author Sam Crow
 * 
 */
public class InertialNavigationTask extends RobotTask {

	/**
	 * The robot's X-axis location in meters. The positive X axis is to the
	 * right of the robot's starting position.
	 */
	private volatile double x = 0;

	/**
	 * The robot's Y-axis location in meters. The positive Y axis is forward of
	 * the robot's starting position.
	 */
	private volatile double y = 0;

	/**
	 * The robot's forward speed in meters per second.
	 */
	private volatile double speed = 0;

	/**
	 * The system that provides heading/acceleration data for navigation.
	 */
	private INavDataProvider robot;

	/**
	 * The number of seconds to wait between navigation cycles
	 */
	private static final double cycleTime = 0.02;

	/**
	 * Used to actually measure the time elapsed between cycles
	 */
	private Timer cycleTimer = new Timer();

	/**
	 * Constructor
	 * 
	 * @param provider
	 *            The object that will provide data to be used in navigation
	 */
	public InertialNavigationTask(INavDataProvider provider) {
		robot = provider;

		cycleTimer.start();
	}

	private void runINav() throws InterruptedException {

		synchronized (this) {

			// Get the actual time that passed since the last loop was done
			double actualCycleTime = cycleTimer.get();
			cycleTimer.reset();

			speed += robot.getAcceleration() * actualCycleTime;

			double distanceTraveled = speed * actualCycleTime;

			double headingRadians = Math.toRadians(robot.getHeading());

			x += distanceTraveled * Math.sin(headingRadians);
			y += distanceTraveled * Math.cos(headingRadians);

		}

		// Wait for the next cycle
		sleep((long) (cycleTime * 1000));
	}

	/**
	 * Get the robot's X-axis location in meters
	 * 
	 * @return The X-axis location
	 */
	public synchronized double getX() {
		return x;
	}

	/**
	 * Get the robot's Y-axis location in meters
	 * 
	 * @return The Y-axis location
	 */
	public synchronized double getY() {
		return y;
	}

	/**
	 * Get the robot's forward speed in meters per second squared
	 * 
	 * @return The speed
	 */
	public synchronized double getSpeed() {
		return speed;
	}

	/**
	 * Set the robot's speed (according to this inertial navigation system) to
	 * zero. This should be used to correct for drift situations in which the
	 * system thinks that the robot is moving when it isn't.
	 */
	public synchronized void setStopped() {
		speed = 0;
	}

	protected void disabled() throws InterruptedException {
		runINav();
	}

	protected void autonomous() throws InterruptedException {
		runINav();
	}

	protected void teleop() throws InterruptedException {
		runINav();
	}

}
