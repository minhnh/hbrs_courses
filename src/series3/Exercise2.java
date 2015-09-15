/**
 * @author minhnh
 * @date 2015.09.05
 * @filename Exercise2
 * */

package series3;

import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.NxtContext;
import ch.aplu.robotsim.NxtRobot;
import ch.aplu.robotsim.SensorPort;
import ch.aplu.robotsim.TouchSensor;

public class Exercise2 {

	private NxtRobot robot;
	private Gear gear;
	private TouchSensor ts;
	private double currentDirection = 270.0;
	private boolean turnedRight = false;

	Exercise2() {

		robot = new NxtRobot();
		gear = new Gear();
		ts = new TouchSensor(SensorPort.S3);
		robot.addPart(ts);
		robot.addPart(gear);
		gear.setSpeed(30);
		gear.forward();

		while (true) {
			coursHandler();
		}
	}

	private void coursHandler() {
		currentDirection = gear.getDirection();
		NxtContext.setStatusText("direction: " + currentDirection);
		if (ts.isPressed()) {

			gear.backward(1000);

			/* Turn right if have not done so */
			if (!turnedRight) {
				gear.right(1250);
				gear.forward();
				turnedRight = true;
				return;
			}
			/* Turn to opposite direction if have turned right */
			gear.left(2500);
			gear.forward();
			turnedRight = false;
			return;
		}
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		new Exercise2();
	}

	// ------------------ Environment --------------------------
	static {
		NxtContext.showStatusBar(25);
		NxtContext.useObstacle("sprites/parcours.gif", 250, 250);
		NxtContext.setStartPosition(325, 465);
		NxtContext.setStartDirection(-90);
	}

}
