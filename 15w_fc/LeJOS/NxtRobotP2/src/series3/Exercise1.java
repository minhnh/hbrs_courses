/**
 * @author minhnh
 * @date 2015.09.05
 * @filename Exercise1
 * */

package series3;

import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.LegoRobot;
import ch.aplu.robotsim.NxtContext;
import ch.aplu.robotsim.SensorPort;
import ch.aplu.robotsim.TouchSensor;

public class Exercise1 {
	private LegoRobot robot;
	private Gear gear;
	private TouchSensor ts;
	private boolean pointRight = false;

	Exercise1() {

		robot = new LegoRobot();
		gear = new Gear();
		ts = new TouchSensor(SensorPort.S3);
		robot.addPart(ts);
		robot.addPart(gear);
		gear.setSpeed(30);
		gear.forward();

		while (true) {
			obstacleHandler();
		}
	}

	private void obstacleHandler() {

		if (ts.isPressed()) {

			gear.backward(1000);

			if (pointRight) {

				gear.right(1250);
				gear.forward(1000);
				gear.right(1250);
				gear.forward();

				pointRight = false;

			} else { /* Pointing left */

				gear.left(1250);
				gear.forward(1000);
				gear.left(1250);
				gear.forward();

				pointRight = true;

			}
		}
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		new Exercise1();
	}

	// ------------------ Environment --------------------------
	static {
		NxtContext.showStatusBar(25);
		NxtContext.useObstacle("sprites/square.gif", 250, 250);
		NxtContext.setStartPosition(362, 130);
		NxtContext.setStartDirection(180);
	}

}
