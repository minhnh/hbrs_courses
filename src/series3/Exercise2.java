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
	private int pathStep = 0;

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
		if (ts.isPressed()) {

			gear.backward(1000);

			/* Traverse maze based on prior maze knowledge */
			switch (pathStep) {
			case 0:
			case 3:
			case 6:
				gear.left(1250);
				gear.forward();
				break;
			case 1:
			case 2:
			case 4:
			case 5:
			case 7:
				gear.right(1270);
				gear.forward();
				break;
			default:
				gear.stop();
			}

			pathStep++;

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
