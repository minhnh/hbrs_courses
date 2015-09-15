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

	Exercise1() {

		robot = new LegoRobot();
		gear = new Gear();
		ts = new TouchSensor(SensorPort.S3);
		robot.addPart(ts);
		robot.addPart(gear);
		gear.setSpeed(60);
		gear.forward();

		while (true) {
			obstacleHandler();
		}
	}

	private void obstacleHandler() {

		if (ts.isPressed()) {

			gear.backward(1000);

			NxtContext.setStatusText("direction: " + gear.getDirection());
			if (gear.getDirection() < 10.0) { /* Pointing right */

				/* Turn right until pointing down */
				while (gear.getDirection() < 90) {
					gear.right(40);
					NxtContext.setStatusText("direction: "
							+ gear.getDirection());
				}

				gear.forward(1000);

				/* Turn right until pointing right */
				while (gear.getDirection() < 180) {
					gear.right(40);
					NxtContext.setStatusText("direction: "
							+ gear.getDirection());
				}

				/* Continue forward */
				gear.forward();
			} else if (gear.getDirection() > 170.0) { /* Pointing left */

				/* Turn left until pointing down */
				while (gear.getDirection() > 90) {
					gear.left(40);
					NxtContext.setStatusText("direction: "
							+ gear.getDirection());
				}

				gear.forward(1000);

				/* Turn left until pointing right */
				while (gear.getDirection() > 0) {
					gear.left(40);
					NxtContext.setStatusText("direction: "
							+ gear.getDirection());
				}

				/* Continue forward */
				gear.forward();
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
