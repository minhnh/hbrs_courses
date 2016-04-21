import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.LegoRobot;
import ch.aplu.robotsim.LightSensor;
import ch.aplu.robotsim.NxtContext;
import ch.aplu.robotsim.SensorPort;

public class Series2Ex6 {

	Series2Ex6() {

		LegoRobot robot = new LegoRobot();
		LightSensor ls1 = new LightSensor(SensorPort.S1);
		LightSensor ls2 = new LightSensor(SensorPort.S2);
		Gear gear = new Gear();
		int v1 = 0;
		int v1Prev = 0;
		int v2 = 0;
		int v2Prev = 0;

		robot.addPart(ls1);
		robot.addPart(ls2);
		ls1.activate(true);
		ls2.activate(true);

		robot.addPart(gear);
		gear.setSpeed(30);
		gear.forward();

		while (true) {
			v1Prev = v1;
			v2Prev = v2;
			v1 = ls1.getValue();
			v2 = ls2.getValue();

			/* No adjustment if no change in sensor values */
			if (Math.abs(v1 - v1Prev) < 10 && Math.abs(v2 - v2Prev) < 10) {
				continue;
			}

			NxtContext.setStatusText("color value sensor 1: " + v1
					+ ", sensor 2: " + v2);

			/* Perform rightArc() if lost */
			if (v1 < 50 && v2 < 50) {
				gear.forward(500);
				gear.right(950);
				gear.leftArc(0.6);
			} else {

				/* Right sensor sees black */
				if (v1 < 50 && v2 > 950) {
					gear.leftArc(0.1);
				}

				/* Left sensor sees black */
				if (v2 < 50 && v1 > 950) {
					gear.rightArc(0.1);
				}

				/* Go straight if on white path */
				if (v1 > 950 && v2 > 950) {
					gear.forward();
				}
			}
		}
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		new Series2Ex6();
	}

	// ------------------ Environment --------------------------
	static {
		NxtContext.showStatusBar(25);
		NxtContext.setStartPosition(20, 70);
		NxtContext.setStartDirection(0);
		NxtContext.useBackground("sprites/antPath.gif");
	}

}
