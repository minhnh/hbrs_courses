import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.LegoRobot;
import ch.aplu.robotsim.LightSensor;
import ch.aplu.robotsim.NxtContext;
import ch.aplu.robotsim.SensorPort;
import ch.aplu.robotsim.Tools;

public class Series2Ex5 {

	Series2Ex5() {

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
		gear.setSpeed(50);
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

			/* Go straight if lost */
			if (v1 > 950 && v2 > 950)
				gear.forward();

			/* Right sensor sees white */
			if (v1 > 950) {
				gear.leftArc(0.1);
				Tools.delay(150);
			}

			/* Left sensor sees white */
			if (v2 > 950) {
				gear.rightArc(0.1);
				Tools.delay(150);
			}

			/* Go straight if on black path */
			if (v1 < 50 && v2 < 50) {
				gear.forward();
			}
		}
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		new Series2Ex5();
	}

	// ------------------ Environment --------------------------
	static {
		NxtContext.showStatusBar(25);
		NxtContext.setStartPosition(20, 250);
		NxtContext.setStartDirection(0);
		NxtContext.useBackground("sprites/roundPath.gif");
	}

}
