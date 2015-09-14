import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.LegoRobot;
import ch.aplu.robotsim.LightSensor;
import ch.aplu.robotsim.NxtContext;
import ch.aplu.robotsim.SensorPort;
import ch.aplu.robotsim.Tools;

public class Series2Ex3 {

	Series2Ex3() {

		LegoRobot robot = new LegoRobot();
		LightSensor ls = new LightSensor(SensorPort.S3);
		Gear gear = new Gear();
		int loopCount = 0;
		int v = 0;
		int vPrev = 0;

		robot.addPart(ls);
		ls.activate(true);

		robot.addPart(gear);
		gear.setSpeed(70);
		gear.forward();

		while (true) {
			vPrev = v;
			v = ls.getValue();
			if (Math.abs(v - vPrev) < 10) {
				continue;
			}
			if (loopCount > 3) {
				Tools.delay(700);
				break;
			}
			NxtContext.setStatusText("color value: " + v + ", loop number: "
					+ loopCount);
			if (v > 950) {
				gear.forward();
			} else if (v < 700 && v > 610) {
				gear.leftArc(0.03);
			} else if (v < 610 && v > 100) {
				gear.rightArc(0.03);
			} else if (v < 100) {
				gear.forward();
				loopCount += 1;
			}
		}
		robot.exit();
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		new Series2Ex3();
	}

	// ------------------ Environment --------------------------
	static {
		NxtContext.showStatusBar(25);
		NxtContext.setStartPosition(60, 250);
		NxtContext.setStartDirection(90);
		NxtContext.useBackground("sprites/yellowpath.gif");
	}

}
