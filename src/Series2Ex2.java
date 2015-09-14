import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.LegoRobot;
import ch.aplu.robotsim.LightSensor;
import ch.aplu.robotsim.NxtContext;
import ch.aplu.robotsim.SensorPort;

public class Series2Ex2 {

	private LegoRobot robot;

	Series2Ex2() {

		this.robot = new LegoRobot();
		LightSensor ls = new LightSensor(SensorPort.S3);
		Gear gear = new Gear();
		int stripeCount = 0;
		int v_prev = 0;
		int v = 0;

		robot.addPart(ls);
		ls.activate(true);

		robot.addPart(gear);
		gear.setSpeed(30);
		gear.forward();

		while (true) {
			v_prev = v;
			v = ls.getValue();
			if (v == v_prev) {
				continue;
			} else if (v < v_prev) {
				stripeCount += 1;
				robot.playTone(2000, 1000);
				NxtContext.setStatusText("value: " + stripeCount);
			}
		}
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		new Series2Ex2();
	}

	// ------------------ Environment --------------------------
	static {
		NxtContext.showStatusBar(25);
		NxtContext.setStartPosition(10, 250);
		NxtContext.setStartDirection(0);
		NxtContext.useBackground("sprites/panel.gif");
	}

}
