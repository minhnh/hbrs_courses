import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.LegoRobot;
import ch.aplu.robotsim.LightSensor;
import ch.aplu.robotsim.NxtContext;
import ch.aplu.robotsim.SensorPort;

public class Series2Ex1 {

	private LegoRobot robot;

	Series2Ex1() {

		this.robot = new LegoRobot();
		LightSensor ls = new LightSensor(SensorPort.S3);
		Gear gear = new Gear();

		robot.addPart(ls);
		ls.activate(true);

		robot.addPart(gear);
		gear.setSpeed(30);
		gear.forward();

		while (true) {
			int v = ls.getValue();
			if (v == 0) {
				gear.left(2600);
				gear.forward();
			}
		}
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		new Series2Ex1();
	}

	// ------------------ Environment --------------------------
	static {
		NxtContext.setStartPosition(250, 250);
		NxtContext.setStartDirection(0);
		NxtContext.useBackground("sprites/blackPanels.gif");
	}
}
