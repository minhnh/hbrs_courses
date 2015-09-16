package series3;

import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.NxtRobot;
import ch.aplu.robotsim.SensorPort;
import ch.aplu.robotsim.TouchSensor;

public class Exercise3 {

	private NxtRobot robot;
	private Gear gear;
	private TouchSensor ts;

	Exercise3() {

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

		}
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		new Exercise3();
	}

}
