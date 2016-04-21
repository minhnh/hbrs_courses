import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.NxtRobot;

public class MoveEx2 {

	MoveEx2() {
		NxtRobot robot = new NxtRobot();
		Gear gear = new Gear();
		robot.addPart(gear);
		gear.forward(2000);
		gear.setSpeed(30);
		gear.left(480);
		gear.forward(2000);
		gear.right(480);
		gear.forward();
		// Tools.delay(2000);
		robot.exit();
	}

	public static void main(String[] args) {
		new MoveEx2();
	}

}
