import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.LegoRobot;
import ch.aplu.robotsim.Tools;
import ch.aplu.robotsim.TurtleRobot;

public class Series1 {

	private LegoRobot robot;

	Series1(int exerciseNum) {

		this.robot = new LegoRobot();
		switch (exerciseNum) {
		case 1:
			exercise1();
			break;
		case 2:
			exercise2();
			break;
		case 3:
			exercise3();
			break;
		case 4:
			exercise4();
			break;
		}
		robot.exit();
	}

	private void exercise1() {
		((TurtleRobot) robot).forward(100);
		((TurtleRobot) robot).right(180);
		((TurtleRobot) robot).forward(100);
		((TurtleRobot) robot).left(90);
		((TurtleRobot) robot).forward(100);
		((TurtleRobot) robot).right(180);
		((TurtleRobot) robot).forward(100);
		((TurtleRobot) robot).left(90);
		((TurtleRobot) robot).forward(100);
		((TurtleRobot) robot).right(180);
		((TurtleRobot) robot).forward(100);
		((TurtleRobot) robot).left(90);
		((TurtleRobot) robot).forward(100);
		((TurtleRobot) robot).right(180);
		((TurtleRobot) robot).forward(100);
	}

	private void exercise2() {
		/*
		 * Casting here illustrate how TurtleRobot is a sub-class of LegoRobot
		 */
		int i = 2;
		while (i > 0) {
			((TurtleRobot) robot).forward(50);
			((TurtleRobot) robot).right(90);
			((TurtleRobot) robot).forward(50);
			((TurtleRobot) robot).right(90);
			((TurtleRobot) robot).forward(50);
			((TurtleRobot) robot).left(90);
			((TurtleRobot) robot).forward(50);
			((TurtleRobot) robot).left(90);
			i--;
		}
	}

	private void exercise3() {
		Gear gear = new Gear();
		robot.addPart(gear);
		int i = 2;
		while (i > 0) {
			gear.setSpeed(60);
			gear.rightArc(0.2, 3320);
			Tools.delay(1000);
			gear.setSpeed(40);
			gear.left(1700);
			i--;
		}
	}

	private void exercise4() {
		int i = 4;
		Gear gear = new Gear();
		this.robot.addPart(gear);
		while (i > 0) {
			square(gear, 2000);
			i--;
		}
	}

	private void square(Gear gear, int d) {
		gear.setSpeed(50);
		int rotateDuration = 575;
		gear.forward(d);
		gear.right(rotateDuration);
		gear.forward(d);
		gear.right(rotateDuration);
		gear.forward(d);
		gear.right(rotateDuration);
		gear.forward(d);
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		new Series1(4);

	}
}
