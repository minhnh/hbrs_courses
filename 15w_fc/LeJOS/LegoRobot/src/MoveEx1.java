import ch.aplu.robotsim.TurtleRobot;

public class MoveEx1 {
	MoveEx1() {
		TurtleRobot robot = new TurtleRobot();
		robot.forward(100);
		robot.left(70);
		robot.forward(50);
		robot.right(90);
		robot.forward(100);
		robot.exit();
	}

	public static void main(String[] args) {
		new MoveEx1();
	}
}
