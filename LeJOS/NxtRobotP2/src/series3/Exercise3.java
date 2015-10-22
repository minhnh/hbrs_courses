package series3;

import java.awt.Color;
import java.awt.Point;

import ch.aplu.jgamegrid.GGBitmap;
import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.NxtRobot;
import ch.aplu.robotsim.RobotContext;
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
			gear.right(1250);
			gear.forward();
		}
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		new Exercise3();
	}

	// ------------------ Environment --------------------------
	private static GGBitmap bar(int width, int length, Color color) {
		GGBitmap bm = new GGBitmap(width, length);
		bm.setPaintColor(color);
		bm.fillRectangle(new Point(0, 0), new Point(width - 1, length - 1));
		return bm;
	}

	private static GGBitmap arc(int radius, Color color) {
		GGBitmap bm = new GGBitmap(2 * radius, 2 * radius);
		bm.setPaintColor(color);
		bm.setLineWidth(3);
		// bm.drawCircle(new Point(radius, radius), radius - 1);
		bm.fillArc(new Point(radius, radius), radius - 1, 90, 270);
		return bm;
	}

	static {
		RobotContext.setStartPosition(300, 200);
		RobotContext.setStartDirection(30);
		RobotContext.useObstacle(bar(250, 20, Color.GREEN), 250, 150);
		RobotContext.useObstacle(bar(200, 20, Color.gray), 250, 350);
		RobotContext.useObstacle(bar(20, 250, Color.DARK_GRAY), 150, 250);
		RobotContext.useObstacle(bar(20, 200, Color.red), 350, 250);
		RobotContext.useObstacle(arc(40, Color.black), 250, 250);
	}

}
