package exercise11;

import exercise10.TilerRobot;

public class TilerRobotSearchNode {
    private TilerRobot tileInstance;

    public TilerRobotSearchNode() {
	tileInstance = new TilerRobot();
    }

    public TilerRobotSearchNode(int m, int n) {
	tileInstance = new TilerRobot(m, n);
    }

}
