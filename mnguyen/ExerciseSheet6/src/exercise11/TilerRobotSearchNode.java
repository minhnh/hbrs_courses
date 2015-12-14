package exercise11;

import exercise10.TilerRobot;

public class TilerRobotSearchNode {
    public TilerRobot tileInstance;
    public int rowConstraints[];
    public int columnConstraints[];

    public TilerRobotSearchNode() {
	tileInstance = new TilerRobot();
	rowConstraints = new int[tileInstance.getFloorHeight()];
	columnConstraints = new int[tileInstance.getFloorWidth()];
    }

    public TilerRobotSearchNode(int m, int n) {
	tileInstance = new TilerRobot(m, n);
	rowConstraints = new int[tileInstance.getFloorHeight()];
	columnConstraints = new int[tileInstance.getFloorWidth()];
    }

    public void setRowConstraints(int constraints[]) {
	for (int i = 0; i < constraints.length; i++) {
	    if (i > rowConstraints.length - 1) {
		return;
	    }
	    rowConstraints[i] = constraints[i];
	}
    }

    public void setColumnConstraints(int constraints[]) {
	for (int i = 0; i < constraints.length; i++) {
	    if (i > columnConstraints.length - 1) {
		return;
	    }
	    columnConstraints[i] = constraints[i];
	}
    }

    public boolean rectangleBlockConstraintSatisfied() {
	// boolean floor[][] = tileInstance.getFloor();
	return false;
    }

    public void printFloor() {
	boolean floor[][] = tileInstance.getFloor();
	// Row 1: column constraints
	System.out.print("  ");
	for (int i : columnConstraints) {
	    System.out.print(" " + i);
	}
	System.out.println();
	// Row 2 -> end: row constraints and map
	for (int i = 0; i < rowConstraints.length; i++) {
	    System.out.print(" " + rowConstraints[i]);
	    for (boolean color : floor[i]) {
		if (color == TilerRobot.BLACK) {
		    System.out.print(" B");
		} else if (color == TilerRobot.WHITE) {
		    System.out.print("  ");
		}
	    }
	    System.out.println();
	}
    }
}
