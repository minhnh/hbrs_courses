package exercise11;

import exercise10.BlackRectangle;
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

    public boolean rowColumnConstraintsSatisfied() {
	for (int i = 0; i < tileInstance.getFloorHeight(); i++) {
	    if (tileInstance.getBlackTilesInRow(i) != rowConstraints[i])
		return false;
	}
	for (int i = 0; i < tileInstance.getFloorWidth(); i++) {
	    if (tileInstance.getBlackTilesInColumn(i) != columnConstraints[i])
		return false;
	}
	return true;
    }

    private boolean checkConstraintOneRectangle(BlackRectangle rect) {
	/*
	 * Number of remaining black tiles in each row and column must be enough
	 * to form a minimum size rectangle
	 */
	// Check columns
	for (int i = rect.getLeftMostColumn(); i < rect.getLeftMostColumn()
		+ rect.getWidth(); i++) {
	    int remainingColumns = columnConstraints[i] - rect.getHeight()
		    - tileInstance.getBlackTilesInColumn(i);
	    if (remainingColumns < 0 || remainingColumns == 1) {
		return false;
	    }
	}
	// Check rows
	for (int i = rect.getTopRow(); i < rect.getTopRow() + rect.getHeight(); i++) {
	    int remainingRows = rowConstraints[i] - rect.getWidth()
		    - tileInstance.getBlackTilesInRow(i);
	    if (remainingRows < 0 || remainingRows == 1) {
		return false;
	    }
	}
	return true;
    }

    public boolean setRectangle(BlackRectangle rect) {
	if (checkConstraintOneRectangle(rect) && tileInstance.canSetBlackRectangle(rect)) {
	    tileInstance.setBlackRectangle(rect);
	    return true;
	} else {
	    return false;
	}
    }

    public void printFloor() {
	// Row 1: column constraints
	System.out.print("  ");
	for (int i : columnConstraints) {
	    System.out.print(" " + i);
	}
	System.out.println();
	// Row 2 -> end: row constraints and map
	for (int i = 0; i < tileInstance.getFloorHeight(); i++) {
	    System.out.print(" " + rowConstraints[i]);
	    for (int j = 0; j < tileInstance.getFloorWidth(); j++) {
		boolean color = tileInstance.getTileColor(i, j);
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
