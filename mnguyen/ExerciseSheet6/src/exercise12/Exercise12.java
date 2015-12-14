package exercise12;

import exercise10.BlackRectangle;
import exercise11.TilerRobotSearchNode;

public class Exercise12 {

    public static void main(String[] args) {
	TilerRobotSearchNode tilerSearch = new TilerRobotSearchNode();
	int colConstraints[] = { 3, 7, 4, 6, 6, 2, 7, 5, 8, 5 };
	int rowConstraints[] = { 7, 7, 7, 4, 4, 6, 2, 8, 6, 2 };
	tilerSearch.setColumnConstraints(colConstraints);
	tilerSearch.setRowConstraints(rowConstraints);
	if (!tilerSearch.setRectangle(new BlackRectangle(1, 1, 4, 4)))
	    System.out.println("Can't set rectangle");
	tilerSearch.printFloor();
    }

}
