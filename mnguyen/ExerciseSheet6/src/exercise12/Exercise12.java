package exercise12;

import exercise10.BlackRectangle;
import exercise11.TilerRobotSearchNode;

public class Exercise12 {

    public static void main(String[] args) {
	TilerRobotSearchNode tilerSearch = new TilerRobotSearchNode();
	tilerSearch.tileInstance.setBlackBlock(new BlackRectangle(1, 1, 3, 3));
	tilerSearch.printFloor();
    }

}
