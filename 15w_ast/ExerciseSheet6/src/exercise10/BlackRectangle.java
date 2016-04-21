package exercise10;

public class BlackRectangle {
    private int topRow;
    private int leftMostColumn;
    private int width;
    private int height;

    public BlackRectangle(int topRow, int leftMostColumn, int width, int height) {
	this.topRow = topRow;
	this.leftMostColumn = leftMostColumn;
	this.width = width;
	this.height = height;
    }

    public void expand(int newWidth, int newHeight) {
	this.width = newWidth;
	this.height = newHeight;
    }

    public int getTopRow() {
	return topRow;
    }

    public int getLeftMostColumn() {
	return leftMostColumn;
    }

    public int getWidth() {
	return width;
    }

    public int getHeight() {
	return height;
    }

}
