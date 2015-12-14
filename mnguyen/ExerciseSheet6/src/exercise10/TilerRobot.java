package exercise10;

public class TilerRobot {
    public static final boolean BLACK = false;
    public static final boolean WHITE = true;
    private final int DEFAULT_SIZE = 10;
    private final int MAX_SIZE = 1000000;
    private final int MIN_RECTANGLE_SIZE = 2;
    private boolean floor[][];

    public TilerRobot() {
	floor = new boolean[DEFAULT_SIZE][DEFAULT_SIZE];
	for (int i = 0; i < DEFAULT_SIZE; i++) {
	    for (int j = 0; j < DEFAULT_SIZE; j++) {
		floor[i][j] = WHITE;
	    }
	}
    }

    public TilerRobot(int m, int n) {
	if (m > MAX_SIZE)
	    m = MAX_SIZE;
	if (n > MAX_SIZE)
	    n = MAX_SIZE;
	floor = new boolean[m][n];
	for (int i = 0; i < m; i++) {
	    for (int j = 0; j < n; j++) {
		floor[i][j] = WHITE;
	    }
	}
    }

    public void setTileColor(int m, int n, boolean color) {
	if (m > floor.length || n > floor[0].length || m < 0 || n < 0)
	    return;
	floor[m][n] = color;
    }

    private boolean isDimensionValid(int topRow, int leftMostColumn, int height, int width) {
	if (height < MIN_RECTANGLE_SIZE || width < MIN_RECTANGLE_SIZE)
	    return false;
	if ((topRow + height > floor.length) || (leftMostColumn + width > floor[0].length))
	    return false;
	if (topRow < 0 || leftMostColumn < 0)
	    return false;
	return true;
    }

    private boolean isNotTouchingOtherBlocks(
	    int topRow, int leftMostColumn, int height, int width) {
	if (!isDimensionValid(topRow, leftMostColumn, height, width))
	    return false;
	// Check upper and lower rows
	for (int i = leftMostColumn - 1; i < leftMostColumn + width + 1; i++) {
	    if (floor[topRow - 1][i] == BLACK)
		return false;
	    if (floor[topRow + height][i] == BLACK)
		return false;
	}
	// Check left and right columns
	for (int i = topRow; i < topRow + height; i++) {
	    if (floor[i][leftMostColumn - 1] == BLACK)
		return false;
	    if (floor[i][leftMostColumn + width] == BLACK)
		return false;
	}
	return true;
    }

    public boolean isValidBlackBlock(
	    int topRow, int leftMostColumn, int height, int width) {
	if (!isNotTouchingOtherBlocks(topRow, leftMostColumn, height, width))
	    return false;
	// Check inside rectangle if there's a white tile
	for (int i = leftMostColumn; i < leftMostColumn + width; i++) {
	    for (int j = topRow; j < topRow + height; j++) {
		if (floor[j][i] == WHITE)
		    return false;
	    }
	}
	return true;
    }

    public boolean canSetBlackBlock(int topRow, int leftMostColumn, int height, int width) {
	if (!isNotTouchingOtherBlocks(topRow, leftMostColumn, height, width))
	    return false;
	// Check inside rectangle if a tile is already black
	for (int i = topRow; i < topRow + height; i++) {
	    for (int j = leftMostColumn; j < leftMostColumn + width; j++) {
		if (floor[i][j] == BLACK)
		    return false;
	    }
	}
	return true;
    }

    public void setBlackBlock(int topRow, int leftMostColumn, int height, int width) {
	if (canSetBlackBlock(topRow, leftMostColumn, height, width)) {
	    for (int i = topRow; i < topRow + height; i++) {
		for (int j = leftMostColumn; j < leftMostColumn + width; j++) {
		    setTileColor(i, j, BLACK);
		}
	    }
	}
    }

    public int getBlackTilesInRow(int rowNum) {
	int tileCount = 0;
	for (int i = 0; i < floor[0].length; i++) {
	    if (floor[rowNum][i] == BLACK) {
		tileCount++;
	    }
	}
	return tileCount;
    }

    public int getBlackTilesInColumn(int ColumnNum) {
	int tileCount = 0;
	for (int i = 0; i < floor.length; i++) {
	    if (floor[i][ColumnNum] == BLACK) {
		tileCount++;
	    }
	}
	return tileCount;
    }

    public boolean[][] getFloor() {
	return floor;
    }

    public int getFloorHeight() {
	return floor.length;
    }

    public int getFloorWidth() {
	return floor[0].length;
    }
}
