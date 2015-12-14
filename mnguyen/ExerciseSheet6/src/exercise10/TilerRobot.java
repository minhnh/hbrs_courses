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

    private void setTileColor(int m, int n, boolean color) {
	if (m > floor.length || n > floor[0].length || m < 0 || n < 0)
	    return;
	floor[m][n] = color;
    }

    public boolean getTileColor(int m, int n) {
	return floor[m][n];
    }

    private boolean isDimensionValid(BlackRectangle rect) {
	if (rect.getHeight() < MIN_RECTANGLE_SIZE || rect.getWidth() < MIN_RECTANGLE_SIZE)
	    return false;
	if ((rect.getTopRow() + rect.getHeight() > floor.length)
		|| (rect.getLeftMostColumn() + rect.getWidth() > floor[0].length))
	    return false;
	if (rect.getTopRow() < 0 || rect.getLeftMostColumn() < 0)
	    return false;
	return true;
    }

    private boolean isNotTouchingOtherBlocks(BlackRectangle rect) {
	if (!isDimensionValid(rect))
	    return false;
	// Check upper and lower rows
	for (int i = rect.getLeftMostColumn() - 1; i < rect.getLeftMostColumn() + rect.getWidth()
		+ 1; i++) {
	    if (floor[rect.getTopRow() - 1][i] == BLACK)
		return false;
	    if (floor[rect.getTopRow() + rect.getHeight()][i] == BLACK)
		return false;
	}
	// Check left and right columns
	for (int i = rect.getTopRow(); i < rect.getTopRow() + rect.getHeight(); i++) {
	    if (floor[i][rect.getLeftMostColumn() - 1] == BLACK)
		return false;
	    if (floor[i][rect.getLeftMostColumn() + rect.getWidth()] == BLACK)
		return false;
	}
	return true;
    }

    public boolean isValidBlackBlock(
	    BlackRectangle rect) {
	if (!isNotTouchingOtherBlocks(rect))
	    return false;
	// Check inside rectangle if there's a white tile
	for (int i = rect.getLeftMostColumn(); i < rect.getLeftMostColumn()
		+ rect.getWidth(); i++) {
	    for (int j = rect.getTopRow(); j < rect.getTopRow() + rect.getHeight(); j++) {
		if (floor[j][i] == WHITE)
		    return false;
	    }
	}
	return true;
    }

    public boolean canSetBlackBlock(BlackRectangle rect) {
	if (!isNotTouchingOtherBlocks(rect))
	    return false;
	// Check inside rectangle if a tile is already black
	for (int i = rect.getTopRow(); i < rect.getTopRow() + rect.getHeight(); i++) {
	    for (int j = rect.getLeftMostColumn(); j < rect.getLeftMostColumn()
		    + rect.getWidth(); j++) {
		if (floor[i][j] == BLACK)
		    return false;
	    }
	}
	return true;
    }

    public void setBlackBlock(BlackRectangle rect) {
	if (canSetBlackBlock(rect)) {
	    for (int i = rect.getTopRow(); i < rect.getTopRow() + rect.getHeight(); i++) {
		for (int j = rect.getLeftMostColumn(); j < rect.getLeftMostColumn()
			+ rect.getWidth(); j++) {
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

    public int getFloorHeight() {
	return floor.length;
    }

    public int getFloorWidth() {
	return floor[0].length;
    }
}
