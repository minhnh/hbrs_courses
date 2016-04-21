package exercise6;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

import javax.swing.JPanel;

public class ParallelLinesPicturePanel extends JPanel {

	private static final long serialVersionUID = 1L;
	private int firstRectangleWidth = 40;
	private int secondRectangleWidth = 40;
	private int rectangleHeight = 40;
	private int rectangleOffset = 10;
	private int lineThickness = 3;
	private Color lineColor = new Color(127, 127, 127);
	private Color firstRectangleColor = new Color(0, 0, 0);
	private Color secondRectangleColor = new Color(255, 255, 255);

	public enum ParallelLinesFieldName {
		FIRST_RECTANGLE_WIDTH, SECOND_RECTANGLE_WIDTH, RECTANGLE_HEIGHT, LINE_THICKNESS, OFFSET, FIRST_RECTANGLE_COLOR, SECOND_RECTANGLE_COLOR, LINE_COLOR, UNKNOWN
	}

	public ParallelLinesPicturePanel() {
	}

	public void updateColor(ParallelLinesFieldName fieldName, Color newColor) {
		switch (fieldName) {
		case FIRST_RECTANGLE_COLOR:
			firstRectangleColor = newColor;
			break;
		case SECOND_RECTANGLE_COLOR:
			secondRectangleColor = newColor;
			break;
		case LINE_COLOR:
			lineColor = newColor;
			break;
		default:
			return;
		}
		repaint();
	}

	public void updateDimension(ParallelLinesFieldName fieldName, int newDimension) {
		switch (fieldName) {
		case FIRST_RECTANGLE_WIDTH:
			firstRectangleWidth = newDimension;
			break;
		case SECOND_RECTANGLE_WIDTH:
			secondRectangleWidth = newDimension;
			break;
		case RECTANGLE_HEIGHT:
			rectangleHeight = newDimension;
			break;
		case LINE_THICKNESS:
			lineThickness = newDimension;
			break;
		case OFFSET:
			rectangleOffset = newDimension;
			break;
		default:
			return;
		}
		repaint();
	}

	public Dimension getPreferredSize() {
		return new Dimension(300, 300);
	}

	protected void paintComponent(Graphics g) {
		super.paintComponent(g);

		Dimension panelDimension = this.getPreferredSize();

		this.setBackground(secondRectangleColor);

		boolean isDrawingLines = true;
		int drawnRectangleHeight = rectangleHeight;
		int drawnRectangleWidth = firstRectangleWidth;
		int drawnOffset = 0;
		boolean offsetIncreasing = true;
		for (int y = 0; y < panelDimension.height; y += (rectangleHeight + lineThickness)) {

			if (y + rectangleHeight + lineThickness > panelDimension.height) {
				isDrawingLines = false;
				// Adjust edge case
				if (y + rectangleHeight > panelDimension.height) {
					drawnRectangleHeight = panelDimension.height - y;
				}
			}

			for (int x = 0; x < panelDimension.width; x += (firstRectangleWidth + secondRectangleWidth)) {
				if (x + firstRectangleWidth + secondRectangleWidth > panelDimension.width) {
					// Adjust edge case
					if (x + firstRectangleWidth > panelDimension.width) {
						drawnRectangleWidth = panelDimension.width - y;
					}
				}
				// Draw Rectangles
				g.setColor(firstRectangleColor);
				g.fillRect(x + drawnOffset, y, drawnRectangleWidth, drawnRectangleHeight);
			}

			if (isDrawingLines) {
				// Draw Lines
				g.setColor(lineColor);
				g.fillRect(0, y + rectangleHeight, panelDimension.width, lineThickness);
			}
			if (offsetIncreasing) {
				drawnOffset += rectangleOffset;
			} else {
				drawnOffset -= rectangleOffset;
			}
			if (drawnOffset >= 2 * rectangleOffset) {
				offsetIncreasing = false;
			} else if (drawnOffset <= 0) {
				offsetIncreasing = true;
			}

			isDrawingLines = true;
			drawnRectangleWidth = firstRectangleWidth;
		}

	}

}
