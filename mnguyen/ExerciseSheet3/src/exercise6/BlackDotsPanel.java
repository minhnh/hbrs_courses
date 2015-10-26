package exercise6;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

import javax.swing.JPanel;

public class BlackDotsPanel extends JPanel {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private int rectangleWidth = 40;
	private int rectangleHeight = 40;
	private int barWidth = 10;
	private int barHeight = 10;
	private Color barColor = new Color(127, 127, 127);
	private Color rectangleColor = new Color(0, 0, 0);
	private Color dotColor = new Color(255, 255, 255);

	public enum FieldName {
		RECTANGLE_WIDTH, RECTANGLE_HEIGHT, BAR_WIDTH, BAR_HEIGHT, RECTANGLE_COLOR, BAR_COLOR, DOT_COLOR
	}

	public BlackDotsPanel() {
	}

	public void updateColor(FieldName fieldName, Color newColor) {
		switch (fieldName) {
		case RECTANGLE_COLOR:
			rectangleColor = newColor;
			break;
		case BAR_COLOR:
			barColor = newColor;
			break;
		case DOT_COLOR:
			dotColor = newColor;
			break;
		default:
			return;
		}
		repaint();
	}

	public void updateDimension(FieldName fieldName, int newDimension) {
		switch (fieldName) {
		case RECTANGLE_HEIGHT:
			rectangleHeight = newDimension;
			break;
		case RECTANGLE_WIDTH:
			rectangleWidth = newDimension;
			break;
		case BAR_HEIGHT:
			barHeight = newDimension;
			break;
		case BAR_WIDTH:
			barWidth = newDimension;
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

		this.setBackground(barColor);

		boolean isDrawingDots = true;
		int drawnRectangleHeight = rectangleHeight;
		int drawnRectangleWidth = rectangleWidth;
		for (int y = 0; y < panelDimension.height; y += (rectangleHeight + barHeight)) {

			if (y + rectangleHeight + barHeight > panelDimension.height) {
				isDrawingDots = false;
				// Adjust edge case
				if (y + rectangleHeight > panelDimension.height) {
					drawnRectangleHeight = panelDimension.height - y;
				}
			}

			for (int x = 0; x < panelDimension.width; x += (rectangleWidth + barWidth)) {
				if (x + rectangleWidth + barWidth > panelDimension.width) {
					isDrawingDots = false;
					// Adjust edge case
					if (x + rectangleWidth > panelDimension.width) {
						drawnRectangleWidth = panelDimension.width - y;
					}
				}
				// Draw Rectangles
				g.setColor(rectangleColor);
				g.fillRect(x, y, drawnRectangleWidth, drawnRectangleHeight);

				if (isDrawingDots) {
					// Draw dots
					g.setColor(dotColor);
					double rectangleDiagDistance = Math.sqrt(barHeight * barHeight + barWidth * barWidth);
					int dotX = (int) (x + rectangleWidth + (double) barWidth / 2 - rectangleDiagDistance / 2 + 0.5);
					int dotY = (int) (y + rectangleHeight + (double) barHeight / 2 - rectangleDiagDistance / 2 + 0.5);
					g.fillArc(dotX, dotY, (int) (rectangleDiagDistance + 0.5) - 1,
							(int) (rectangleDiagDistance + 0.5) - 1, 0, 360);
				}
			}
		}

	}

}
