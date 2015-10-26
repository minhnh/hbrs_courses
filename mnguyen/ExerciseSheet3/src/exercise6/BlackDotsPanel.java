package exercise6;

import java.awt.Color;
import java.awt.Graphics;

import javax.swing.JPanel;

public class BlackDotsPanel extends JPanel {
	private int rectangleWidth = 20;
	private int rectanlgeHeight = 20;
	private int barWidth = 5;
	private int barHeight = 5;
	private Color barColor = new Color(127, 127, 127);
	private Color rectangleColor = new Color(0, 0, 0);
	private Color dotColor = new Color(255, 255, 255);

	public BlackDotsPanel() {

	}

	protected void paintComponent(Graphics g) {
		super.paintComponent(g);

	}

}
