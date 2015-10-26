package exercise6;

import java.awt.CardLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.EventQueue;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.GroupLayout;
import javax.swing.GroupLayout.Alignment;
import javax.swing.JButton;
import javax.swing.JColorChooser;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JSlider;
import javax.swing.JTabbedPane;

public class Exercise6 {
	private JFrame frame;
	private JPanel chooseColorPanel;
	private JPanel itemSizePanel;
	private JPanel picturePanel;
	private JPanel parallelLinesPanel;
	private Color color = new Color(0, 0, 0);
	private JPanel colorChosenPanel;

	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					Exercise6 window = new Exercise6();
					window.frame.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the application.
	 */
	public Exercise6() {
		initialize();
	}

	/**
	 * Initialize the contents of the frame.
	 */
	private void initialize() {
		frame = new JFrame("AST WS 2015 - Exercise 6 (by Minh Nguyen)");
		frame.setBounds(100, 100, 640, 420);
		// frame.setResizable(false);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		JTabbedPane tabbedPane = new JTabbedPane(JTabbedPane.TOP);
		GroupLayout groupLayout = new GroupLayout(frame.getContentPane());
		groupLayout.setHorizontalGroup(groupLayout.createParallelGroup(Alignment.LEADING).addComponent(tabbedPane,
				GroupLayout.PREFERRED_SIZE, 637, Short.MAX_VALUE));
		groupLayout.setVerticalGroup(groupLayout.createParallelGroup(Alignment.LEADING).addComponent(tabbedPane,
				GroupLayout.DEFAULT_SIZE, 492, Short.MAX_VALUE));

		JPanel blackDotsPanel = new JPanel();
		tabbedPane.addTab("Black Dots", null, blackDotsPanel, null);

		picturePanel = new JPanel();
		picturePanel.setMaximumSize(new Dimension(300, 300));
		picturePanel.setMinimumSize(new Dimension(300, 300));

		/* Initialize Color Picker Frame */
		initializeColorPicker();

		/* Initialize Item Size Frame */
		initializeItemSize();

		GroupLayout gl_blackDotsPanel = new GroupLayout(blackDotsPanel);
		gl_blackDotsPanel.setHorizontalGroup(gl_blackDotsPanel.createParallelGroup(Alignment.LEADING)
				.addGroup(gl_blackDotsPanel.createSequentialGroup().addGap(18)
						.addGroup(gl_blackDotsPanel.createParallelGroup(Alignment.TRAILING)
								.addComponent(itemSizePanel, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE,
										Short.MAX_VALUE)
								.addComponent(chooseColorPanel, GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
										GroupLayout.PREFERRED_SIZE))
						.addGap(18)
						.addComponent(picturePanel, GroupLayout.PREFERRED_SIZE, 354, GroupLayout.PREFERRED_SIZE)
						.addContainerGap(30, Short.MAX_VALUE)));
		gl_blackDotsPanel.setVerticalGroup(gl_blackDotsPanel.createParallelGroup(Alignment.LEADING)
				.addGroup(gl_blackDotsPanel.createSequentialGroup().addContainerGap()
						.addGroup(gl_blackDotsPanel.createParallelGroup(Alignment.LEADING)
								.addComponent(picturePanel, GroupLayout.PREFERRED_SIZE, 320, GroupLayout.PREFERRED_SIZE)
								.addGroup(gl_blackDotsPanel.createSequentialGroup()
										.addComponent(chooseColorPanel, GroupLayout.PREFERRED_SIZE, 113,
												GroupLayout.PREFERRED_SIZE)
										.addGap(18).addComponent(itemSizePanel, GroupLayout.PREFERRED_SIZE,
												GroupLayout.DEFAULT_SIZE, GroupLayout.PREFERRED_SIZE)))
						.addContainerGap(76, Short.MAX_VALUE)));

		JSlider slider = new JSlider();
		slider.setValue(20);
		slider.setMinimum(10);
		slider.setPaintTicks(true);
		GridBagConstraints gbc_slider = new GridBagConstraints();
		gbc_slider.gridwidth = 2;
		gbc_slider.gridx = 0;
		gbc_slider.gridy = 4;
		itemSizePanel.add(slider, gbc_slider);

		blackDotsPanel.setLayout(gl_blackDotsPanel);

		parallelLinesPanel = new JPanel();
		tabbedPane.addTab("ParallelLines", null, parallelLinesPanel, null);
		frame.getContentPane().setLayout(groupLayout);
	}

	private void initializeColorPicker() {
		chooseColorPanel = new JPanel();
		chooseColorPanel.setBorder(BorderFactory.createTitledBorder("Choose Item Color"));
		GridBagLayout gbl_chooseColorPanel = new GridBagLayout();
		gbl_chooseColorPanel.columnWidths = new int[] { 100, 100, 0 };
		gbl_chooseColorPanel.rowHeights = new int[] { 30, 30, 30, 0 };
		gbl_chooseColorPanel.columnWeights = new double[] { 0.0, 1.0, Double.MIN_VALUE };
		gbl_chooseColorPanel.rowWeights = new double[] { 0.0, 0.0, 0.0, Double.MIN_VALUE };
		chooseColorPanel.setLayout(gbl_chooseColorPanel);

		JColorChooser tcc = new JColorChooser(chooseColorPanel.getForeground());
		tcc.setPreviewPanel(new JPanel());

		JButton btnPickColor = new JButton("Pick Color");
		btnPickColor.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				color = JColorChooser.showDialog(tcc, "Choose Color", color);
				colorChosenPanel.setBackground(color);
			}
		});

		JRadioButton rdbtnRectangle = new JRadioButton("Rectangles");
		GridBagConstraints gbc_rdbtnRectangle = new GridBagConstraints();
		gbc_rdbtnRectangle.anchor = GridBagConstraints.WEST;
		gbc_rdbtnRectangle.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnRectangle.gridx = 0;
		gbc_rdbtnRectangle.gridy = 0;
		chooseColorPanel.add(rdbtnRectangle, gbc_rdbtnRectangle);

		colorChosenPanel = new JPanel();
		GridBagConstraints gbc_ChosenPanel = new GridBagConstraints();
		gbc_ChosenPanel.insets = new Insets(0, 0, 5, 0);
		gbc_ChosenPanel.fill = GridBagConstraints.BOTH;
		gbc_ChosenPanel.gridx = 1;
		gbc_ChosenPanel.gridy = 0;
		chooseColorPanel.add(colorChosenPanel, gbc_ChosenPanel);

		JRadioButton rdbtnBars = new JRadioButton("Bars");
		GridBagConstraints gbc_rdbtnBars = new GridBagConstraints();
		gbc_rdbtnBars.anchor = GridBagConstraints.WEST;
		gbc_rdbtnBars.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnBars.gridx = 0;
		gbc_rdbtnBars.gridy = 1;
		chooseColorPanel.add(rdbtnBars, gbc_rdbtnBars);
		GridBagConstraints gbc_btnPickColor = new GridBagConstraints();
		gbc_btnPickColor.fill = GridBagConstraints.HORIZONTAL;
		gbc_btnPickColor.insets = new Insets(0, 0, 5, 0);
		gbc_btnPickColor.gridx = 1;
		gbc_btnPickColor.gridy = 1;
		chooseColorPanel.add(btnPickColor, gbc_btnPickColor);

		JRadioButton rdbtnDots = new JRadioButton("Dots");
		GridBagConstraints gbc_rdbtnDots = new GridBagConstraints();
		gbc_rdbtnDots.anchor = GridBagConstraints.WEST;
		gbc_rdbtnDots.insets = new Insets(0, 0, 0, 5);
		gbc_rdbtnDots.gridx = 0;
		gbc_rdbtnDots.gridy = 2;
		chooseColorPanel.add(rdbtnDots, gbc_rdbtnDots);

		JButton buttonSetColor = new JButton("Set Color");
		GridBagConstraints gbc_buttonSetColor = new GridBagConstraints();
		gbc_buttonSetColor.fill = GridBagConstraints.HORIZONTAL;
		gbc_buttonSetColor.gridx = 1;
		gbc_buttonSetColor.gridy = 2;
		chooseColorPanel.add(buttonSetColor, gbc_buttonSetColor);
		picturePanel.setLayout(new CardLayout(0, 0));
	}

	private void initializeItemSize() {

		itemSizePanel = new JPanel();
		itemSizePanel.setBorder(BorderFactory.createTitledBorder("Choose Item Size"));

		GridBagLayout gbl_itemSizePanel = new GridBagLayout();
		gbl_itemSizePanel.columnWidths = new int[] { 100, 100, 0 };
		gbl_itemSizePanel.rowHeights = new int[] { 0, 30, 30, 30, 30, 0 };
		gbl_itemSizePanel.columnWeights = new double[] { 0.0, 1.0, Double.MIN_VALUE };
		gbl_itemSizePanel.rowWeights = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE };
		itemSizePanel.setLayout(gbl_itemSizePanel);

		JRadioButton rdbtnRectangleWidth = new JRadioButton("Rect. Width");
		GridBagConstraints gbc_rdbtnRectangleWidth = new GridBagConstraints();
		gbc_rdbtnRectangleWidth.anchor = GridBagConstraints.WEST;
		gbc_rdbtnRectangleWidth.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnRectangleWidth.gridx = 0;
		gbc_rdbtnRectangleWidth.gridy = 0;
		itemSizePanel.add(rdbtnRectangleWidth, gbc_rdbtnRectangleWidth);

		JRadioButton rdbtnRectangleHeight = new JRadioButton("Rect. Height");
		GridBagConstraints gbc_rdbtnRectangleHeight = new GridBagConstraints();
		gbc_rdbtnRectangleHeight.anchor = GridBagConstraints.WEST;
		gbc_rdbtnRectangleHeight.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnRectangleHeight.gridx = 0;
		gbc_rdbtnRectangleHeight.gridy = 1;
		itemSizePanel.add(rdbtnRectangleHeight, gbc_rdbtnRectangleHeight);

		JRadioButton rdbtnBarWidth = new JRadioButton("Bar Width");
		GridBagConstraints gbc_rdbtnBarWidth = new GridBagConstraints();
		gbc_rdbtnBarWidth.anchor = GridBagConstraints.WEST;
		gbc_rdbtnBarWidth.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnBarWidth.gridx = 0;
		gbc_rdbtnBarWidth.gridy = 2;
		itemSizePanel.add(rdbtnBarWidth, gbc_rdbtnBarWidth);

		JButton buttonSetSize = new JButton("Set Size");
		GridBagConstraints gbc_buttonSetSize = new GridBagConstraints();
		gbc_buttonSetSize.insets = new Insets(0, 0, 5, 0);
		gbc_buttonSetSize.fill = GridBagConstraints.HORIZONTAL;
		gbc_buttonSetSize.gridx = 1;
		gbc_buttonSetSize.gridy = 2;
		itemSizePanel.add(buttonSetSize, gbc_buttonSetSize);

		JRadioButton rdbtnBarHeight = new JRadioButton("Bar Height");
		GridBagConstraints gbc_rdbtnBarHeight = new GridBagConstraints();
		gbc_rdbtnBarHeight.anchor = GridBagConstraints.WEST;
		gbc_rdbtnBarHeight.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnBarHeight.gridx = 0;
		gbc_rdbtnBarHeight.gridy = 3;
		itemSizePanel.add(rdbtnBarHeight, gbc_rdbtnBarHeight);

	}
}
