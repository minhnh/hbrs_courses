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
import javax.swing.JTextField;
import javax.swing.SwingConstants;

public class Exercise6 {
	private JFrame frame;
	private JPanel picturePanel;
	private Color color = new Color(0, 0, 0);
	private JPanel colorChosenPanel;
	private JTextField textField;
	private JTextField textField_1;
	private JPanel chooseColorPanelBlackDots;
	private JPanel itemSizePanelBlackDots;
	private JPanel chooseColorPanelParallelLines;
	private JPanel itemSizePanelParallelLines;

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

		picturePanel = new JPanel();
		picturePanel.setMaximumSize(new Dimension(300, 300));
		picturePanel.setMinimumSize(new Dimension(300, 300));
		picturePanel.setLayout(new CardLayout(0, 0));

		/* Initialize Black Dots Tab */
		JPanel blackDotsPanel = new JPanel();
		tabbedPane.addTab("Black Dots", null, blackDotsPanel, null);
		initializeColorPickerBlackDots();
		initializeItemSizeBlackDots();

		GroupLayout gl_blackDotsPanel = new GroupLayout(blackDotsPanel);
		gl_blackDotsPanel
				.setHorizontalGroup(gl_blackDotsPanel.createParallelGroup(Alignment.LEADING)
						.addGroup(gl_blackDotsPanel.createSequentialGroup().addGap(18)
								.addGroup(gl_blackDotsPanel.createParallelGroup(Alignment.TRAILING)
										.addComponent(chooseColorPanelBlackDots, GroupLayout.DEFAULT_SIZE, 212,
												Short.MAX_VALUE)
										.addComponent(itemSizePanelBlackDots, GroupLayout.DEFAULT_SIZE,
												GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
								.addGap(18)
								.addComponent(picturePanel, GroupLayout.PREFERRED_SIZE, 354, GroupLayout.PREFERRED_SIZE)
								.addGap(30)));
		gl_blackDotsPanel.setVerticalGroup(gl_blackDotsPanel.createParallelGroup(Alignment.LEADING)
				.addGroup(gl_blackDotsPanel.createSequentialGroup().addContainerGap()
						.addGroup(gl_blackDotsPanel.createParallelGroup(Alignment.LEADING)
								.addComponent(picturePanel, GroupLayout.PREFERRED_SIZE, 320, GroupLayout.PREFERRED_SIZE)
								.addGroup(gl_blackDotsPanel.createSequentialGroup()
										.addComponent(chooseColorPanelBlackDots, GroupLayout.PREFERRED_SIZE, 113,
												GroupLayout.PREFERRED_SIZE)
										.addGap(18).addComponent(itemSizePanelBlackDots, GroupLayout.PREFERRED_SIZE,
												GroupLayout.DEFAULT_SIZE, GroupLayout.PREFERRED_SIZE)))
						.addContainerGap(22, Short.MAX_VALUE)));

		blackDotsPanel.setLayout(gl_blackDotsPanel);

		/* Initialize Parallel Lines Tab */
		JPanel parallelLinesPanel = new JPanel();
		tabbedPane.addTab("ParallelLines", null, parallelLinesPanel, null);
		initializeColorPickerParallelLines();
		initializeItemSizeParallelLines();

		JPanel panel_2 = new JPanel();
		panel_2.setMinimumSize(new Dimension(300, 300));
		panel_2.setMaximumSize(new Dimension(300, 300));
		panel_2.setLayout(new CardLayout(0, 0));

		GroupLayout gl_parallelLinesPanel = new GroupLayout(parallelLinesPanel);
		gl_parallelLinesPanel.setHorizontalGroup(gl_parallelLinesPanel.createParallelGroup(Alignment.LEADING)
				.addGroup(gl_parallelLinesPanel.createSequentialGroup().addGap(18)
						.addGroup(gl_parallelLinesPanel.createParallelGroup(Alignment.LEADING)
								.addComponent(chooseColorPanelParallelLines, GroupLayout.PREFERRED_SIZE, 212,
										GroupLayout.PREFERRED_SIZE)
								.addComponent(itemSizePanelParallelLines, GroupLayout.PREFERRED_SIZE, 212,
										GroupLayout.PREFERRED_SIZE))
						.addGap(18).addComponent(panel_2, GroupLayout.PREFERRED_SIZE, 354, GroupLayout.PREFERRED_SIZE)
						.addContainerGap(30, Short.MAX_VALUE)));
		gl_parallelLinesPanel.setVerticalGroup(gl_parallelLinesPanel.createParallelGroup(Alignment.LEADING)
				.addGroup(gl_parallelLinesPanel.createSequentialGroup().addContainerGap().addGroup(gl_parallelLinesPanel
						.createParallelGroup(Alignment.LEADING)
						.addGroup(gl_parallelLinesPanel.createSequentialGroup()
								.addComponent(chooseColorPanelParallelLines, GroupLayout.PREFERRED_SIZE, 113,
										GroupLayout.PREFERRED_SIZE)
								.addGap(18).addComponent(itemSizePanelParallelLines, GroupLayout.PREFERRED_SIZE, 172,
										GroupLayout.PREFERRED_SIZE))
						.addComponent(panel_2, GroupLayout.PREFERRED_SIZE, 320, GroupLayout.PREFERRED_SIZE))
						.addContainerGap(22, Short.MAX_VALUE)));
		parallelLinesPanel.setLayout(gl_parallelLinesPanel);
		frame.getContentPane().setLayout(groupLayout);
	}

	private void initializeColorPickerBlackDots() {
		chooseColorPanelBlackDots = new JPanel();
		chooseColorPanelBlackDots.setBorder(BorderFactory.createTitledBorder("Choose Item Color"));
		GridBagLayout gbl_chooseColorPanelBlackDots_1 = new GridBagLayout();
		gbl_chooseColorPanelBlackDots_1.columnWidths = new int[] { 100, 100, 0 };
		gbl_chooseColorPanelBlackDots_1.rowHeights = new int[] { 30, 30, 30, 0 };
		gbl_chooseColorPanelBlackDots_1.columnWeights = new double[] { 0.0, 1.0, Double.MIN_VALUE };
		gbl_chooseColorPanelBlackDots_1.rowWeights = new double[] { 0.0, 0.0, 0.0, Double.MIN_VALUE };
		chooseColorPanelBlackDots.setLayout(gbl_chooseColorPanelBlackDots_1);

		JColorChooser tcc = new JColorChooser(chooseColorPanelBlackDots.getForeground());
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
		chooseColorPanelBlackDots.add(rdbtnRectangle, gbc_rdbtnRectangle);

		colorChosenPanel = new JPanel();
		GridBagConstraints gbc_ChosenPanel = new GridBagConstraints();
		gbc_ChosenPanel.insets = new Insets(0, 0, 5, 0);
		gbc_ChosenPanel.fill = GridBagConstraints.BOTH;
		gbc_ChosenPanel.gridx = 1;
		gbc_ChosenPanel.gridy = 0;
		chooseColorPanelBlackDots.add(colorChosenPanel, gbc_ChosenPanel);

		JRadioButton rdbtnBars = new JRadioButton("Bars");
		GridBagConstraints gbc_rdbtnBars = new GridBagConstraints();
		gbc_rdbtnBars.anchor = GridBagConstraints.WEST;
		gbc_rdbtnBars.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnBars.gridx = 0;
		gbc_rdbtnBars.gridy = 1;
		chooseColorPanelBlackDots.add(rdbtnBars, gbc_rdbtnBars);
		GridBagConstraints gbc_btnPickColor = new GridBagConstraints();
		gbc_btnPickColor.fill = GridBagConstraints.HORIZONTAL;
		gbc_btnPickColor.insets = new Insets(0, 0, 5, 0);
		gbc_btnPickColor.gridx = 1;
		gbc_btnPickColor.gridy = 1;
		chooseColorPanelBlackDots.add(btnPickColor, gbc_btnPickColor);

		JRadioButton rdbtnDots = new JRadioButton("Dots");
		GridBagConstraints gbc_rdbtnDots = new GridBagConstraints();
		gbc_rdbtnDots.anchor = GridBagConstraints.WEST;
		gbc_rdbtnDots.insets = new Insets(0, 0, 0, 5);
		gbc_rdbtnDots.gridx = 0;
		gbc_rdbtnDots.gridy = 2;
		chooseColorPanelBlackDots.add(rdbtnDots, gbc_rdbtnDots);

		JButton buttonSetColor = new JButton("Set Color");
		GridBagConstraints gbc_buttonSetColor = new GridBagConstraints();
		gbc_buttonSetColor.fill = GridBagConstraints.HORIZONTAL;
		gbc_buttonSetColor.gridx = 1;
		gbc_buttonSetColor.gridy = 2;
		chooseColorPanelBlackDots.add(buttonSetColor, gbc_buttonSetColor);
	}

	private void initializeItemSizeBlackDots() {

		itemSizePanelBlackDots = new JPanel();
		itemSizePanelBlackDots.setBorder(BorderFactory.createTitledBorder("Choose Item Size"));

		GridBagLayout gbl_itemSizePanelBlackDots_1 = new GridBagLayout();
		gbl_itemSizePanelBlackDots_1.columnWidths = new int[] { 100, 100, 0 };
		gbl_itemSizePanelBlackDots_1.rowHeights = new int[] { 0, 30, 30, 30, 30, 0 };
		gbl_itemSizePanelBlackDots_1.columnWeights = new double[] { 0.0, 1.0, Double.MIN_VALUE };
		gbl_itemSizePanelBlackDots_1.rowWeights = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE };
		itemSizePanelBlackDots.setLayout(gbl_itemSizePanelBlackDots_1);

		JRadioButton rdbtnRectangleWidth = new JRadioButton("Rect. Width");
		GridBagConstraints gbc_rdbtnRectangleWidth = new GridBagConstraints();
		gbc_rdbtnRectangleWidth.anchor = GridBagConstraints.WEST;
		gbc_rdbtnRectangleWidth.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnRectangleWidth.gridx = 0;
		gbc_rdbtnRectangleWidth.gridy = 0;
		itemSizePanelBlackDots.add(rdbtnRectangleWidth, gbc_rdbtnRectangleWidth);

		JRadioButton rdbtnRectangleHeight = new JRadioButton("Rect. Height");
		GridBagConstraints gbc_rdbtnRectangleHeight = new GridBagConstraints();
		gbc_rdbtnRectangleHeight.anchor = GridBagConstraints.WEST;
		gbc_rdbtnRectangleHeight.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnRectangleHeight.gridx = 0;
		gbc_rdbtnRectangleHeight.gridy = 1;
		itemSizePanelBlackDots.add(rdbtnRectangleHeight, gbc_rdbtnRectangleHeight);

		JRadioButton rdbtnBarWidth = new JRadioButton("Bar Width");
		GridBagConstraints gbc_rdbtnBarWidth = new GridBagConstraints();
		gbc_rdbtnBarWidth.anchor = GridBagConstraints.WEST;
		gbc_rdbtnBarWidth.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnBarWidth.gridx = 0;
		gbc_rdbtnBarWidth.gridy = 2;
		itemSizePanelBlackDots.add(rdbtnBarWidth, gbc_rdbtnBarWidth);

		JButton buttonSetSize = new JButton("Set Size");
		GridBagConstraints gbc_buttonSetSize = new GridBagConstraints();
		gbc_buttonSetSize.insets = new Insets(0, 0, 5, 0);
		gbc_buttonSetSize.fill = GridBagConstraints.HORIZONTAL;
		gbc_buttonSetSize.gridx = 1;
		gbc_buttonSetSize.gridy = 2;
		itemSizePanelBlackDots.add(buttonSetSize, gbc_buttonSetSize);

		JRadioButton rdbtnBarHeight = new JRadioButton("Bar Height");
		GridBagConstraints gbc_rdbtnBarHeight = new GridBagConstraints();
		gbc_rdbtnBarHeight.anchor = GridBagConstraints.WEST;
		gbc_rdbtnBarHeight.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnBarHeight.gridx = 0;
		gbc_rdbtnBarHeight.gridy = 3;
		itemSizePanelBlackDots.add(rdbtnBarHeight, gbc_rdbtnBarHeight);

		textField = new JTextField();
		textField.setHorizontalAlignment(SwingConstants.CENTER);
		textField.setText("20");
		GridBagConstraints gbc_textField = new GridBagConstraints();
		gbc_textField.fill = GridBagConstraints.HORIZONTAL;
		gbc_textField.insets = new Insets(0, 0, 5, 0);
		gbc_textField.gridx = 1;
		gbc_textField.gridy = 3;
		itemSizePanelBlackDots.add(textField, gbc_textField);
		textField.setColumns(10);

		JSlider sliderSize = new JSlider();
		sliderSize.setMajorTickSpacing(20);
		sliderSize.setValue(20);
		sliderSize.setMinimum(10);
		sliderSize.setPaintTicks(true);
		GridBagConstraints gbc_sliderSize = new GridBagConstraints();
		gbc_sliderSize.fill = GridBagConstraints.HORIZONTAL;
		gbc_sliderSize.gridwidth = 2;
		gbc_sliderSize.gridx = 0;
		gbc_sliderSize.gridy = 4;
		itemSizePanelBlackDots.add(sliderSize, gbc_sliderSize);
	}

	private void initializeColorPickerParallelLines() {

		chooseColorPanelParallelLines = new JPanel();
		chooseColorPanelParallelLines.setBorder(BorderFactory.createTitledBorder("Choose Item Color"));
		GridBagLayout gbl_chooseColorPanelParallelLines_1 = new GridBagLayout();
		gbl_chooseColorPanelParallelLines_1.columnWidths = new int[] { 100, 100, 0 };
		gbl_chooseColorPanelParallelLines_1.rowHeights = new int[] { 30, 30, 30, 0 };
		gbl_chooseColorPanelParallelLines_1.columnWeights = new double[] { 0.0, 1.0, Double.MIN_VALUE };
		gbl_chooseColorPanelParallelLines_1.rowWeights = new double[] { 0.0, 0.0, 0.0, Double.MIN_VALUE };
		chooseColorPanelParallelLines.setLayout(gbl_chooseColorPanelParallelLines_1);

		JRadioButton rdbtnRectangle = new JRadioButton("Rectangles");
		GridBagConstraints gbc_radioButton = new GridBagConstraints();
		gbc_radioButton.anchor = GridBagConstraints.WEST;
		gbc_radioButton.insets = new Insets(0, 0, 5, 5);
		gbc_radioButton.gridx = 0;
		gbc_radioButton.gridy = 0;
		chooseColorPanelParallelLines.add(rdbtnRectangle, gbc_radioButton);

		JPanel panel_1 = new JPanel();
		GridBagConstraints gbc_panel_1 = new GridBagConstraints();
		gbc_panel_1.fill = GridBagConstraints.BOTH;
		gbc_panel_1.insets = new Insets(0, 0, 5, 0);
		gbc_panel_1.gridx = 1;
		gbc_panel_1.gridy = 0;
		chooseColorPanelParallelLines.add(panel_1, gbc_panel_1);

		JRadioButton radioButton_1 = new JRadioButton("Bars");
		GridBagConstraints gbc_radioButton_1 = new GridBagConstraints();
		gbc_radioButton_1.anchor = GridBagConstraints.WEST;
		gbc_radioButton_1.insets = new Insets(0, 0, 5, 5);
		gbc_radioButton_1.gridx = 0;
		gbc_radioButton_1.gridy = 1;
		chooseColorPanelParallelLines.add(radioButton_1, gbc_radioButton_1);

		JButton button = new JButton("Pick Color");
		GridBagConstraints gbc_button = new GridBagConstraints();
		gbc_button.fill = GridBagConstraints.HORIZONTAL;
		gbc_button.insets = new Insets(0, 0, 5, 0);
		gbc_button.gridx = 1;
		gbc_button.gridy = 1;
		chooseColorPanelParallelLines.add(button, gbc_button);

		JRadioButton radioButton_2 = new JRadioButton("Dots");
		GridBagConstraints gbc_radioButton_2 = new GridBagConstraints();
		gbc_radioButton_2.anchor = GridBagConstraints.WEST;
		gbc_radioButton_2.insets = new Insets(0, 0, 0, 5);
		gbc_radioButton_2.gridx = 0;
		gbc_radioButton_2.gridy = 2;
		chooseColorPanelParallelLines.add(radioButton_2, gbc_radioButton_2);

		JButton button_1 = new JButton("Set Color");
		GridBagConstraints gbc_button_1 = new GridBagConstraints();
		gbc_button_1.fill = GridBagConstraints.HORIZONTAL;
		gbc_button_1.gridx = 1;
		gbc_button_1.gridy = 2;
		chooseColorPanelParallelLines.add(button_1, gbc_button_1);
	}

	private void initializeItemSizeParallelLines() {

		itemSizePanelParallelLines = new JPanel();
		itemSizePanelParallelLines.setBorder(BorderFactory.createTitledBorder("Choose Item Size"));
		GridBagLayout gbl_itemSizePanelParallelLines_1 = new GridBagLayout();
		gbl_itemSizePanelParallelLines_1.columnWidths = new int[] { 100, 100, 0 };
		gbl_itemSizePanelParallelLines_1.rowHeights = new int[] { 0, 30, 30, 30, 30, 0 };
		gbl_itemSizePanelParallelLines_1.columnWeights = new double[] { 0.0, 1.0, Double.MIN_VALUE };
		gbl_itemSizePanelParallelLines_1.rowWeights = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE };
		itemSizePanelParallelLines.setLayout(gbl_itemSizePanelParallelLines_1);

		JRadioButton radioButton_3 = new JRadioButton("Rect. Width");
		GridBagConstraints gbc_radioButton_3 = new GridBagConstraints();
		gbc_radioButton_3.anchor = GridBagConstraints.WEST;
		gbc_radioButton_3.insets = new Insets(0, 0, 5, 5);
		gbc_radioButton_3.gridx = 0;
		gbc_radioButton_3.gridy = 0;
		itemSizePanelParallelLines.add(radioButton_3, gbc_radioButton_3);

		JRadioButton radioButton_4 = new JRadioButton("Rect. Height");
		GridBagConstraints gbc_radioButton_4 = new GridBagConstraints();
		gbc_radioButton_4.anchor = GridBagConstraints.WEST;
		gbc_radioButton_4.insets = new Insets(0, 0, 5, 5);
		gbc_radioButton_4.gridx = 0;
		gbc_radioButton_4.gridy = 1;
		itemSizePanelParallelLines.add(radioButton_4, gbc_radioButton_4);

		JRadioButton radioButton_5 = new JRadioButton("Bar Width");
		GridBagConstraints gbc_radioButton_5 = new GridBagConstraints();
		gbc_radioButton_5.anchor = GridBagConstraints.WEST;
		gbc_radioButton_5.insets = new Insets(0, 0, 5, 5);
		gbc_radioButton_5.gridx = 0;
		gbc_radioButton_5.gridy = 2;
		itemSizePanelParallelLines.add(radioButton_5, gbc_radioButton_5);

		JButton button_2 = new JButton("Set Size");
		GridBagConstraints gbc_button_2 = new GridBagConstraints();
		gbc_button_2.fill = GridBagConstraints.HORIZONTAL;
		gbc_button_2.insets = new Insets(0, 0, 5, 0);
		gbc_button_2.gridx = 1;
		gbc_button_2.gridy = 2;
		itemSizePanelParallelLines.add(button_2, gbc_button_2);

		JRadioButton radioButton_6 = new JRadioButton("Bar Height");
		GridBagConstraints gbc_radioButton_6 = new GridBagConstraints();
		gbc_radioButton_6.anchor = GridBagConstraints.WEST;
		gbc_radioButton_6.insets = new Insets(0, 0, 5, 5);
		gbc_radioButton_6.gridx = 0;
		gbc_radioButton_6.gridy = 3;
		itemSizePanelParallelLines.add(radioButton_6, gbc_radioButton_6);

		textField_1 = new JTextField();
		textField_1.setText("20");
		textField_1.setHorizontalAlignment(SwingConstants.CENTER);
		textField_1.setColumns(10);
		GridBagConstraints gbc_textField_1 = new GridBagConstraints();
		gbc_textField_1.fill = GridBagConstraints.HORIZONTAL;
		gbc_textField_1.insets = new Insets(0, 0, 5, 0);
		gbc_textField_1.gridx = 1;
		gbc_textField_1.gridy = 3;
		itemSizePanelParallelLines.add(textField_1, gbc_textField_1);

		JSlider slider = new JSlider();
		slider.setValue(20);
		slider.setPaintTicks(true);
		slider.setMinimum(10);
		slider.setMajorTickSpacing(20);
		GridBagConstraints gbc_slider = new GridBagConstraints();
		gbc_slider.fill = GridBagConstraints.HORIZONTAL;
		gbc_slider.gridwidth = 2;
		gbc_slider.gridx = 0;
		gbc_slider.gridy = 4;
		itemSizePanelParallelLines.add(slider, gbc_slider);
	}
}
