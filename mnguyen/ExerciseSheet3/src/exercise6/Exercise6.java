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
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JSlider;
import javax.swing.JTabbedPane;
import javax.swing.JTextField;
import javax.swing.SwingConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

public class Exercise6 {
	private JFrame frame;
	private Color color = new Color(0, 0, 0);
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

		/* Initialize Black Dots Tab */
		BlackDotsPanel blackDotsPanel = new BlackDotsPanel();
		tabbedPane.addTab("Black Dots", null, blackDotsPanel, null);

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

	private void initializeColorPickerParallelLines() {

		chooseColorPanelParallelLines = new JPanel();
		chooseColorPanelParallelLines.setBorder(BorderFactory.createTitledBorder("Choose Item Color"));
		GridBagLayout gbl_chooseColorPanelParallelLines = new GridBagLayout();
		gbl_chooseColorPanelParallelLines.columnWidths = new int[] { 100, 100, 0 };
		gbl_chooseColorPanelParallelLines.rowHeights = new int[] { 30, 30, 30, 0 };
		gbl_chooseColorPanelParallelLines.columnWeights = new double[] { 0.0, 1.0, Double.MIN_VALUE };
		gbl_chooseColorPanelParallelLines.rowWeights = new double[] { 0.0, 0.0, 0.0, Double.MIN_VALUE };
		chooseColorPanelParallelLines.setLayout(gbl_chooseColorPanelParallelLines);

		JPanel colorChosenPanel = new JPanel();
		GridBagConstraints gbc_colorChosenPanel = new GridBagConstraints();
		gbc_colorChosenPanel.fill = GridBagConstraints.BOTH;
		gbc_colorChosenPanel.insets = new Insets(0, 0, 5, 0);
		gbc_colorChosenPanel.gridx = 1;
		gbc_colorChosenPanel.gridy = 0;

		JButton btnPickColor = new JButton("Pick Color");
		GridBagConstraints gbc_btnPickColor = new GridBagConstraints();
		gbc_btnPickColor.fill = GridBagConstraints.HORIZONTAL;
		gbc_btnPickColor.insets = new Insets(0, 0, 5, 0);
		gbc_btnPickColor.gridx = 1;
		gbc_btnPickColor.gridy = 1;

		JRadioButton rdbtnRectangles = new JRadioButton("Rectangles");
		GridBagConstraints gbc_rdbtnRectangles = new GridBagConstraints();
		gbc_rdbtnRectangles.anchor = GridBagConstraints.WEST;
		gbc_rdbtnRectangles.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnRectangles.gridx = 0;
		gbc_rdbtnRectangles.gridy = 0;

		JRadioButton rdbtnBars = new JRadioButton("Bars");
		GridBagConstraints gbc_rdbtnBars = new GridBagConstraints();
		gbc_rdbtnBars.anchor = GridBagConstraints.WEST;
		gbc_rdbtnBars.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnBars.gridx = 0;
		gbc_rdbtnBars.gridy = 1;

		JRadioButton radioButton_2 = new JRadioButton("Dots");
		GridBagConstraints gbc_radioButton_2 = new GridBagConstraints();
		gbc_radioButton_2.anchor = GridBagConstraints.WEST;
		gbc_radioButton_2.insets = new Insets(0, 0, 0, 5);
		gbc_radioButton_2.gridx = 0;
		gbc_radioButton_2.gridy = 2;

		JButton button_1 = new JButton("Set Color");
		GridBagConstraints gbc_button_1 = new GridBagConstraints();
		gbc_button_1.fill = GridBagConstraints.HORIZONTAL;
		gbc_button_1.gridx = 1;
		gbc_button_1.gridy = 2;

		/* Link elements' actions */
		JColorChooser tcc = new JColorChooser(chooseColorPanelParallelLines.getForeground());
		tcc.setPreviewPanel(new JPanel());
		btnPickColor.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				color = JColorChooser.showDialog(tcc, "Choose Color", color);
				colorChosenPanel.setBackground(color);
			}
		});

		/* Add elements to panel */
		chooseColorPanelParallelLines.add(colorChosenPanel, gbc_colorChosenPanel);
		chooseColorPanelParallelLines.add(btnPickColor, gbc_btnPickColor);
		chooseColorPanelParallelLines.add(rdbtnRectangles, gbc_rdbtnRectangles);
		chooseColorPanelParallelLines.add(rdbtnBars, gbc_rdbtnBars);
		chooseColorPanelParallelLines.add(radioButton_2, gbc_radioButton_2);
		chooseColorPanelParallelLines.add(button_1, gbc_button_1);

	}

	private void initializeItemSizeParallelLines() {

		itemSizePanelParallelLines = new JPanel();
		itemSizePanelParallelLines.setBorder(BorderFactory.createTitledBorder("Choose Item Size"));
		GridBagLayout gbl_itemSizePanelParallelLines = new GridBagLayout();
		gbl_itemSizePanelParallelLines.columnWidths = new int[] { 100, 100, 0 };
		gbl_itemSizePanelParallelLines.rowHeights = new int[] { 0, 30, 30, 30, 30, 0 };
		gbl_itemSizePanelParallelLines.columnWeights = new double[] { 0.0, 1.0, Double.MIN_VALUE };
		gbl_itemSizePanelParallelLines.rowWeights = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE };
		itemSizePanelParallelLines.setLayout(gbl_itemSizePanelParallelLines);

		/* Declare Elements */
		JRadioButton radioButton_3 = new JRadioButton("Rect. Width");
		GridBagConstraints gbc_radioButton_3 = new GridBagConstraints();
		gbc_radioButton_3.anchor = GridBagConstraints.WEST;
		gbc_radioButton_3.insets = new Insets(0, 0, 5, 5);
		gbc_radioButton_3.gridx = 0;
		gbc_radioButton_3.gridy = 0;

		JRadioButton radioButton_4 = new JRadioButton("Rect. Height");
		GridBagConstraints gbc_radioButton_4 = new GridBagConstraints();
		gbc_radioButton_4.anchor = GridBagConstraints.WEST;
		gbc_radioButton_4.insets = new Insets(0, 0, 5, 5);
		gbc_radioButton_4.gridx = 0;
		gbc_radioButton_4.gridy = 1;

		JRadioButton radioButton_5 = new JRadioButton("Bar Width");
		GridBagConstraints gbc_radioButton_5 = new GridBagConstraints();
		gbc_radioButton_5.anchor = GridBagConstraints.WEST;
		gbc_radioButton_5.insets = new Insets(0, 0, 5, 5);
		gbc_radioButton_5.gridx = 0;
		gbc_radioButton_5.gridy = 2;

		JRadioButton radioButton_6 = new JRadioButton("Bar Height");
		GridBagConstraints gbc_radioButton_6 = new GridBagConstraints();
		gbc_radioButton_6.anchor = GridBagConstraints.WEST;
		gbc_radioButton_6.insets = new Insets(0, 0, 5, 5);
		gbc_radioButton_6.gridx = 0;
		gbc_radioButton_6.gridy = 3;

		JButton button_2 = new JButton("Set Size");
		GridBagConstraints gbc_button_2 = new GridBagConstraints();
		gbc_button_2.fill = GridBagConstraints.HORIZONTAL;
		gbc_button_2.insets = new Insets(0, 0, 5, 0);
		gbc_button_2.gridx = 1;
		gbc_button_2.gridy = 2;

		JTextField textField_1 = new JTextField();
		textField_1.setText("20");
		textField_1.setHorizontalAlignment(SwingConstants.CENTER);
		textField_1.setColumns(10);
		GridBagConstraints gbc_textField_1 = new GridBagConstraints();
		gbc_textField_1.fill = GridBagConstraints.HORIZONTAL;
		gbc_textField_1.insets = new Insets(0, 0, 5, 0);
		gbc_textField_1.gridx = 1;
		gbc_textField_1.gridy = 3;

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

		/* Link elements' actions */

		/* Add Elements to Panel */
		itemSizePanelParallelLines.add(radioButton_3, gbc_radioButton_3);
		itemSizePanelParallelLines.add(radioButton_4, gbc_radioButton_4);
		itemSizePanelParallelLines.add(radioButton_5, gbc_radioButton_5);
		itemSizePanelParallelLines.add(radioButton_6, gbc_radioButton_6);
		itemSizePanelParallelLines.add(button_2, gbc_button_2);
		itemSizePanelParallelLines.add(textField_1, gbc_textField_1);
		itemSizePanelParallelLines.add(slider, gbc_slider);
	}

	public static void setSliderTextFieldActions(JSlider slider, JTextField textField) {
		textField.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				int value = 0;
				try {
					value = Integer.parseInt(textField.getText());
				} catch (NumberFormatException e) {
					JOptionPane.showMessageDialog(null, "Invalid input");
					return;
				}
				if (value < 0 || value > 255) {
					JOptionPane.showMessageDialog(null, "Input not in range 0-255");
				} else {
					slider.setValue(value);
				}
			}
		});
		slider.addChangeListener(new ChangeListener() {
			public void stateChanged(ChangeEvent arg0) {
				textField.setText(Integer.toString(slider.getValue()));
			}
		});
	}

	public static void set3RadioButtonActions(JRadioButton button1, JRadioButton button2, JRadioButton button3) {
		button1.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				button1.setSelected(true);
				button2.setSelected(false);
				button3.setSelected(false);
			}
		});
		button2.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				button2.setSelected(true);
				button1.setSelected(false);
				button3.setSelected(false);
			}
		});
		button3.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				button3.setSelected(true);
				button2.setSelected(false);
				button1.setSelected(false);
			}
		});
	}

	public static void set4RadioButtonActions(JRadioButton button1, JRadioButton button2, JRadioButton button3,
			JRadioButton button4) {
		button1.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				button1.setSelected(true);
				button2.setSelected(false);
				button3.setSelected(false);
				button4.setSelected(false);
			}
		});
		button2.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				button2.setSelected(true);
				button1.setSelected(false);
				button3.setSelected(false);
				button4.setSelected(false);
			}
		});
		button3.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				button3.setSelected(true);
				button2.setSelected(false);
				button1.setSelected(false);
				button4.setSelected(false);
			}
		});
		button4.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				button4.setSelected(true);
				button1.setSelected(false);
				button2.setSelected(false);
				button3.setSelected(false);
			}
		});
	}
}
