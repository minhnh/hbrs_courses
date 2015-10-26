package exercise6;

import java.awt.CardLayout;
import java.awt.Color;
import java.awt.Dimension;
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
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.SwingConstants;

public class ParallelLinesPanel extends JPanel {

	private static final long serialVersionUID = 1L;
	private JPanel chooseColorPanelParallelLines;
	private JPanel itemSizePanelParallelLines;
	// private BlackDotsPicturePanel blackDotsPicture;
	private Color color;

	public ParallelLinesPanel() {
		initialize();
	}

	private void initialize() {

		initializeColorPickerParallelLines();
		initializeItemSizeParallelLines();

		JPanel panel_2 = new JPanel();
		panel_2.setMinimumSize(new Dimension(300, 300));
		panel_2.setMaximumSize(new Dimension(300, 300));
		panel_2.setLayout(new CardLayout(0, 0));

		GroupLayout gl_parallelLinesPanel = new GroupLayout(this);
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
		this.setLayout(gl_parallelLinesPanel);
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

		JSlider slider = new JSlider();
		slider.setValue(40);
		slider.setPaintTicks(true);
		slider.setMinimum(10);
		slider.setMajorTickSpacing(20);
		GridBagConstraints gbc_slider = new GridBagConstraints();
		gbc_slider.fill = GridBagConstraints.HORIZONTAL;
		gbc_slider.gridwidth = 2;
		gbc_slider.gridx = 0;
		gbc_slider.gridy = 4;

		JTextField textField_1 = new JTextField();
		textField_1.setText(Integer.toString(slider.getValue()));
		textField_1.setHorizontalAlignment(SwingConstants.CENTER);
		textField_1.setColumns(10);
		GridBagConstraints gbc_textField_1 = new GridBagConstraints();
		gbc_textField_1.fill = GridBagConstraints.HORIZONTAL;
		gbc_textField_1.insets = new Insets(0, 0, 5, 0);
		gbc_textField_1.gridx = 1;
		gbc_textField_1.gridy = 3;

		/* Link elements' actions */

		/* Add Elements to Panel */
		itemSizePanelParallelLines.add(radioButton_3, gbc_radioButton_3);
		itemSizePanelParallelLines.add(radioButton_4, gbc_radioButton_4);
		itemSizePanelParallelLines.add(radioButton_5, gbc_radioButton_5);
		itemSizePanelParallelLines.add(radioButton_6, gbc_radioButton_6);
		itemSizePanelParallelLines.add(button_2, gbc_button_2);
		itemSizePanelParallelLines.add(slider, gbc_slider);
		itemSizePanelParallelLines.add(textField_1, gbc_textField_1);
	}

}
