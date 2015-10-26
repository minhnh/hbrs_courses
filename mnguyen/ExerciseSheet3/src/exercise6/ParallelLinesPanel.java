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

import exercise6.ParallelLinesPicturePanel.ParallelLinesFieldName;

public class ParallelLinesPanel extends JPanel {

	private static final long serialVersionUID = 1L;
	private JPanel chooseColorPanelParallelLines;
	private JPanel itemSizePanelParallelLines;
	private ParallelLinesPicturePanel parallelLinesPicture;
	// private BlackDotsPicturePanel blackDotsPicture;
	private Color color;

	public ParallelLinesPanel() {
		initialize();
	}

	private void initialize() {

		initializeColorPickerParallelLines();
		initializeItemSizeParallelLines();

		parallelLinesPicture = new ParallelLinesPicturePanel();
		parallelLinesPicture.setMinimumSize(new Dimension(300, 300));
		parallelLinesPicture.setLayout(new CardLayout(0, 0));

		GroupLayout gl_parallelLinesPanel = new GroupLayout(this);
		gl_parallelLinesPanel.setHorizontalGroup(gl_parallelLinesPanel.createParallelGroup(Alignment.LEADING)
				.addGroup(gl_parallelLinesPanel.createSequentialGroup().addGap(18)
						.addGroup(gl_parallelLinesPanel.createParallelGroup(Alignment.TRAILING)
								.addComponent(chooseColorPanelParallelLines, GroupLayout.DEFAULT_SIZE, 266,
										Short.MAX_VALUE)
						.addComponent(itemSizePanelParallelLines, GroupLayout.DEFAULT_SIZE, 266, Short.MAX_VALUE))
						.addGap(18).addComponent(parallelLinesPicture, GroupLayout.PREFERRED_SIZE,
								GroupLayout.DEFAULT_SIZE, GroupLayout.PREFERRED_SIZE)
						.addGap(30)));
		gl_parallelLinesPanel.setVerticalGroup(gl_parallelLinesPanel.createParallelGroup(Alignment.LEADING)
				.addGroup(gl_parallelLinesPanel.createSequentialGroup().addContainerGap()
						.addGroup(gl_parallelLinesPanel.createParallelGroup(Alignment.LEADING)
								.addComponent(parallelLinesPicture, GroupLayout.PREFERRED_SIZE,
										GroupLayout.DEFAULT_SIZE, GroupLayout.PREFERRED_SIZE)
						.addGroup(gl_parallelLinesPanel.createSequentialGroup()
								.addComponent(chooseColorPanelParallelLines, GroupLayout.PREFERRED_SIZE, 113,
										GroupLayout.PREFERRED_SIZE)
								.addGap(18).addComponent(itemSizePanelParallelLines, GroupLayout.PREFERRED_SIZE,
										GroupLayout.DEFAULT_SIZE, GroupLayout.PREFERRED_SIZE)))
						.addContainerGap(39, Short.MAX_VALUE)));
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

		JRadioButton rdbtnFirstRectangles = new JRadioButton("First Rectangles");
		GridBagConstraints gbc_rdbtnFirstRectangles = new GridBagConstraints();
		gbc_rdbtnFirstRectangles.anchor = GridBagConstraints.WEST;
		gbc_rdbtnFirstRectangles.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnFirstRectangles.gridx = 0;
		gbc_rdbtnFirstRectangles.gridy = 0;

		JRadioButton rdbtnSecondRectangles = new JRadioButton("Second Rectangles");
		GridBagConstraints gbc_rdbtnSecondRectangles = new GridBagConstraints();
		gbc_rdbtnSecondRectangles.anchor = GridBagConstraints.WEST;
		gbc_rdbtnSecondRectangles.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnSecondRectangles.gridx = 0;
		gbc_rdbtnSecondRectangles.gridy = 1;

		JRadioButton rdbtnLines = new JRadioButton("Lines");
		GridBagConstraints gbc_rdbtnLines = new GridBagConstraints();
		gbc_rdbtnLines.anchor = GridBagConstraints.WEST;
		gbc_rdbtnLines.insets = new Insets(0, 0, 0, 5);
		gbc_rdbtnLines.gridx = 0;
		gbc_rdbtnLines.gridy = 2;

		JButton buttonSetColor = new JButton("Set Color");
		GridBagConstraints gbc_buttonSetColor = new GridBagConstraints();
		gbc_buttonSetColor.fill = GridBagConstraints.HORIZONTAL;
		gbc_buttonSetColor.gridx = 1;
		gbc_buttonSetColor.gridy = 2;

		/* Link elements' actions */
		JColorChooser tcc = new JColorChooser(chooseColorPanelParallelLines.getForeground());
		tcc.setPreviewPanel(new JPanel());
		btnPickColor.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				color = JColorChooser.showDialog(tcc, "Choose Color", color);
				colorChosenPanel.setBackground(color);
			}
		});
		Exercise6.set3RadioButtonActions(rdbtnFirstRectangles, rdbtnSecondRectangles, rdbtnLines);

		buttonSetColor.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				ParallelLinesFieldName fieldName = ParallelLinesFieldName.UNKNOWN;
				if (rdbtnFirstRectangles.isSelected()) {
					fieldName = ParallelLinesFieldName.FIRST_RECTANGLE_COLOR;
				} else if (rdbtnSecondRectangles.isSelected()) {
					fieldName = ParallelLinesFieldName.SECOND_RECTANGLE_COLOR;
				} else if (rdbtnLines.isSelected()) {
					fieldName = ParallelLinesFieldName.LINE_COLOR;
				}
				parallelLinesPicture.updateColor(fieldName, color);
			}
		});

		/* Add elements to panel */
		chooseColorPanelParallelLines.add(colorChosenPanel, gbc_colorChosenPanel);
		chooseColorPanelParallelLines.add(btnPickColor, gbc_btnPickColor);
		chooseColorPanelParallelLines.add(rdbtnFirstRectangles, gbc_rdbtnFirstRectangles);
		chooseColorPanelParallelLines.add(rdbtnSecondRectangles, gbc_rdbtnSecondRectangles);
		chooseColorPanelParallelLines.add(rdbtnLines, gbc_rdbtnLines);
		chooseColorPanelParallelLines.add(buttonSetColor, gbc_buttonSetColor);

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
		JRadioButton radioButtonFirstWidth = new JRadioButton("First Width");
		GridBagConstraints gbc_radioButtonFirstWidth = new GridBagConstraints();
		gbc_radioButtonFirstWidth.anchor = GridBagConstraints.WEST;
		gbc_radioButtonFirstWidth.insets = new Insets(0, 0, 5, 5);
		gbc_radioButtonFirstWidth.gridx = 0;
		gbc_radioButtonFirstWidth.gridy = 0;

		JRadioButton radioButtonSecondWidth = new JRadioButton("Second Width");
		GridBagConstraints gbc_radioButtonSecondWidth = new GridBagConstraints();
		gbc_radioButtonSecondWidth.anchor = GridBagConstraints.WEST;
		gbc_radioButtonSecondWidth.insets = new Insets(0, 0, 5, 5);
		gbc_radioButtonSecondWidth.gridx = 0;
		gbc_radioButtonSecondWidth.gridy = 1;

		JRadioButton radioButtonHeight = new JRadioButton("Height");
		GridBagConstraints gbc_radioButtonHeight = new GridBagConstraints();
		gbc_radioButtonHeight.anchor = GridBagConstraints.WEST;
		gbc_radioButtonHeight.insets = new Insets(0, 0, 5, 5);
		gbc_radioButtonHeight.gridx = 0;
		gbc_radioButtonHeight.gridy = 2;

		JRadioButton radioButtonOffset = new JRadioButton("Offset");
		GridBagConstraints gbc_radioButtonOffset = new GridBagConstraints();
		gbc_radioButtonOffset.anchor = GridBagConstraints.WEST;
		gbc_radioButtonOffset.insets = new Insets(0, 0, 5, 5);
		gbc_radioButtonOffset.gridx = 0;
		gbc_radioButtonOffset.gridy = 3;

		JButton buttonSetSize = new JButton("Set Size");
		GridBagConstraints gbcbuttonSetSize = new GridBagConstraints();
		gbcbuttonSetSize.fill = GridBagConstraints.HORIZONTAL;
		gbcbuttonSetSize.insets = new Insets(0, 0, 5, 0);
		gbcbuttonSetSize.gridx = 1;
		gbcbuttonSetSize.gridy = 2;

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

		JTextField textFieldSize = new JTextField();
		textFieldSize.setText(Integer.toString(slider.getValue()));
		textFieldSize.setHorizontalAlignment(SwingConstants.CENTER);
		textFieldSize.setColumns(10);
		GridBagConstraints gbc_textFieldSize = new GridBagConstraints();
		gbc_textFieldSize.fill = GridBagConstraints.HORIZONTAL;
		gbc_textFieldSize.insets = new Insets(0, 0, 5, 0);
		gbc_textFieldSize.gridx = 1;
		gbc_textFieldSize.gridy = 3;

		/* Link elements' actions */
		buttonSetSize.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				ParallelLinesFieldName fieldName = ParallelLinesFieldName.UNKNOWN;
				if (radioButtonFirstWidth.isSelected()) {
					fieldName = ParallelLinesFieldName.FIRST_RECTANGLE_WIDTH;
				} else if (radioButtonSecondWidth.isSelected()) {
					fieldName = ParallelLinesFieldName.SECOND_RECTANGLE_WIDTH;
				} else if (radioButtonHeight.isSelected()) {
					fieldName = ParallelLinesFieldName.RECTANGLE_HEIGHT;
				} else if (radioButtonOffset.isSelected()) {
					fieldName = ParallelLinesFieldName.OFFSET;
				}
				parallelLinesPicture.updateDimension(fieldName, slider.getValue());
			}
		});
		Exercise6.set4RadioButtonActions(radioButtonFirstWidth, radioButtonSecondWidth, radioButtonHeight,
				radioButtonOffset);
		Exercise6.setSliderTextFieldActions(slider, textFieldSize);

		/* Add Elements to Panel */
		itemSizePanelParallelLines.add(radioButtonFirstWidth, gbc_radioButtonFirstWidth);
		itemSizePanelParallelLines.add(radioButtonSecondWidth, gbc_radioButtonSecondWidth);
		itemSizePanelParallelLines.add(radioButtonHeight, gbc_radioButtonHeight);
		itemSizePanelParallelLines.add(radioButtonOffset, gbc_radioButtonOffset);
		itemSizePanelParallelLines.add(buttonSetSize, gbcbuttonSetSize);
		itemSizePanelParallelLines.add(slider, gbc_slider);
		itemSizePanelParallelLines.add(textFieldSize, gbc_textFieldSize);
	}

}
