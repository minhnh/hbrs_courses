package exercise6;

import java.awt.CardLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

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

import exercise6.BlackDotsPicturePanel.BlackDotsFieldName;

public class BlackDotsPanel extends JPanel {

    /**
     * 
     */
    private static final long serialVersionUID = 1L;
    private JPanel chooseColorPanelBlackDots;
    private JPanel itemSizePanelBlackDots;
    private BlackDotsPicturePanel blackDotsPicture;
    private Color color;

    public BlackDotsPanel() {
	initialize();
    }

    private void initialize() {

	blackDotsPicture = new BlackDotsPicturePanel();
	blackDotsPicture.setMinimumSize(new Dimension(300, 300));
	blackDotsPicture.setLayout(new CardLayout(0, 0));

	initializeColorPickerBlackDots();
	initializeItemSizeBlackDots();

	GroupLayout gl_blackDotsPanel = new GroupLayout(this);
	gl_blackDotsPanel
		.setHorizontalGroup(gl_blackDotsPanel.createParallelGroup(Alignment.LEADING)
			.addGroup(gl_blackDotsPanel.createSequentialGroup().addGap(18)
				.addGroup(gl_blackDotsPanel.createParallelGroup(Alignment.TRAILING)
					.addComponent(chooseColorPanelBlackDots,
						GroupLayout.DEFAULT_SIZE, 266,
						Short.MAX_VALUE)
					.addComponent(itemSizePanelBlackDots,
						GroupLayout.DEFAULT_SIZE, 266, Short.MAX_VALUE))
				.addGap(18).addComponent(blackDotsPicture,
					GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
					GroupLayout.PREFERRED_SIZE)
				.addGap(30)));
	gl_blackDotsPanel.setVerticalGroup(gl_blackDotsPanel.createParallelGroup(Alignment.LEADING)
		.addGroup(gl_blackDotsPanel.createSequentialGroup().addContainerGap()
			.addGroup(gl_blackDotsPanel.createParallelGroup(Alignment.LEADING)
				.addComponent(blackDotsPicture, GroupLayout.PREFERRED_SIZE,
					GroupLayout.DEFAULT_SIZE,
					GroupLayout.PREFERRED_SIZE)
				.addGroup(gl_blackDotsPanel.createSequentialGroup()
					.addComponent(chooseColorPanelBlackDots,
						GroupLayout.PREFERRED_SIZE, 113,
						GroupLayout.PREFERRED_SIZE)
					.addGap(18).addComponent(itemSizePanelBlackDots,
						GroupLayout.PREFERRED_SIZE,
						GroupLayout.DEFAULT_SIZE,
						GroupLayout.PREFERRED_SIZE)))
			.addContainerGap(39, Short.MAX_VALUE)));

	this.setLayout(gl_blackDotsPanel);
    }

    private void initializeColorPickerBlackDots() {
	chooseColorPanelBlackDots = new JPanel();
	chooseColorPanelBlackDots.setBorder(BorderFactory.createTitledBorder("Choose Item Color"));
	GridBagLayout gbl_chooseColorPanelBlackDots = new GridBagLayout();
	gbl_chooseColorPanelBlackDots.columnWidths = new int[] { 100, 100, 0 };
	gbl_chooseColorPanelBlackDots.rowHeights = new int[] { 30, 30, 30, 0 };
	gbl_chooseColorPanelBlackDots.columnWeights = new double[] { 0.0, 1.0, Double.MIN_VALUE };
	gbl_chooseColorPanelBlackDots.rowWeights = new double[] { 0.0, 0.0, 0.0, Double.MIN_VALUE };
	chooseColorPanelBlackDots.setLayout(gbl_chooseColorPanelBlackDots);

	/* Declare Elements */

	JPanel colorChosenPanel = new JPanel();
	GridBagConstraints gbc_colorChosenPanel = new GridBagConstraints();
	gbc_colorChosenPanel.insets = new Insets(0, 0, 5, 0);
	gbc_colorChosenPanel.fill = GridBagConstraints.BOTH;
	gbc_colorChosenPanel.gridx = 1;
	gbc_colorChosenPanel.gridy = 0;

	JButton btnPickColor = new JButton("Pick Color");

	JRadioButton rdbtnRectangles = new JRadioButton("Rectangles");
	GridBagConstraints gbc_rdbtnRectangle = new GridBagConstraints();
	gbc_rdbtnRectangle.anchor = GridBagConstraints.WEST;
	gbc_rdbtnRectangle.insets = new Insets(0, 0, 5, 5);
	gbc_rdbtnRectangle.gridx = 0;
	gbc_rdbtnRectangle.gridy = 0;
	rdbtnRectangles.setSelected(true);

	JRadioButton rdbtnBars = new JRadioButton("Bars");
	GridBagConstraints gbc_rdbtnBars = new GridBagConstraints();
	gbc_rdbtnBars.anchor = GridBagConstraints.WEST;
	gbc_rdbtnBars.insets = new Insets(0, 0, 5, 5);
	gbc_rdbtnBars.gridx = 0;
	gbc_rdbtnBars.gridy = 1;
	GridBagConstraints gbc_btnPickColor = new GridBagConstraints();
	gbc_btnPickColor.fill = GridBagConstraints.HORIZONTAL;
	gbc_btnPickColor.insets = new Insets(0, 0, 5, 0);
	gbc_btnPickColor.gridx = 1;
	gbc_btnPickColor.gridy = 1;

	JRadioButton rdbtnDots = new JRadioButton("Dots");
	GridBagConstraints gbc_rdbtnDots = new GridBagConstraints();
	gbc_rdbtnDots.anchor = GridBagConstraints.WEST;
	gbc_rdbtnDots.insets = new Insets(0, 0, 0, 5);
	gbc_rdbtnDots.gridx = 0;
	gbc_rdbtnDots.gridy = 2;

	JButton buttonSetColor = new JButton("Set Color");
	GridBagConstraints gbc_buttonSetColor = new GridBagConstraints();
	gbc_buttonSetColor.fill = GridBagConstraints.HORIZONTAL;
	gbc_buttonSetColor.gridx = 1;
	gbc_buttonSetColor.gridy = 2;

	/* Link elements' actions */
	JColorChooser tcc = new JColorChooser(chooseColorPanelBlackDots.getForeground());
	tcc.setPreviewPanel(new JPanel());
	btnPickColor.addActionListener(new ActionListener() {
	    public void actionPerformed(ActionEvent arg0) {
		color = JColorChooser.showDialog(tcc, "Choose Color", color);
		colorChosenPanel.setBackground(color);
	    }
	});

	buttonSetColor.addActionListener(new ActionListener() {
	    @Override
	    public void actionPerformed(ActionEvent e) {
		BlackDotsFieldName fieldName = BlackDotsFieldName.UNKNOWN;
		if (rdbtnRectangles.isSelected()) {
		    fieldName = BlackDotsFieldName.RECTANGLE_COLOR;
		} else if (rdbtnBars.isSelected()) {
		    fieldName = BlackDotsFieldName.BAR_COLOR;
		} else if (rdbtnDots.isSelected()) {
		    fieldName = BlackDotsFieldName.DOT_COLOR;
		}
		blackDotsPicture.updateColor(fieldName, color);
	    }
	});

	ArrayList<JRadioButton> buttons = new ArrayList<>();
	buttons.add(rdbtnRectangles);
	buttons.add(rdbtnBars);
	buttons.add(rdbtnDots);
	Exercise6.setRadioButtonsActions(buttons);

	/* Add Elements to panel */
	chooseColorPanelBlackDots.add(rdbtnRectangles, gbc_rdbtnRectangle);
	chooseColorPanelBlackDots.add(colorChosenPanel, gbc_colorChosenPanel);
	chooseColorPanelBlackDots.add(rdbtnBars, gbc_rdbtnBars);
	chooseColorPanelBlackDots.add(btnPickColor, gbc_btnPickColor);
	chooseColorPanelBlackDots.add(rdbtnDots, gbc_rdbtnDots);
	chooseColorPanelBlackDots.add(buttonSetColor, gbc_buttonSetColor);
    }

    private void initializeItemSizeBlackDots() {

	itemSizePanelBlackDots = new JPanel();
	itemSizePanelBlackDots.setBorder(BorderFactory.createTitledBorder("Choose Item Size"));

	GridBagLayout gbl_itemSizePanelBlackDots = new GridBagLayout();
	gbl_itemSizePanelBlackDots.columnWidths = new int[] { 100, 100, 0 };
	gbl_itemSizePanelBlackDots.rowHeights = new int[] { 0, 30, 30, 30, 30, 0 };
	gbl_itemSizePanelBlackDots.columnWeights = new double[] { 0.0, 1.0, Double.MIN_VALUE };
	gbl_itemSizePanelBlackDots.rowWeights = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0,
		Double.MIN_VALUE };
	itemSizePanelBlackDots.setLayout(gbl_itemSizePanelBlackDots);

	/* Declare Elements */
	JRadioButton rdbtnRectangleWidth = new JRadioButton("Rect. Width");
	GridBagConstraints gbc_rdbtnRectangleWidth = new GridBagConstraints();
	gbc_rdbtnRectangleWidth.anchor = GridBagConstraints.WEST;
	gbc_rdbtnRectangleWidth.insets = new Insets(0, 0, 5, 5);
	gbc_rdbtnRectangleWidth.gridx = 0;
	gbc_rdbtnRectangleWidth.gridy = 0;
	rdbtnRectangleWidth.setSelected(true);

	JRadioButton rdbtnRectangleHeight = new JRadioButton("Rect. Height");
	GridBagConstraints gbc_rdbtnRectangleHeight = new GridBagConstraints();
	gbc_rdbtnRectangleHeight.anchor = GridBagConstraints.WEST;
	gbc_rdbtnRectangleHeight.insets = new Insets(0, 0, 5, 5);
	gbc_rdbtnRectangleHeight.gridx = 0;
	gbc_rdbtnRectangleHeight.gridy = 1;

	JRadioButton rdbtnBarWidth = new JRadioButton("Bar Width");
	GridBagConstraints gbc_rdbtnBarWidth = new GridBagConstraints();
	gbc_rdbtnBarWidth.anchor = GridBagConstraints.WEST;
	gbc_rdbtnBarWidth.insets = new Insets(0, 0, 5, 5);
	gbc_rdbtnBarWidth.gridx = 0;
	gbc_rdbtnBarWidth.gridy = 2;

	JRadioButton rdbtnBarHeight = new JRadioButton("Bar Height");
	GridBagConstraints gbc_rdbtnBarHeight = new GridBagConstraints();
	gbc_rdbtnBarHeight.anchor = GridBagConstraints.WEST;
	gbc_rdbtnBarHeight.insets = new Insets(0, 0, 5, 5);
	gbc_rdbtnBarHeight.gridx = 0;
	gbc_rdbtnBarHeight.gridy = 3;

	JButton buttonSetSize = new JButton("Set Size");
	GridBagConstraints gbc_buttonSetSize = new GridBagConstraints();
	gbc_buttonSetSize.insets = new Insets(0, 0, 5, 0);
	gbc_buttonSetSize.fill = GridBagConstraints.HORIZONTAL;
	gbc_buttonSetSize.gridx = 1;
	gbc_buttonSetSize.gridy = 2;

	JSlider sliderSize = new JSlider();
	sliderSize.setMajorTickSpacing(20);
	sliderSize.setValue(40);
	sliderSize.setMinimum(10);
	sliderSize.setPaintTicks(true);
	GridBagConstraints gbc_sliderSize = new GridBagConstraints();
	gbc_sliderSize.fill = GridBagConstraints.HORIZONTAL;
	gbc_sliderSize.gridwidth = 2;
	gbc_sliderSize.gridx = 0;
	gbc_sliderSize.gridy = 4;

	JTextField sizeTextField = new JTextField();
	sizeTextField.setHorizontalAlignment(SwingConstants.CENTER);
	sizeTextField.setText(Integer.toString(sliderSize.getValue()));
	GridBagConstraints gbc_sizeTextField = new GridBagConstraints();
	gbc_sizeTextField.fill = GridBagConstraints.HORIZONTAL;
	gbc_sizeTextField.insets = new Insets(0, 0, 5, 0);
	gbc_sizeTextField.gridx = 1;
	gbc_sizeTextField.gridy = 3;
	sizeTextField.setColumns(10);

	/* Link elements' actions */
	buttonSetSize.addActionListener(new ActionListener() {
	    @Override
	    public void actionPerformed(ActionEvent e) {
		BlackDotsFieldName fieldName = BlackDotsFieldName.UNKNOWN;
		if (rdbtnRectangleWidth.isSelected()) {
		    fieldName = BlackDotsFieldName.RECTANGLE_WIDTH;
		} else if (rdbtnRectangleHeight.isSelected()) {
		    fieldName = BlackDotsFieldName.RECTANGLE_HEIGHT;
		} else if (rdbtnBarWidth.isSelected()) {
		    fieldName = BlackDotsFieldName.BAR_WIDTH;
		} else if (rdbtnBarHeight.isSelected()) {
		    fieldName = BlackDotsFieldName.BAR_HEIGHT;
		}
		blackDotsPicture.updateDimension(fieldName, sliderSize.getValue());
	    }
	});
	ArrayList<JRadioButton> buttons = new ArrayList<>();
	buttons.add(rdbtnRectangleWidth);
	buttons.add(rdbtnRectangleHeight);
	buttons.add(rdbtnBarWidth);
	buttons.add(rdbtnBarHeight);
	Exercise6.setRadioButtonsActions(buttons);
	Exercise6.setSliderTextFieldActions(sliderSize, sizeTextField);

	/* Add Elements to Panel */
	itemSizePanelBlackDots.add(rdbtnRectangleWidth, gbc_rdbtnRectangleWidth);
	itemSizePanelBlackDots.add(rdbtnRectangleHeight, gbc_rdbtnRectangleHeight);
	itemSizePanelBlackDots.add(rdbtnBarWidth, gbc_rdbtnBarWidth);
	itemSizePanelBlackDots.add(rdbtnBarHeight, gbc_rdbtnBarHeight);
	itemSizePanelBlackDots.add(buttonSetSize, gbc_buttonSetSize);
	itemSizePanelBlackDots.add(sliderSize, gbc_sliderSize);
	itemSizePanelBlackDots.add(sizeTextField, gbc_sizeTextField);
    }

}
