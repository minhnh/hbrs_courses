package exercise5;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.EventQueue;
import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.BorderFactory;
import javax.swing.GroupLayout;
import javax.swing.GroupLayout.Alignment;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.LayoutStyle.ComponentPlacement;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

public class Exercise5 {

	private JFrame frame;
	private JSlider redSlider;
	private JTextField redValue;
	private JTextField greenValue;
	private JLabel redLabel;
	private JLabel blueLabel;
	private JSlider blueSlider;
	private JTextField blueValue;
	private JButton btnSetColor;
	private JRadioButton rdbtnRight;
	private JRadioButton rdbtnLeft;
	private final Action setPictureLeftAction = new SetPictureLeftAction();
	private final Action setPictureRightAction = new SetPictureRightAction();
	private JLabel greenLabel;
	private JSlider greenSlider;
	private JPanel colorPanel;
	private JPanel choosePicturePanel;
	private JPanel picturePanel;
	private JPanel pictureLeftPanel;
	private JPanel pictureRightPanel;
	private JButton buttonLoadColor;
	private JPanel normPanel;
	private JLabel lblNormValue;

	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					Exercise5 window = new Exercise5();
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
	public Exercise5() {
		initialize();
	}

	private void getTextFieldInt(JTextField textField, JSlider slider) {
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

	private void setSlidersFromPicture() {
		Color color;
		if (rdbtnLeft.isSelected()) {
			color = pictureLeftPanel.getBackground();
		} else if (rdbtnRight.isSelected()) {
			color = pictureRightPanel.getBackground();
		} else {
			color = new Color(127, 127, 127);
		}
		redSlider.setValue(color.getRed());
		greenSlider.setValue(color.getGreen());
		blueSlider.setValue(color.getBlue());
	}

	private void setPictureFromSliders() {
		Color colors = new Color(redSlider.getValue(), greenSlider.getValue(), blueSlider.getValue());
		if (rdbtnLeft.isSelected()) {
			pictureLeftPanel.setBackground(colors);
		} else if (rdbtnRight.isSelected()) {
			pictureRightPanel.setBackground(colors);
		}
	}

	private void setNormValue() {
		double normL3Left = 0.0d;
		double normL3Right = 0.0d;
		Color colorLeft = pictureLeftPanel.getBackground();
		Color colorRight = pictureRightPanel.getBackground();

		normL3Left = Math.pow(
				Math.pow(colorLeft.getRed(), 3) + Math.pow(colorLeft.getGreen(), 3) + Math.pow(colorLeft.getBlue(), 3),
				(double) 1 / 3);

		normL3Right = Math.pow(Math.pow(colorRight.getRed(), 3) + Math.pow(colorRight.getGreen(), 3)
				+ Math.pow(colorRight.getBlue(), 3), (double) 1 / 3);

		String resultString = String.format("%5.5f", Math.abs(normL3Left - normL3Right));
		lblNormValue.setText(resultString);
	}

	/**
	 * Initialize the contents of the frame.
	 */
	private void initialize() {
		frame = new JFrame("AST WS 2015 - Exercise 5 (by Minh Nguyen)");
		frame.setBounds(100, 100, 625, 500);
		frame.setResizable(false);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		colorPanel = new JPanel();
		colorPanel.setForeground(Color.BLACK);
		colorPanel.setBorder(BorderFactory.createTitledBorder("Pick Color"));

		choosePicturePanel = new JPanel();
		choosePicturePanel.setBorder(BorderFactory.createTitledBorder("Choose Picture"));

		picturePanel = new JPanel();
		picturePanel.setMaximumSize(new Dimension(300, 300));

		normPanel = new JPanel();
		normPanel.setBorder(
				new TitledBorder(null, "Norm Difference", TitledBorder.LEADING, TitledBorder.TOP, null, null));
		GroupLayout groupLayout = new GroupLayout(frame.getContentPane());
		groupLayout
				.setHorizontalGroup(groupLayout.createParallelGroup(Alignment.LEADING)
						.addGroup(groupLayout.createSequentialGroup().addContainerGap()
								.addGroup(groupLayout.createParallelGroup(Alignment.LEADING)
										.addComponent(picturePanel, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE,
												Short.MAX_VALUE)
								.addGroup(groupLayout.createSequentialGroup()
										.addComponent(colorPanel, GroupLayout.PREFERRED_SIZE, 322,
												GroupLayout.PREFERRED_SIZE)
										.addGap(18)
										.addGroup(groupLayout.createParallelGroup(Alignment.LEADING, false)
												.addComponent(normPanel, GroupLayout.DEFAULT_SIZE,
														GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
												.addComponent(choosePicturePanel, GroupLayout.DEFAULT_SIZE,
														GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
								.addPreferredGap(ComponentPlacement.RELATED, 48, Short.MAX_VALUE))).addContainerGap()));
		groupLayout.setVerticalGroup(groupLayout.createParallelGroup(Alignment.LEADING)
				.addGroup(groupLayout.createSequentialGroup().addContainerGap()
						.addGroup(groupLayout.createParallelGroup(Alignment.BASELINE)
								.addComponent(colorPanel, GroupLayout.PREFERRED_SIZE, 136, GroupLayout.PREFERRED_SIZE)
								.addGroup(groupLayout.createSequentialGroup()
										.addComponent(choosePicturePanel, GroupLayout.PREFERRED_SIZE,
												GroupLayout.DEFAULT_SIZE, GroupLayout.PREFERRED_SIZE)
										.addPreferredGap(ComponentPlacement.RELATED, 8, Short.MAX_VALUE).addComponent(
												normPanel, GroupLayout.PREFERRED_SIZE, 52, GroupLayout.PREFERRED_SIZE)))
				.addPreferredGap(ComponentPlacement.RELATED).addComponent(picturePanel, GroupLayout.PREFERRED_SIZE,
						GroupLayout.DEFAULT_SIZE, GroupLayout.PREFERRED_SIZE).addContainerGap(18, Short.MAX_VALUE)));
		GridBagLayout gbl_normPanel = new GridBagLayout();
		gbl_normPanel.columnWidths = new int[] { 100, 100, 0 };
		gbl_normPanel.rowHeights = new int[] { 0, 0 };
		gbl_normPanel.columnWeights = new double[] { 0.0, 0.0, Double.MIN_VALUE };
		gbl_normPanel.rowWeights = new double[] { 0.0, Double.MIN_VALUE };
		normPanel.setLayout(gbl_normPanel);

		lblNormValue = new JLabel("N/A");
		GridBagConstraints gbc_lblNormValue = new GridBagConstraints();
		gbc_lblNormValue.fill = GridBagConstraints.VERTICAL;
		gbc_lblNormValue.insets = new Insets(0, 0, 0, 5);
		gbc_lblNormValue.gridx = 0;
		gbc_lblNormValue.gridy = 0;
		normPanel.add(lblNormValue, gbc_lblNormValue);

		JButton buttonCompute = new JButton("Compute");
		GridBagConstraints gbc_btnCompute = new GridBagConstraints();
		gbc_btnCompute.fill = GridBagConstraints.BOTH;
		gbc_btnCompute.gridx = 1;
		gbc_btnCompute.gridy = 0;
		normPanel.add(buttonCompute, gbc_btnCompute);
		GridBagLayout gbl_leftPicturePanel = new GridBagLayout();
		gbl_leftPicturePanel.columnWidths = new int[] { 300, 300, 0 };
		gbl_leftPicturePanel.rowHeights = new int[] { 300, 0 };
		gbl_leftPicturePanel.columnWeights = new double[] { 1.0, 1.0, Double.MIN_VALUE };
		gbl_leftPicturePanel.rowWeights = new double[] { 1.0, Double.MIN_VALUE };
		picturePanel.setLayout(gbl_leftPicturePanel);
		buttonCompute.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				setNormValue();
			}
		});

		pictureLeftPanel = new JPanel();
		pictureLeftPanel.setBorder(null);
		FlowLayout flowLayout = (FlowLayout) pictureLeftPanel.getLayout();
		flowLayout.setVgap(0);
		flowLayout.setHgap(0);
		pictureLeftPanel.setBackground(Color.RED);
		GridBagConstraints gbc_pictureLeftPanel = new GridBagConstraints();
		gbc_pictureLeftPanel.fill = GridBagConstraints.BOTH;
		gbc_pictureLeftPanel.gridx = 0;
		gbc_pictureLeftPanel.gridy = 0;
		picturePanel.add(pictureLeftPanel, gbc_pictureLeftPanel);

		pictureRightPanel = new JPanel();
		pictureRightPanel.setBorder(null);
		FlowLayout flowLayout_1 = (FlowLayout) pictureRightPanel.getLayout();
		flowLayout_1.setVgap(0);
		flowLayout_1.setHgap(0);
		pictureRightPanel.setBackground(Color.GREEN);
		GridBagConstraints gbc_pictureRightPanel = new GridBagConstraints();
		gbc_pictureRightPanel.fill = GridBagConstraints.BOTH;
		gbc_pictureRightPanel.gridx = 1;
		gbc_pictureRightPanel.gridy = 0;
		picturePanel.add(pictureRightPanel, gbc_pictureRightPanel);
		GridBagLayout gbl_choosePicturePanel = new GridBagLayout();
		gbl_choosePicturePanel.columnWidths = new int[] { 100, 100, 0 };
		gbl_choosePicturePanel.rowHeights = new int[] { 25, 25, 0 };
		gbl_choosePicturePanel.columnWeights = new double[] { 0.0, 0.0, Double.MIN_VALUE };
		gbl_choosePicturePanel.rowWeights = new double[] { 0.0, 0.0, Double.MIN_VALUE };
		choosePicturePanel.setLayout(gbl_choosePicturePanel);

		rdbtnLeft = new JRadioButton("Left");
		GridBagConstraints gbc_rdbtnLeft = new GridBagConstraints();
		gbc_rdbtnLeft.anchor = GridBagConstraints.WEST;
		gbc_rdbtnLeft.insets = new Insets(0, 0, 5, 5);
		gbc_rdbtnLeft.gridx = 0;
		gbc_rdbtnLeft.gridy = 0;
		choosePicturePanel.add(rdbtnLeft, gbc_rdbtnLeft);
		rdbtnLeft.setAction(setPictureLeftAction);
		rdbtnLeft.setSelected(true);

		btnSetColor = new JButton("Set Color");
		GridBagConstraints gbc_btnSetColor = new GridBagConstraints();
		gbc_btnSetColor.fill = GridBagConstraints.HORIZONTAL;
		gbc_btnSetColor.insets = new Insets(0, 0, 5, 0);
		gbc_btnSetColor.gridx = 1;
		gbc_btnSetColor.gridy = 0;
		choosePicturePanel.add(btnSetColor, gbc_btnSetColor);
		btnSetColor.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				setPictureFromSliders();
			}
		});

		rdbtnRight = new JRadioButton("Right");
		GridBagConstraints gbc_rdbtnRight = new GridBagConstraints();
		gbc_rdbtnRight.anchor = GridBagConstraints.WEST;
		gbc_rdbtnRight.insets = new Insets(0, 0, 0, 5);
		gbc_rdbtnRight.gridx = 0;
		gbc_rdbtnRight.gridy = 1;
		choosePicturePanel.add(rdbtnRight, gbc_rdbtnRight);
		rdbtnRight.setAction(setPictureRightAction);

		buttonLoadColor = new JButton("Load Color");
		GridBagConstraints gbc_btnLoadColor = new GridBagConstraints();
		gbc_btnLoadColor.fill = GridBagConstraints.HORIZONTAL;
		gbc_btnLoadColor.gridx = 1;
		gbc_btnLoadColor.gridy = 1;
		choosePicturePanel.add(buttonLoadColor, gbc_btnLoadColor);
		GridBagLayout gbl_colorPanel = new GridBagLayout();
		gbl_colorPanel.columnWidths = new int[] { 50, 200, 50, 0 };
		gbl_colorPanel.rowHeights = new int[] { 36, 0, 0, 0 };
		gbl_colorPanel.columnWeights = new double[] { 0.0, 0.0, 0.0, Double.MIN_VALUE };
		gbl_colorPanel.rowWeights = new double[] { 0.0, 0.0, 0.0, Double.MIN_VALUE };
		colorPanel.setLayout(gbl_colorPanel);
		buttonLoadColor.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				setSlidersFromPicture();
			}
		});

		redLabel = new JLabel("Red");
		GridBagConstraints gbc_redLabel = new GridBagConstraints();
		gbc_redLabel.anchor = GridBagConstraints.EAST;
		gbc_redLabel.insets = new Insets(0, 0, 5, 5);
		gbc_redLabel.gridx = 0;
		gbc_redLabel.gridy = 0;
		colorPanel.add(redLabel, gbc_redLabel);

		redSlider = new JSlider(0, 255, 127);
		redSlider.addChangeListener(new ChangeListener() {
			public void stateChanged(ChangeEvent arg0) {
				redValue.setText(Integer.toString(redSlider.getValue()));
				setPictureFromSliders();
			}
		});
		redSlider.setMajorTickSpacing(32);
		redSlider.setPaintTicks(true);
		redSlider.setMinorTickSpacing(8);
		GridBagConstraints gbc_redSlider = new GridBagConstraints();
		gbc_redSlider.fill = GridBagConstraints.HORIZONTAL;
		gbc_redSlider.insets = new Insets(0, 0, 5, 5);
		gbc_redSlider.gridx = 1;
		gbc_redSlider.gridy = 0;
		colorPanel.add(redSlider, gbc_redSlider);

		redValue = new JTextField();
		redValue.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				getTextFieldInt(redValue, redSlider);
			}
		});
		GridBagConstraints gbc_redValue = new GridBagConstraints();
		gbc_redValue.fill = GridBagConstraints.HORIZONTAL;
		gbc_redValue.insets = new Insets(0, 0, 5, 0);
		gbc_redValue.gridx = 2;
		gbc_redValue.gridy = 0;
		colorPanel.add(redValue, gbc_redValue);
		redValue.setColumns(10);
		redValue.setText(Integer.toString(redSlider.getValue()));

		greenLabel = new JLabel("Green");
		GridBagConstraints gbc_label_2 = new GridBagConstraints();
		gbc_label_2.anchor = GridBagConstraints.EAST;
		gbc_label_2.insets = new Insets(0, 0, 5, 5);
		gbc_label_2.gridx = 0;
		gbc_label_2.gridy = 1;
		colorPanel.add(greenLabel, gbc_label_2);

		greenSlider = new JSlider(0, 255, 127);
		greenSlider.addChangeListener(new ChangeListener() {
			public void stateChanged(ChangeEvent arg0) {
				greenValue.setText(Integer.toString(greenSlider.getValue()));
				setPictureFromSliders();
			}
		});
		greenSlider.setMajorTickSpacing(32);
		greenSlider.setMinorTickSpacing(8);
		greenSlider.setPaintTicks(true);
		GridBagConstraints gbc_slider_2 = new GridBagConstraints();
		gbc_slider_2.fill = GridBagConstraints.HORIZONTAL;
		gbc_slider_2.insets = new Insets(0, 0, 5, 5);
		gbc_slider_2.gridx = 1;
		gbc_slider_2.gridy = 1;
		colorPanel.add(greenSlider, gbc_slider_2);

		greenValue = new JTextField();
		greenValue.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				getTextFieldInt(greenValue, greenSlider);
			}
		});
		GridBagConstraints gbc_greenValue = new GridBagConstraints();
		gbc_greenValue.insets = new Insets(0, 0, 5, 0);
		gbc_greenValue.fill = GridBagConstraints.HORIZONTAL;
		gbc_greenValue.gridx = 2;
		gbc_greenValue.gridy = 1;
		colorPanel.add(greenValue, gbc_greenValue);
		greenValue.setColumns(10);
		greenValue.setText(Integer.toString(greenSlider.getValue()));

		blueLabel = new JLabel("Blue");
		GridBagConstraints gbc_blueLabel = new GridBagConstraints();
		gbc_blueLabel.anchor = GridBagConstraints.EAST;
		gbc_blueLabel.insets = new Insets(0, 0, 0, 5);
		gbc_blueLabel.gridx = 0;
		gbc_blueLabel.gridy = 2;
		colorPanel.add(blueLabel, gbc_blueLabel);

		blueSlider = new JSlider(0, 255, 127);
		blueSlider.addChangeListener(new ChangeListener() {
			public void stateChanged(ChangeEvent arg0) {
				blueValue.setText(Integer.toString(blueSlider.getValue()));
				setPictureFromSliders();
			}
		});
		blueSlider.setMinorTickSpacing(8);
		blueSlider.setMajorTickSpacing(32);
		blueSlider.setPaintTicks(true);
		GridBagConstraints gbc_blueSlider = new GridBagConstraints();
		gbc_blueSlider.fill = GridBagConstraints.HORIZONTAL;
		gbc_blueSlider.insets = new Insets(0, 0, 0, 5);
		gbc_blueSlider.gridx = 1;
		gbc_blueSlider.gridy = 2;
		colorPanel.add(blueSlider, gbc_blueSlider);

		blueValue = new JTextField();
		blueValue.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				getTextFieldInt(blueValue, blueSlider);
			}
		});
		GridBagConstraints gbc_blueColor = new GridBagConstraints();
		gbc_blueColor.fill = GridBagConstraints.HORIZONTAL;
		gbc_blueColor.gridx = 2;
		gbc_blueColor.gridy = 2;
		colorPanel.add(blueValue, gbc_blueColor);
		blueValue.setColumns(10);
		blueValue.setText(Integer.toString(blueSlider.getValue()));
		frame.getContentPane().setLayout(groupLayout);
	}

	private class SetPictureLeftAction extends AbstractAction {
		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		public SetPictureLeftAction() {
			putValue(NAME, "Left");
			putValue(SHORT_DESCRIPTION, "Choose left picture to set color");
		}

		public void actionPerformed(ActionEvent e) {
			rdbtnLeft.setSelected(true);
			rdbtnRight.setSelected(false);
		}
	}

	private class SetPictureRightAction extends AbstractAction {
		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		public SetPictureRightAction() {
			putValue(NAME, "Right");
			putValue(SHORT_DESCRIPTION, "Choose right picture to set color");
		}

		public void actionPerformed(ActionEvent e) {
			rdbtnRight.setSelected(true);
			rdbtnLeft.setSelected(false);
		}
	}
}
