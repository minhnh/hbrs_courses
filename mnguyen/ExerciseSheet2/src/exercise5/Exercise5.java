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
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.border.Border;

public class Exercise5 {

	private JFrame frame;
	private JSlider redSlider;
	private JTextField redValue;
	private JTextField greenValue;
	private JLabel redLabel;
	private JLabel blueLabel;
	private JSlider blueSlider;
	private JTextField blueColor;
	private JButton btnSetColor;
	private JRadioButton rdbtnRight;
	private JRadioButton rdbtnLeft;
	private final Action setColorAction = new SetColorAction();
	private JLabel greenLabel;
	private JSlider greenSlider;
	private JPanel colorPanel;
	private JPanel choosePicturePanel;
	private JPanel picturePanel;
	private JPanel pictureLeftPanel;
	private JPanel pictureRightPanel;

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

	/**
	 * Initialize the contents of the frame.
	 */
	private void initialize() {
		frame = new JFrame();
		frame.setBounds(100, 100, 625, 510);
		frame.setResizable(false);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		colorPanel = new JPanel();
		colorPanel.setForeground(Color.BLACK);
		colorPanel.setBorder(BorderFactory.createTitledBorder("Pick Color"));

		choosePicturePanel = new JPanel();
		choosePicturePanel.setBorder(BorderFactory.createTitledBorder("Choose Picture"));

		picturePanel = new JPanel();
		picturePanel.setMaximumSize(new Dimension(300, 300));
		GroupLayout groupLayout = new GroupLayout(frame.getContentPane());
		groupLayout
				.setHorizontalGroup(groupLayout.createParallelGroup(Alignment.LEADING)
						.addGroup(groupLayout.createSequentialGroup().addContainerGap()
								.addGroup(groupLayout.createParallelGroup(Alignment.LEADING)
										.addComponent(picturePanel, GroupLayout.PREFERRED_SIZE,
												GroupLayout.DEFAULT_SIZE, GroupLayout.PREFERRED_SIZE)
								.addGroup(groupLayout.createSequentialGroup()
										.addComponent(colorPanel, GroupLayout.PREFERRED_SIZE, 322,
												GroupLayout.PREFERRED_SIZE)
										.addGap(33).addComponent(choosePicturePanel, GroupLayout.PREFERRED_SIZE, 114,
												GroupLayout.PREFERRED_SIZE)))
				.addContainerGap(GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)));
		groupLayout.setVerticalGroup(groupLayout.createParallelGroup(Alignment.LEADING)
				.addGroup(groupLayout.createSequentialGroup().addContainerGap()
						.addGroup(groupLayout.createParallelGroup(Alignment.TRAILING, false)
								.addComponent(choosePicturePanel, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE,
										Short.MAX_VALUE)
						.addComponent(colorPanel, GroupLayout.DEFAULT_SIZE, 136, Short.MAX_VALUE)).addGap(18)
				.addComponent(picturePanel, GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
						GroupLayout.PREFERRED_SIZE).addContainerGap(14, Short.MAX_VALUE)));
		GridBagLayout gbl_leftPicturePanel = new GridBagLayout();
		gbl_leftPicturePanel.columnWidths = new int[] { 300, 300, 0 };
		gbl_leftPicturePanel.rowHeights = new int[] { 300, 0 };
		gbl_leftPicturePanel.columnWeights = new double[] { 1.0, 1.0, Double.MIN_VALUE };
		gbl_leftPicturePanel.rowWeights = new double[] { 1.0, Double.MIN_VALUE };
		picturePanel.setLayout(gbl_leftPicturePanel);

		pictureLeftPanel = new JPanel();
		FlowLayout flowLayout = (FlowLayout) pictureLeftPanel.getLayout();
		flowLayout.setVgap(0);
		flowLayout.setHgap(0);
		pictureLeftPanel.setBackground(Color.RED);
		GridBagConstraints gbc_pictureLeftPanel = new GridBagConstraints();
		gbc_pictureLeftPanel.insets = new Insets(0, 0, 0, 5);
		gbc_pictureLeftPanel.fill = GridBagConstraints.BOTH;
		gbc_pictureLeftPanel.gridx = 0;
		gbc_pictureLeftPanel.gridy = 0;
		picturePanel.add(pictureLeftPanel, gbc_pictureLeftPanel);

		pictureRightPanel = new JPanel();
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
		gbl_choosePicturePanel.columnWidths = new int[] { 0, 100, 0 };
		gbl_choosePicturePanel.rowHeights = new int[] { 20, 0, 0, 0, 0 };
		gbl_choosePicturePanel.columnWeights = new double[] { 0.0, 0.0, Double.MIN_VALUE };
		gbl_choosePicturePanel.rowWeights = new double[] { 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE };
		choosePicturePanel.setLayout(gbl_choosePicturePanel);

		rdbtnLeft = new JRadioButton("Left");
		GridBagConstraints gbc_rdbtnLeft = new GridBagConstraints();
		gbc_rdbtnLeft.anchor = GridBagConstraints.WEST;
		gbc_rdbtnLeft.insets = new Insets(0, 0, 5, 0);
		gbc_rdbtnLeft.gridx = 1;
		gbc_rdbtnLeft.gridy = 1;
		choosePicturePanel.add(rdbtnLeft, gbc_rdbtnLeft);
		rdbtnLeft.setAction(setColorAction);

		rdbtnRight = new JRadioButton("Right");
		GridBagConstraints gbc_rdbtnRight = new GridBagConstraints();
		gbc_rdbtnRight.anchor = GridBagConstraints.WEST;
		gbc_rdbtnRight.insets = new Insets(0, 0, 5, 0);
		gbc_rdbtnRight.gridx = 1;
		gbc_rdbtnRight.gridy = 2;
		choosePicturePanel.add(rdbtnRight, gbc_rdbtnRight);

		btnSetColor = new JButton("Set Color");
		GridBagConstraints gbc_btnSetColor = new GridBagConstraints();
		gbc_btnSetColor.anchor = GridBagConstraints.WEST;
		gbc_btnSetColor.gridx = 1;
		gbc_btnSetColor.gridy = 3;
		choosePicturePanel.add(btnSetColor, gbc_btnSetColor);
		btnSetColor.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
			}
		});
		GridBagLayout gbl_colorPanel = new GridBagLayout();
		gbl_colorPanel.columnWidths = new int[] { 50, 200, 50, 0 };
		gbl_colorPanel.rowHeights = new int[] { 36, 0, 0, 0 };
		gbl_colorPanel.columnWeights = new double[] { 0.0, 0.0, 0.0, Double.MIN_VALUE };
		gbl_colorPanel.rowWeights = new double[] { 0.0, 0.0, 0.0, Double.MIN_VALUE };
		colorPanel.setLayout(gbl_colorPanel);

		redLabel = new JLabel("Red");
		GridBagConstraints gbc_redLabel = new GridBagConstraints();
		gbc_redLabel.anchor = GridBagConstraints.EAST;
		gbc_redLabel.insets = new Insets(0, 0, 5, 5);
		gbc_redLabel.gridx = 0;
		gbc_redLabel.gridy = 0;
		colorPanel.add(redLabel, gbc_redLabel);

		redSlider = new JSlider();
		GridBagConstraints gbc_redSlider = new GridBagConstraints();
		gbc_redSlider.fill = GridBagConstraints.HORIZONTAL;
		gbc_redSlider.insets = new Insets(0, 0, 5, 5);
		gbc_redSlider.gridx = 1;
		gbc_redSlider.gridy = 0;
		colorPanel.add(redSlider, gbc_redSlider);

		redValue = new JTextField();
		GridBagConstraints gbc_redValue = new GridBagConstraints();
		gbc_redValue.fill = GridBagConstraints.HORIZONTAL;
		gbc_redValue.insets = new Insets(0, 0, 5, 0);
		gbc_redValue.gridx = 2;
		gbc_redValue.gridy = 0;
		colorPanel.add(redValue, gbc_redValue);
		redValue.setColumns(10);

		blueLabel = new JLabel("Blue");
		GridBagConstraints gbc_blueLabel = new GridBagConstraints();
		gbc_blueLabel.anchor = GridBagConstraints.EAST;
		gbc_blueLabel.insets = new Insets(0, 0, 5, 5);
		gbc_blueLabel.gridx = 0;
		gbc_blueLabel.gridy = 1;
		colorPanel.add(blueLabel, gbc_blueLabel);

		blueSlider = new JSlider();
		GridBagConstraints gbc_blueSlider = new GridBagConstraints();
		gbc_blueSlider.fill = GridBagConstraints.HORIZONTAL;
		gbc_blueSlider.insets = new Insets(0, 0, 5, 5);
		gbc_blueSlider.gridx = 1;
		gbc_blueSlider.gridy = 1;
		colorPanel.add(blueSlider, gbc_blueSlider);

		blueColor = new JTextField();
		GridBagConstraints gbc_blueColor = new GridBagConstraints();
		gbc_blueColor.fill = GridBagConstraints.HORIZONTAL;
		gbc_blueColor.insets = new Insets(0, 0, 5, 0);
		gbc_blueColor.gridx = 2;
		gbc_blueColor.gridy = 1;
		colorPanel.add(blueColor, gbc_blueColor);
		blueColor.setColumns(10);

		greenLabel = new JLabel("Green");
		GridBagConstraints gbc_label_2 = new GridBagConstraints();
		gbc_label_2.anchor = GridBagConstraints.EAST;
		gbc_label_2.insets = new Insets(0, 0, 0, 5);
		gbc_label_2.gridx = 0;
		gbc_label_2.gridy = 2;
		colorPanel.add(greenLabel, gbc_label_2);

		greenSlider = new JSlider();
		GridBagConstraints gbc_slider_2 = new GridBagConstraints();
		gbc_slider_2.fill = GridBagConstraints.HORIZONTAL;
		gbc_slider_2.insets = new Insets(0, 0, 0, 5);
		gbc_slider_2.gridx = 1;
		gbc_slider_2.gridy = 2;
		colorPanel.add(greenSlider, gbc_slider_2);

		greenValue = new JTextField();
		GridBagConstraints gbc_greenValue = new GridBagConstraints();
		gbc_greenValue.fill = GridBagConstraints.HORIZONTAL;
		gbc_greenValue.gridx = 2;
		gbc_greenValue.gridy = 2;
		colorPanel.add(greenValue, gbc_greenValue);
		greenValue.setColumns(10);
		frame.getContentPane().setLayout(groupLayout);
	}

	private class SetColorAction extends AbstractAction {
		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		public SetColorAction() {
			putValue(NAME, "Left");
			putValue(SHORT_DESCRIPTION, "Some short description");
		}

		public void actionPerformed(ActionEvent e) {
		}
	}

	public Border getColorPanelBorder() {
		return colorPanel.getBorder();
	}

	public void setColorPanelBorder(Border border) {
		colorPanel.setBorder(border);
	}
}
