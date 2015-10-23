package exercise5;

import java.awt.EventQueue;

import javax.swing.JFrame;
import javax.swing.JPanel;
import java.awt.BorderLayout;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import java.awt.Insets;
import javax.swing.JLabel;
import java.awt.GridLayout;
import javax.swing.JTextField;
import javax.swing.JButton;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;

public class Exercise5 {

	private JFrame frame;
	private JSlider redSlider;
	private JTextField redValue;
	private JSlider greenSlider;
	private JTextField greenValue;
	private JLabel greenLabel;
	private JLabel redLabel;
	private JLabel blueLabel;
	private JSlider blueSlider;
	private JTextField blueColor;
	private JPanel panel;
	private JButton btnSelect;

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
		frame.setBounds(100, 100, 590, 348);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		GridBagLayout gridBagLayout = new GridBagLayout();
		gridBagLayout.columnWidths = new int[]{100, 287, 0, 0, 0};
		gridBagLayout.rowHeights = new int[]{2, 0, 0, 0};
		gridBagLayout.columnWeights = new double[]{1.0, 0.0, 1.0, 0.0, Double.MIN_VALUE};
		gridBagLayout.rowWeights = new double[]{0.0, 0.0, 0.0, Double.MIN_VALUE};
		frame.getContentPane().setLayout(gridBagLayout);
		
		redLabel = new JLabel("Red");
		GridBagConstraints gbc_redLabel = new GridBagConstraints();
		gbc_redLabel.insets = new Insets(0, 0, 5, 5);
		gbc_redLabel.gridx = 0;
		gbc_redLabel.gridy = 0;
		frame.getContentPane().add(redLabel, gbc_redLabel);
		
		panel = new JPanel();
		GridBagConstraints gbc_panel = new GridBagConstraints();
		gbc_panel.fill = GridBagConstraints.BOTH;
		gbc_panel.insets = new Insets(0, 0, 5, 5);
		gbc_panel.gridx = 1;
		gbc_panel.gridy = 0;
		frame.getContentPane().add(panel, gbc_panel);
		GridBagLayout gbl_panel = new GridBagLayout();
		gbl_panel.columnWidths = new int[]{287, 0};
		gbl_panel.rowHeights = new int[]{2, 0};
		gbl_panel.columnWeights = new double[]{0.0, Double.MIN_VALUE};
		gbl_panel.rowWeights = new double[]{0.0, Double.MIN_VALUE};
		panel.setLayout(gbl_panel);
		
		redSlider = new JSlider();
		GridBagConstraints gbc_redSlider = new GridBagConstraints();
		gbc_redSlider.gridx = 0;
		gbc_redSlider.gridy = 0;
		panel.add(redSlider, gbc_redSlider);
		
		redValue = new JTextField();
		GridBagConstraints gbc_redValue = new GridBagConstraints();
		gbc_redValue.insets = new Insets(0, 0, 5, 5);
		gbc_redValue.fill = GridBagConstraints.HORIZONTAL;
		gbc_redValue.gridx = 2;
		gbc_redValue.gridy = 0;
		frame.getContentPane().add(redValue, gbc_redValue);
		redValue.setColumns(10);
		
		greenLabel = new JLabel("Green");
		GridBagConstraints gbc_greenLabel = new GridBagConstraints();
		gbc_greenLabel.insets = new Insets(0, 0, 5, 5);
		gbc_greenLabel.gridx = 0;
		gbc_greenLabel.gridy = 1;
		frame.getContentPane().add(greenLabel, gbc_greenLabel);
		
		greenSlider = new JSlider();
		GridBagConstraints gbc_greenSlider = new GridBagConstraints();
		gbc_greenSlider.insets = new Insets(0, 0, 5, 5);
		gbc_greenSlider.gridx = 1;
		gbc_greenSlider.gridy = 1;
		frame.getContentPane().add(greenSlider, gbc_greenSlider);
		
		greenValue = new JTextField();
		greenValue.setColumns(10);
		GridBagConstraints gbc_greenValue = new GridBagConstraints();
		gbc_greenValue.insets = new Insets(0, 0, 5, 5);
		gbc_greenValue.fill = GridBagConstraints.HORIZONTAL;
		gbc_greenValue.gridx = 2;
		gbc_greenValue.gridy = 1;
		frame.getContentPane().add(greenValue, gbc_greenValue);
		
		blueLabel = new JLabel("Blue");
		GridBagConstraints gbc_blueLabel = new GridBagConstraints();
		gbc_blueLabel.insets = new Insets(0, 0, 0, 5);
		gbc_blueLabel.gridx = 0;
		gbc_blueLabel.gridy = 2;
		frame.getContentPane().add(blueLabel, gbc_blueLabel);
		
		blueSlider = new JSlider();
		GridBagConstraints gbc_blueSlider = new GridBagConstraints();
		gbc_blueSlider.insets = new Insets(0, 0, 0, 5);
		gbc_blueSlider.gridx = 1;
		gbc_blueSlider.gridy = 2;
		frame.getContentPane().add(blueSlider, gbc_blueSlider);
		
		blueColor = new JTextField();
		blueColor.setColumns(10);
		GridBagConstraints gbc_blueColor = new GridBagConstraints();
		gbc_blueColor.insets = new Insets(0, 0, 0, 5);
		gbc_blueColor.fill = GridBagConstraints.HORIZONTAL;
		gbc_blueColor.gridx = 2;
		gbc_blueColor.gridy = 2;
		frame.getContentPane().add(blueColor, gbc_blueColor);
		
		btnSelect = new JButton("Set Color");
		btnSelect.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
			}
		});
		GridBagConstraints gbc_btnSelect = new GridBagConstraints();
		gbc_btnSelect.gridx = 3;
		gbc_btnSelect.gridy = 2;
		frame.getContentPane().add(btnSelect, gbc_btnSelect);
	}

}
