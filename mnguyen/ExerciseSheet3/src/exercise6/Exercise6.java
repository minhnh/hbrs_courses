package exercise6;

import java.awt.EventQueue;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.GroupLayout;
import javax.swing.GroupLayout.Alignment;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JRadioButton;
import javax.swing.JSlider;
import javax.swing.JTabbedPane;
import javax.swing.JTextField;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

public class Exercise6 {
	private JFrame frame;

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
		ParallelLinesPanel parallelLinesPanel = new ParallelLinesPanel();
		tabbedPane.addTab("ParallelLines", null, parallelLinesPanel, null);
		frame.getContentPane().setLayout(groupLayout);
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
