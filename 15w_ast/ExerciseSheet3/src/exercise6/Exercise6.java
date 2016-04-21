package exercise6;

import java.awt.EventQueue;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

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
	groupLayout.setHorizontalGroup(
		groupLayout.createParallelGroup(Alignment.LEADING).addComponent(tabbedPane,
			GroupLayout.PREFERRED_SIZE, 637, Short.MAX_VALUE));
	groupLayout.setVerticalGroup(
		groupLayout.createParallelGroup(Alignment.LEADING).addComponent(tabbedPane,
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

    public static void setRadioButtonsActions(ArrayList<JRadioButton> buttons) {
	for (int i = 0; i < buttons.size(); i++) {
	    // required to bypass "Local variable statement defined in an
	    // enclosing scope must be final"
	    int k = i;
	    buttons.get(i).addActionListener(new ActionListener() {
		@Override
		public void actionPerformed(ActionEvent e) {
		    buttons.get(k).setSelected(true);
		    for (int j = 0; j < buttons.size(); j++) {
			if (j != k)
			    buttons.get(j).setSelected(false);
		    }
		}
	    });
	}
    }

}
