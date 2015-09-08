/**
 * Exercise 1 for Foundation Course WS2015
 *
 * @author Santosh Thoduka
 *
 */
public class Exercise1 {

	/**
	 * The number of time steps for which a position is to be calculated.
	 */
	private final int NUMBER_OF_TIME_STEPS = 10;

	/**
	 * This function returns an array of intermediate 1D positions for each time
	 * step for a moving object. The object moves with velocity
	 * <code>velocity</code> for time <code>timeToTravel</code>. The number of
	 * time steps is given by <code>NUMBER_OF_TIME_STEPS</code>.
     * The first position in the array should be the <code>startPosition</code>
	 *
	 * @param startPosition
	 *            starting 1D position of the object
	 * @param timeToTravel
	 *            total time that the object travels
	 * @param velocity
	 *            velocity at which the object travels
	 * @return an array of 1D positions at each time step
	 */
	public double[] getIntermediatePositions(double startPosition,
			double timeToTravel, double velocity) {
		double[] intermediatePositions = new double[NUMBER_OF_TIME_STEPS];

		// your code here

		return intermediatePositions;
	}

	/**
	 * This function prints the Fibonacci sequence up to a maximum value of
	 * <code>maximumNumber</code>.
	 *
	 * @param maximumNumber
	 *            maximum number that can be printed
	 */
	public void printFibonacciSequence(int maximumNumber) {
		// your code here
	}

	public static void main(String[] args) {
		// create object of Exercise1 class
		Exercise1 ex1 = new Exercise1();

		System.out.println("Fibonacci Sequence: ");
		// call method of ex1 object to print Fibonacci Sequence
		ex1.printFibonacciSequence(100);

		// call method of ex1 object to get intermediate positions
		double[] positions = ex1.getIntermediatePositions(0.0, 1.0, 4.0);

		// display positions
		System.out.println("Intermediate positions are: ");
		for (int i = 0; i < positions.length; i++) {
			if (i != 0) {
				System.out.print(", ");
			}
			System.out.format("%.2f", positions[i]);
		}
	}
}
