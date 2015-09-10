/**
 * Exercise2
 * @author minhnh
 * @version 1.0
 * */
import java.util.ArrayList;
import java.util.Random;

public class Exercise2 {

	/**
	 * Calculate standard deviation and mean. Print out results.
	 * 
	 * @param n
	 *            : number of samples
	 * @param meanT
	 *            : true mean
	 * @param stdDeviationT
	 *            : true standard deviation
	 * @return void
	 */
	public void calStdDeviationAndMean(int n, double meanT, double stdDeviationT) {

		/* Variable declaration and initialization */
		double sum = 0.0d;
		double meanS = 0.0d;
		double deviationSum = 0.0d;
		double stdDeviationS = 0.0d;
		double meanDiff = 0.0d;
		double stdDeviationDiffPercent = 0.0d;
		ArrayList<Double> data = new ArrayList<Double>();
		Random r = new Random(System.currentTimeMillis());

		/* Generate data */
		for (int i = 0; i < n; i++) {
			data.add(r.nextGaussian() * stdDeviationT + meanT);
			sum += data.get(i);
		}

		/* Calculate mean of samples */
		meanS = sum / n;

		/* Calculate standard deviation of samples */
		for (double sample : data) {
			deviationSum += (sample - meanS) * (sample - meanS);
		}
		stdDeviationS = Math.sqrt(deviationSum / n);

		/*
		 * Calculate absolute mean difference and standard deviation difference
		 * percentage
		 */
		meanDiff = Math.abs(meanS - meanT);
		stdDeviationDiffPercent = Math.abs(stdDeviationS - stdDeviationT)
				/ stdDeviationT * 100;

		/* Print out formatted result */
		System.out.format("%10d %10.2f %10.2f %10.4f %10.4f %10.4f %10.4f\n",
				n, meanT, stdDeviationT, meanS, stdDeviationS, meanDiff,
				stdDeviationDiffPercent);

		return;
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		Exercise2 ex2 = new Exercise2();

		/* Calculate and print out results */
		System.out.println("         N  Mean True    SD True  Mean Samp    "
				+ "SD Samp  Mean Diff  SD Diff %");
		ex2.calStdDeviationAndMean(10, 0.0, 1.0);
		ex2.calStdDeviationAndMean(10, 5.0, 3.0);
		ex2.calStdDeviationAndMean(10, 20.0, 5.0);
		ex2.calStdDeviationAndMean(1000, 0.0, 1.0);
		ex2.calStdDeviationAndMean(1000, 5.0, 3.0);
		ex2.calStdDeviationAndMean(1000, 20.0, 5.0);
		ex2.calStdDeviationAndMean(100000, 0.0, 1.0);
		ex2.calStdDeviationAndMean(100000, 5.0, 3.0);
		ex2.calStdDeviationAndMean(100000, 20.0, 5.0);
	}
}
