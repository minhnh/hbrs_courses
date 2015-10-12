/*
 * Exercise3.java
 * 
 * @author:	Minh Nguyen
 * @date:	12.10.2015
 * */

package exercise3;

public class Exercise3 {

	/* f(n) = 2*n */
	public int function1(int n) {
		return 2 * n;
	}

	/* f(n) = n^3 */
	public int function2(int n) {
		return n * n * n;
	}

	/* f(n) = n^(1/2) */
	public double function3(int n) {
		return Math.sqrt((double) n);
	}

	/* f(n) = 2^(1/n) */
	public double function4(int n) {
		return Math.pow(2, (double) 1 / n);
	}

	/* f(n) = 10^n */
	public double function5(int n) {
		return Math.pow(10, n);
	}

	/* f(n) = e^n */
	public double function6(int n) {
		return Math.pow(Math.E, n);
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		Exercise3 ex3 = new Exercise3();
		int function1Data = 0;
		int function2Data = 0;
		double function3Data = 0.0d;
		double function4Data = 0.0d;
		double function5Data = 0;
		double function6Data = 0.0d;

		System.out.println("            n" + "     f(n)=2*n" + "     f(n)=n^3"
				+ " f(n)=n^(1/2)" + " f(n)=2^(1/n)" + "    f(n)=10^n"
				+ "     f(n)=e^n");
		for (int i = 1; i < 101; i++) {
			function1Data = ex3.function1(i);
			function2Data = ex3.function2(i);
			function3Data = ex3.function3(i);
			function4Data = ex3.function4(i);
			function5Data = ex3.function5(i);
			function6Data = ex3.function6(i);
			System.out.format("%13d%13d%13d%13.3f%13.3f%13.3e%13.3e\n", i,
					function1Data, function2Data, function3Data, function4Data,
					function5Data, function6Data);
		}

	}

}
