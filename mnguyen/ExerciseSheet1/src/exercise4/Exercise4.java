/*
 * Exercise4.java
 * 
 * @author:	Minh Nguyen
 * @date:	12.10.2015
 * */

package exercise4;

import java.util.Scanner;

public class Exercise4 {

	/**
	 * Do approximation 10 decimal points at a time
	 */
	public double roundPi(int precision) {
		double result = 0.0d;

		int i = 0;
		for (i = precision; i > 9; i -= 10) {
			result += ((long) ((Math.PI - result) * Math.pow(10, precision - i)))
					/ Math.pow(10, precision - i);
		}

		result += ((long) ((Math.PI - result) * Math.pow(10, precision)))
				/ Math.pow(10, precision);

		return result;
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		Exercise4 ex4 = new Exercise4();
		Scanner scanner = new Scanner(System.in);
		double diameter = 0.0f;
		int maxPrecision = 0;

		System.out.print("Input the diameter of type double: ");
		while (!scanner.hasNextDouble()) {
			System.out.print("Double, please: ");
			scanner.nextLine();
		}
		diameter = scanner.nextDouble();
		scanner.nextLine();

		while (true) {
			System.out
					.print("Input the maximum precision (Integer from 0 to 20): ");
			if (!scanner.hasNextInt()) {
				System.out.print("Integer between 0 and 20, please: ");
				scanner.nextLine();
				continue;
			}

			maxPrecision = scanner.nextInt();
			scanner.nextLine();

			if (maxPrecision < 0 || maxPrecision > 20) {
				System.out.println("Number out of acceptable range");
				continue;
			}

			break;
		}

		scanner.close();

		System.out.format(
				"Precision%17sDiameter%3sIncrease %%%21sArea%3sIncrease %%\n",
				"", "", "", "");

		double circumference = 0.0d;
		double circumferencePercent = 0.0d;
		double area = 0.0d;
		double areaPercent = 0.0d;
		double temp = 0.0d;
		for (int i = 0; i < maxPrecision + 1; i++) {

			double pi = ex4.roundPi(i);

			temp = diameter * pi;
			if (circumference > 0.001) {
				circumferencePercent = (temp - circumference) / circumference
						* 100;
			}
			circumference = temp;

			temp = (diameter / 2) * (diameter / 2) * pi;
			if (area > 0.001) {
				areaPercent = (temp - area) / area * 100;
			}
			area = temp;

			System.out.format("%9d%25.20f%13.3e%25.20f%13.3e\n", i,
					circumference, circumferencePercent, area, areaPercent);
		}
	}
}
