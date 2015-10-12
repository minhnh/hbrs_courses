/*
 * Exercise1.java
 * 
 * @author:	Minh Nguyen
 * @date:	12.10.2015
 * */

package exercise1;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Scanner;

public class Exercise1 {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		Scanner scanner = new Scanner(System.in);
		ArrayList<Integer> numList = new ArrayList<Integer>();
		int n = 0;
		int sum = 0;
		int product = 1;
		float mean = 0.0f;
		float variance = 0.0f;

		while (true) {
			System.out.print("Input an integer number from 1 to 100: ");
			if (!scanner.hasNextInt()) {
				System.out.print("Integer between 1 and 100, please: ");
				scanner.nextLine();
				continue;
			}

			n = scanner.nextInt();
			scanner.nextLine();

			if (n < 1 || n > 100) {
				System.out.println("Number out of acceptable range");
				continue;
			}

			break;
		}

		System.out.print("Now input " + n + " integers: ");
		for (int i = 0; i < n; i++) {
			if (!scanner.hasNextInt()) {
				scanner.nextLine();
				System.out.print("Integer, please: ");
				i--;
				continue;
			}
			numList.add(scanner.nextInt());
		}

		scanner.close();

		for (int number : numList) {
			sum += number;
			product *= number;
		}

		mean = (float) sum / n;

		for (int number : numList) {
			variance += (number - mean) * (number - mean);
		}
		variance = variance / n;

		int max = Collections.max(numList);
		int min = Collections.min(numList);

		System.out.println("Sum:      " + sum);
		System.out.println("Product:  " + product);
		System.out.println("Average:  " + (int) (mean + 0.5));
		System.out.println("Variance: " + (int) (variance + 0.5));
		System.out.println("Maximum:  " + max);
		System.out.println("Minimum:  " + min);

	}
}
