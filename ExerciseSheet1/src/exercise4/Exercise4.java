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
	 * @param args
	 */
	public static void main(String[] args) {
		Scanner scanner = new Scanner(System.in);
		float diameter = 0.0f;
		int maxPrecision = 0;

		System.out.print("Input the diameter of type float: ");
		while (!scanner.hasNextFloat()) {
			System.out.print("Float, please: ");
			scanner.nextLine();
		}
		diameter = scanner.nextFloat();
		scanner.nextLine();

		System.out
				.print("Input the maximum precision (Integer from 1 to 100): ");
		while (!scanner.hasNextInt()) {
			System.out.print("Integer between 1 and 100, please: ");
			scanner.nextLine();
		}
		maxPrecision = scanner.nextInt();
		scanner.nextLine();

		scanner.close();

	}

}
