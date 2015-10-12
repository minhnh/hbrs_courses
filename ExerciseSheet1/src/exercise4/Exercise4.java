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

		System.out.print("Input the diameter of type double: ");
		while (!scanner.hasNextInt()) {
			System.out.print("Integer between 1 and 100, please: ");
			scanner.nextLine();
		}
		int n = scanner.nextInt();
		scanner.nextLine();

	}

}
