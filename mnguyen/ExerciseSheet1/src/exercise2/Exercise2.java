/*
 * Exercise2.java
 * 
 * @author:	Minh Nguyen
 * @date:	12.10.2015
 * */

package exercise2;

import java.util.ArrayList;
import java.util.Arrays;

public class Exercise2 {

	/**
	 * Prime generation method based on the implementation of the segmented
	 * sieve of Eratosthenes from http://primesieve.org
	 */
	public long segmentedSieve(long limit, int segmentSize) {

		int limitSqrt = (int) Math.sqrt((double) limit);
		boolean smallPrimes[] = new boolean[limitSqrt + 1];
		long count = 0;

		Arrays.fill(smallPrimes, true);
		for (int i = 2; i * i <= limitSqrt; i++) {
			if (smallPrimes[i]) {
				for (int j = i * i; j <= limitSqrt; j += i) {
					smallPrimes[j] = false;
				}
			}
		}

		ArrayList<Long> primes = new ArrayList<Long>();
		ArrayList<Integer> nextCompositeIndex = new ArrayList<Integer>();
		int smallPrime = 2;
		int prime = 3;
		boolean sieve[] = new boolean[segmentSize];

		for (long low = 0; low <= limit; low += segmentSize) {

			/* Handle end of range */
			long high = Math.min(low + segmentSize - 1, limit);

			Arrays.fill(sieve, true);

			/*
			 * Add the small primes and the corresponding composites in the next
			 * segment.
			 */
			for (; smallPrime * smallPrime <= high; smallPrime++) {
				if (smallPrimes[smallPrime]) {
					primes.add((long) smallPrime);
					nextCompositeIndex.add(smallPrime * smallPrime - (int) low);
				}
			}

			/* Sieve the current segment */
			for (int i = 1; i < primes.size(); i++) {
				int j = nextCompositeIndex.get(i);
				for (long k = primes.get(i) * 2; j < segmentSize; j += k) {
					sieve[j] = false;
					nextCompositeIndex.set(i, j - segmentSize);
				}
			}

			for (; prime <= high; prime += 2) {
				if (sieve[(int) (prime - low)]) { // a prime is found
					count++;
					System.out.print(prime + " ");
					if ((count % 10) == 0) {
						System.out.println();
					}
				}
			}

		}

		System.out.println();

		return count;

	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		Exercise2 ex2 = new Exercise2();

		long count = ex2.segmentedSieve(Integer.MAX_VALUE, 32768);

		System.out.println("Found " + count + " primes.");

	}

}
