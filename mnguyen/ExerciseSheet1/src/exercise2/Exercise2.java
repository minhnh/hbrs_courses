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
	public long segmentedSieve(long limit, int segmentSize, boolean print) {

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
		ArrayList<Integer> nextSegmentCompositeIndex = new ArrayList<Integer>();
		int smallPrime = 2;
		long prime = 3;
		boolean sieve[] = new boolean[segmentSize];

		for (long low = 0; low <= limit; low += segmentSize) {

			/* Handle end of range */
			long high = Math.min(low + segmentSize - 1, limit);

			Arrays.fill(sieve, true);

			/*
			 * Add the small primes and the corresponding composites in the next
			 * segment.
			 */
			for (; (long) smallPrime * smallPrime <= high; smallPrime++) {
				if (smallPrimes[smallPrime]) {
					primes.add((long) smallPrime);
					nextSegmentCompositeIndex.add((int) ((long) smallPrime
							* smallPrime - low));
				}
			}

			/* Sieve the current segment */
			for (int i = 1; i < nextSegmentCompositeIndex.size(); i++) {
				int j = nextSegmentCompositeIndex.get(i);
				for (long k = primes.get(i) * 2; j < segmentSize; j += k) {
					sieve[j] = false;
				}
				nextSegmentCompositeIndex.set(i, j - segmentSize);
			}

			for (; prime <= high; prime += 2) {
				if (sieve[(int) (prime - low)]) { // a prime is found
					count++;
					if (print) {
						System.out.print(prime + " ");
						if ((count % 10) == 0) {
							System.out.println();
						}
					}
				}
			}

		}

		if (print) {
			System.out.println();
		}

		return count;

	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		Exercise2 ex2 = new Exercise2();

		long time1 = System.nanoTime();
		long count = ex2.segmentedSieve(Integer.MAX_VALUE, 32768, false);
		long time2 = System.nanoTime();

		System.out.println("Found " + count + " primes.");
		System.out.format("Time elapsed in seconds: %.4f\n",
				((double) time2 - time1) / 1.0e9);

	}

}
