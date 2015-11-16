package exercise7;

import java.util.Random;

interface SampleGenerator<T extends SamplePose3D> {

	T createUniformSample(int sampleCount, Random r);

	T createGaussianSample(int sampleCount, Random r);

	boolean isInMap(T sample);
}
