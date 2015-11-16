package exercise7;

interface SampleGenerator<T extends SamplePose3D> {
	T createUniformSample(int sampleCount);

	T createGaussianSample(int sampleCount);

	boolean isInMap(T sample);
}
