package exercise7;

interface SampleGenerator<T extends SamplePose3D> {
	T createUniformSample();
}
