package exercise7;

public class SamplePose2D extends SamplePose3D {
	static final float Z_MAX = 0.0f;
	static final float BETA_MAX = 0.0f;
	static final float GAMMA_MAX = 0.0f;

	// static inner class for SampleGenerator<T> implementation
	public static class SampleGeneratorClass implements SampleGenerator<SamplePose2D> {
		@Override
		public SamplePose2D createUniformSample() {
			return null;
		}
	}

	public SamplePose2D(float x, float y, float phi, float w) {
		super(x, y, 0.0f, phi, 0.0f, 0.0f, w);
	}

}
