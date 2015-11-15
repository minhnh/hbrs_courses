package exercise7;

public class SamplePose3D {
	static final float X_MAX = 200.0f;
	static final float X_MIN = -200.0f;
	static final float Y_MAX = 200.0f;
	static final float Y_MIN = -200.0f;
	static final float Z_MAX = 20.0f;
	static final float Z_MIN = 0.0f;
	static final float ALPHA_MAX = 360.0f;
	static final float ALPHA_MIN = 0.0f;
	static final float BETA_MAX = 360.0f;
	static final float BETA_MIN = 0.0f;
	static final float GAMMA_MAX = 360.0f;
	static final float GAMMA_MIN = 0.0f;
	private float x;
	private float y;
	private float z;
	private float alpha;
	private float beta;
	private float gamma;
	private float w;

	// static inner class for SampleGenerator<T> implementation
	public static class SampleGeneratorClass implements SampleGenerator<SamplePose3D> {
		@Override
		public SamplePose3D createUniformSample() {
			return null;
		}
	}

	public SamplePose3D(float x, float y, float z, float alpha, float beta, float gamma, float w) {
		this.x = checkValue(x, X_MIN, X_MAX);
		this.y = checkValue(y, Y_MIN, Y_MAX);
		this.z = checkValue(z, Z_MIN, Z_MAX);
		this.alpha = checkValue(alpha, ALPHA_MIN, ALPHA_MAX);
		this.beta = checkValue(beta, BETA_MIN, BETA_MAX);
		this.gamma = checkValue(gamma, GAMMA_MIN, GAMMA_MAX);
		this.w = w;
	}

	public void setWeight(float w) {
		this.w = w;
	}

	public float getWeight() {
		return this.w;
	}

	private float checkValue(float value, float lowerBound, float upperBound) {
		if (value > upperBound) {
			return upperBound;
		} else if (value < lowerBound) {
			return lowerBound;
		} else {
			return value;
		}
	}

}
