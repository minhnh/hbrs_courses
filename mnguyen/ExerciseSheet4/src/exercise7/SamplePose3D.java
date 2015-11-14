package exercise7;

public class SamplePose3D {
	static final float XY_MAX = 200.0f;
	static final float XY_MIN = -200.0f;
	static final float Z_MAX = 20.0f;
	static final float Z_MIN = 0.0f;
	static final float ANGLE_MAX = 360.0f;
	static final float ANGLE_MIN = 0.0f;
	private float x;
	private float y;
	private float z;
	private float alpha;
	private float beta;
	private float gamma;
	private float w;

	public SamplePose3D(float x, float y, float z, float alpha, float beta, float gamma, float w) {
		this.x = checkValue(x, XY_MIN, XY_MAX);
		this.y = checkValue(y, XY_MIN, XY_MAX);
		this.z = checkValue(z, Z_MIN, Z_MAX);
		this.alpha = checkValue(alpha, ANGLE_MIN, ANGLE_MAX);
		this.beta = checkValue(beta, ANGLE_MIN, ANGLE_MAX);
		this.gamma = checkValue(gamma, ANGLE_MIN, ANGLE_MAX);
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
