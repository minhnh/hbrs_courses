package exercise7;

public class Sample3D {
	float x;
	float y;
	float z;
	float alpha;
	float beta;
	float gamma;
	float w;

	public Sample3D(float x, float y, float z, float alpha, float beta, float gamma, float w) {
		this.x = checkValue(x, -200.0f, 200.0f);
		this.y = checkValue(y, -200.0f, 200.0f);
		this.z = checkValue(z, 0.0f, 200.0f);
		this.alpha = checkValue(alpha, 0.0f, 360.0f);
		this.beta = checkValue(beta, 0.0f, 360.0f);
		this.gamma = checkValue(gamma, 0.0f, 360.0f);
		this.w = w;
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
