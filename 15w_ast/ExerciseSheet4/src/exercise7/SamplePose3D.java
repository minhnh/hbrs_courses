package exercise7;

import java.util.Random;

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

    // TODO Build actual mean samples for 3D
    static SamplePose3D[] meanSamples = { new SamplePose2D(-140.0f, 20.0f, 125.0f, 0.0f),
	    new SamplePose2D(-60.0f, 20.0f, 270.0f, 0.0f),
	    new SamplePose2D(20.0f, 120.0f, 0.0f, 0.0f),
	    new SamplePose2D(80.0f, 0.0f, 225.0f, 0.0f),
	    new SamplePose2D(100.0f, -100.0f, 125.0f, 0.0f) };
    static float sigmaLinear = 20.0f;
    static float sigmaAngular = 10.0f;

    private float x;
    private float y;
    private float z;
    private float alpha;
    private float beta;
    private float gamma;
    private float w;
    private float weightSumThreshold = 0.0f;

    public SamplePose3D(float x, float y, float z, float alpha, float beta, float gamma, float w) {
	this.x = checkValue(x, X_MIN, X_MAX);
	this.y = checkValue(y, Y_MIN, Y_MAX);
	this.z = checkValue(z, Z_MIN, Z_MAX);
	this.alpha = checkValue(alpha, ALPHA_MIN, ALPHA_MAX);
	this.beta = checkValue(beta, BETA_MIN, BETA_MAX);
	this.gamma = checkValue(gamma, GAMMA_MIN, GAMMA_MAX);
	this.w = w;
    }

    // static inner class for SampleGenerator<T> implementation
    public static class SampleGeneratorClass implements SampleGenerator<SamplePose3D> {
	@Override
	public SamplePose3D createUniformSample(int sampleCount, Random r) {
	    if (sampleCount == 0)
		return new SamplePose3D(0, 0, 0, 0, 0, 0, 0);
	    SamplePose3D sample;
	    float w = 1.0f / sampleCount;
	    do {
		float x = SamplePose3D.X_MIN
			+ r.nextFloat() * (SamplePose3D.X_MAX - SamplePose3D.X_MIN);
		float y = SamplePose3D.Y_MIN
			+ r.nextFloat() * (SamplePose3D.Y_MAX - SamplePose3D.Y_MIN);
		float z = SamplePose3D.Z_MIN
			+ r.nextFloat() * (SamplePose3D.Z_MAX - SamplePose3D.Z_MIN);
		float alpha = SamplePose3D.ALPHA_MIN
			+ r.nextFloat() * (SamplePose3D.ALPHA_MAX - SamplePose3D.ALPHA_MIN);
		float beta = SamplePose3D.BETA_MIN
			+ r.nextFloat() * (SamplePose3D.BETA_MAX - SamplePose3D.BETA_MIN);
		float gamma = SamplePose3D.GAMMA_MIN
			+ r.nextFloat() * (SamplePose3D.GAMMA_MAX - SamplePose3D.GAMMA_MIN);
		sample = new SamplePose3D(x, y, z, alpha, beta, gamma, w);
	    } while (!isInMap(sample));
	    return sample;
	}

	@Override
	public boolean isInMap(SamplePose3D sample) {
	    if (sample.getX() + sample.getY() < 200.0 && sample.getX() + sample.getY() > -200.0) {
		return true;
	    }
	    return false;
	}

	@Override
	public SamplePose3D createGaussianSample(int sampleCount, Random r,
		SamplePose3D meanSample) {
	    if (sampleCount == 0)
		return new SamplePose3D(0, 0, 0, 0, 0, 0, 0);
	    SamplePose3D sample;
	    float w = 1.0f / sampleCount;
	    do {
		if (meanSample == null) {
		    meanSample = meanSamples[r.nextInt(meanSamples.length)];
		}
		float x = (float) (meanSample.getX() + r.nextGaussian() * sigmaLinear);
		float y = (float) (meanSample.getY() + r.nextGaussian() * sigmaLinear);
		float z = (float) (meanSample.getZ() + r.nextGaussian() * sigmaLinear);
		float alpha = (float) (meanSample.getAlpha() + r.nextGaussian() * sigmaAngular);
		float beta = (float) (meanSample.getBeta() + r.nextGaussian() * sigmaAngular);
		float gamma = (float) (meanSample.getGamma() + r.nextGaussian() * sigmaAngular);
		sample = new SamplePose3D(x, y, z, alpha, beta, gamma, w);
	    } while (!isInMap(sample));
	    return sample;
	}
    }

    public void recalculateWeight(SamplePose3D actualPose) {
	float gaussianX = (float) calculateGaussian(this.getX(), actualPose.getX(),
		SamplePose3D.sigmaLinear);
	float gaussianY = (float) calculateGaussian(this.getY(), actualPose.getY(),
		SamplePose3D.sigmaLinear);
	float gaussianZ = (float) calculateGaussian(this.getY(), actualPose.getY(),
		SamplePose3D.sigmaLinear);
	float gaussianAlpha = (float) calculateGaussian(this.getAlpha(), actualPose.getAlpha(),
		SamplePose3D.sigmaAngular);
	float gaussianBeta = (float) calculateGaussian(this.getBeta(), actualPose.getBeta(),
		SamplePose3D.sigmaAngular);
	float gaussianGamma = (float) calculateGaussian(this.getGamma(), actualPose.getGamma(),
		SamplePose3D.sigmaAngular);
	this.setWeight(
		gaussianX * gaussianY * gaussianZ * gaussianAlpha * gaussianBeta * gaussianGamma);
    }

    public double calculateGaussian(double currentValue, double actualValue, double deviation) {
	return Math.exp(-Math.pow((currentValue - actualValue), 2.0d)
		/ (2.0d * Math.pow(deviation, 2)));
    }

    @Override
    public String toString() {
	return Float.toString(this.getX()) + "," + Float.toString(this.getY()) + ","
		+ Float.toString(this.getZ()) + ","
		+ Float.toString(this.getAlpha()) + "," + Float.toString(this.getBeta()) + ","
		+ Float.toString(this.getGamma()) + "," + Float.toString(this.getWeight());
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

    public void setWeight(float w) {
	this.w = w;
    }

    public float getWeight() {
	return this.w;
    }

    public float getX() {
	return this.x;
    }

    public float getY() {
	return this.y;
    }

    public float getZ() {
	return this.z;
    }

    public float getAlpha() {
	return this.alpha;
    }

    public float getBeta() {
	return this.beta;
    }

    public float getGamma() {
	return this.gamma;
    }

    public void setWeightThreshold(float threshold) {
	this.weightSumThreshold = threshold;
    }

    public float getWeightThreshold() {
	return weightSumThreshold;
    }

}
