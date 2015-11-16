package exercise7;

import java.util.Random;

public class SamplePose2D extends SamplePose3D {
	static final float Z_MAX = 0.0f;
	static final float BETA_MAX = 0.0f;
	static final float GAMMA_MAX = 0.0f;
	static SamplePose2D[] meanSamples = { new SamplePose2D(-140.0f, 20.0f, 125.0f, 0.0f),
			new SamplePose2D(-60.0f, 20.0f, 270.0f, 0.0f), new SamplePose2D(20.0f, 120.0f, 0.0f, 0.0f),
			new SamplePose2D(80.0f, 0.0f, 225.0f, 0.0f), new SamplePose2D(100.0f, -100.0f, 125.0f, 0.0f) };

	// static inner class for SampleGenerator<T> implementation
	public static class SampleGeneratorClass implements SampleGenerator<SamplePose2D> {
		@Override
		public SamplePose2D createUniformSample(int sampleCount) {
			SamplePose2D sample;
			Random r = new Random(System.currentTimeMillis());
			float w = 1.0f / sampleCount;
			do {
				float x = SamplePose2D.X_MIN + r.nextFloat() * (SamplePose2D.X_MAX - SamplePose2D.X_MIN);
				float y = SamplePose2D.Y_MIN + r.nextFloat() * (SamplePose2D.Y_MAX - SamplePose2D.Y_MIN);
				float alpha = SamplePose2D.ALPHA_MIN
						+ r.nextFloat() * (SamplePose2D.ALPHA_MAX - SamplePose2D.ALPHA_MIN);
				sample = new SamplePose2D(x, y, alpha, w);
			} while (!isInMap(sample));
			return sample;
		}

		@Override
		public SamplePose2D createGaussianSample(int sampleCount) {
			SamplePose2D sample;
			Random r = new Random(System.currentTimeMillis());
			float w = 1.0f / sampleCount;
			do {
				SamplePose2D meanSample = meanSamples[r.nextInt(meanSamples.length)];
				float x = (float) (meanSample.getX() + r.nextGaussian() * sigmaLinear);
				float y = (float) (meanSample.getY() + r.nextGaussian() * sigmaLinear);
				float alpha = (float) (meanSample.getAlpha() + r.nextGaussian() * sigmaAngular);
				sample = new SamplePose2D(x, y, alpha, w);
			} while (!isInMap(sample));
			return sample;
		}

		@Override
		public boolean isInMap(SamplePose2D sample) {
			if (sample.getX() + sample.getY() < 200.0 && sample.getX() + sample.getY() > -200.0) {
				return true;
			}
			return false;
		}
	}

	public SamplePose2D(float x, float y, float phi, float w) {
		super(x, y, Z_MAX, phi, BETA_MAX, GAMMA_MAX, w);
	}

}
