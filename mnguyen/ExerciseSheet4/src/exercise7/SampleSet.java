package exercise7;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

public class SampleSet<T extends SamplePose3D> {
	static final String newLine = System.getProperty("line.separator");
	ArrayList<T> samples;
	private SampleGenerator<T> generator;
	private T actualPose = null;

	public SampleSet(SampleGenerator<T> generator) {
		this.samples = new ArrayList<>();
		this.generator = generator;
	}

	public void createActualPose() {
		Random r = new Random(System.currentTimeMillis());
		actualPose = this.generator.createGaussianSample(samples.size(), r);
		recalculateWeight();
	}

	private void recalculateWeight() {
		if (actualPose == null)
			return;
		for (T sample : samples) {
			sample.recalculateWeight(actualPose);
		}
	}

	public void generateUniformSet(int sampleCount) {

		Random r = new Random(System.currentTimeMillis());
		int setSize = samples.size();

		if (setSize < sampleCount) {
			for (int i = 0; i < setSize; i++) {
				samples.set(i, this.generator.createUniformSample(sampleCount, r));
			}
			for (int i = setSize; i < sampleCount; i++) {
				samples.add(this.generator.createUniformSample(sampleCount, r));
			}
		} else {
			for (int i = 0; i < sampleCount; i++) {
				samples.set(i, this.generator.createUniformSample(sampleCount, r));
			}
			for (int i = sampleCount; i < setSize - sampleCount; i++) {
				samples.remove(i);
			}
		}
	}

	public void generateGaussianSet(int sampleCount) {

		Random r = new Random(System.currentTimeMillis());
		int setSize = samples.size();

		if (setSize < sampleCount) {
			for (int i = 0; i < setSize; i++) {
				samples.set(i, this.generator.createGaussianSample(sampleCount, r));
			}
			for (int i = setSize; i < sampleCount; i++) {
				samples.add(this.generator.createGaussianSample(sampleCount, r));
			}
		} else {
			for (int i = 0; i < sampleCount; i++) {
				samples.set(i, this.generator.createGaussianSample(sampleCount, r));
			}
			for (int i = sampleCount; i < setSize - sampleCount; i++) {
				samples.remove(i);
			}
		}
	}

	public void writeSetToFile(String fileName) {
		try {
			FileWriter f = new FileWriter(fileName);

			if (actualPose == null) {
				T dummyPose = this.generator.createGaussianSample(0, null);
				f.write(dummyPose.toString() + newLine);
			} else {
				f.write(actualPose.toString() + newLine);
			}
			for (T sample : this.samples) {
				f.write(sample.toString() + newLine);
			}

			f.close();
		} catch (IOException e) {
			System.err.println("Can't open file " + fileName + ": " + e);
			e.printStackTrace();
		}
	}
}
