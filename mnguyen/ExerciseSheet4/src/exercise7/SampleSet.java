package exercise7;

import java.util.ArrayList;

public class SampleSet<T extends SamplePose3D> {
	private ArrayList<T> samples;
	private SampleGenerator<T> generator;

	public SampleSet(SampleGenerator<T> generator) {
		this.samples = new ArrayList<>();
		this.generator = generator;
	}

	public void generateUniform(int count) {
		for (int i = 0; i < count; i++) {
			samples.add(this.generator.createUniformSample());
		}
	}

}
