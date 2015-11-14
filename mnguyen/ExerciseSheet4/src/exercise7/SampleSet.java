package exercise7;

import java.util.ArrayList;

public class SampleSet<T extends SamplePose3D> {
	private ArrayList<T> samples;

	public SampleSet() {
		this.samples = new ArrayList<>();
	}

	public void generateUniform(int count) {
		for (int i = 0; i < count; i++) {
		}
	}

	public void generateGaussian(int count) {

	}

}
