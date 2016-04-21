package exercise8;

import exercise7.SamplePose3D;
import exercise7.SampleSet;

public class Exercise8 {

	public static void generate3DSamples() {
		SampleSet<SamplePose3D> sampleSet = new SampleSet<>(new SamplePose3D.SampleGeneratorClass());
		int[] sampleSizes = { 100, 1000, 10000, 100000 };
		for (int i : sampleSizes) {
			String fileNameUniform = "samples/sample3DUniform" + Integer.toString(i) + ".csv";
			String fileNameGaussian = "samples/sample3DGaussian" + Integer.toString(i) + ".csv";
			sampleSet.generateUniformSet(i);
			sampleSet.writeSetToFile(fileNameUniform);
			sampleSet.generateGaussianSet(i);
			sampleSet.writeSetToFile(fileNameGaussian);
		}
	}

	public static void main(String[] args) {
		generate3DSamples();

	}

}
