package exercise7;

public class Exercise7 {
    public static void generate2DSamples() {
	SampleSet<SamplePose2D> sampleSet = new SampleSet<>(
		new SamplePose2D.SampleGeneratorClass());
	int[] sampleSizes = { 100, 1000, 10000, 100000 };//
	for (int i : sampleSizes) {
	    String fileNameUniform = "samples/sample2DUniform" + Integer.toString(i) + ".csv";
	    String fileNameGaussian = "samples/sample2DGaussian" + Integer.toString(i) + ".csv";
	    sampleSet.generateUniformSet(i);
	    sampleSet.writeSetToFile(fileNameUniform);
	    sampleSet.generateGaussianSet(i);
	    sampleSet.writeSetToFile(fileNameGaussian);
	}
    }

    public static void generateMCLExample() {
	SampleSet<SamplePose2D> sampleSet = new SampleSet<>(
		new SamplePose2D.SampleGeneratorClass());
	int[] sampleSizes = { 100, 1000, 10000 };
	for (int i : sampleSizes) {
	    String fileNameUniform = "samples/sample2DMLCUniform" + Integer.toString(i) + ".csv";
	    String fileNameGaussian = "samples/sample2DMLCCalculated" + Integer.toString(i)
		    + ".csv";
	    sampleSet.generateUniformSet(i);
	    sampleSet.writeSetToFile(fileNameUniform);
	    sampleSet.createActualPose();
	    sampleSet.recalculateWeight();
	    sampleSet.writeSetToFile(fileNameGaussian);
	}
    }

    public static void main(String[] args) {
	generateMCLExample();
    }
}
