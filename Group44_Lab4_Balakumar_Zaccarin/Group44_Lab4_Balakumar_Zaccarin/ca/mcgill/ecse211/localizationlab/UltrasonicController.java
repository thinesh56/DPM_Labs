package ca.mcgill.ecse211.localizationlab;

public interface UltrasonicController {

	public void processUSData(int distance);

	public int readUSDistance();
}
