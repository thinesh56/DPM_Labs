package ca.mcgill.ecse211.navigationlab;

public interface UltrasonicController {

	public void processUSData(int distance);

	public int readUSDistance();
}
