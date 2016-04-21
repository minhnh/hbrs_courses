package series4;

import java.awt.Point;
import java.util.Random;

import ch.aplu.robotsim.*;

public class Exercise2 implements TouchListener
{
	Gear gear = new Gear();
	UltrasonicSensor uSensor = new UltrasonicSensor(SensorPort.S1);
	
	Exercise2()
	{
		/*Instancing robot, part, and listener objects*/
		NxtRobot robot = new NxtRobot();
		TouchSensor tSensor = new TouchSensor(SensorPort.S2);
		tSensor.addTouchListener(this);
		
		/*Add parts to robot*/
		robot.addPart(gear);
		robot.addPart(uSensor);
		robot.addPart(tSensor);
		
		/*Start the robot: Search -> Steer -> Move to*/

		int min = search();
		steerToMin(min);
		goToMin(uSensor);

	}
	
	public static void main(String[] args) 
	{
		new Exercise2();
	}
	
	/*If the touch sensor is pressed, run over the obstacle*/
	public void pressed(SensorPort arg0) 
	{
		gear.forward();
	}

	/*If the touch sensor is released, restart sequence: Search -> Steer -> Move to*/
	public void released(SensorPort arg0) 
	{
		int min = search();	
		steerToMin(min);
		goToMin(uSensor);
	}
	
	/*Robot rotates 360 to search for the closest object*/
	public int search()
	{
		int distance = 0;
		int min = 500;
		boolean searchHalf = false;		//Indicated that the robot has searched half of the area
		
		gear.right();
		
		while(true)
		{	
			distance = uSensor.getDistance();
			
			/*Get the minimum distance*/
			if(distance < min && distance != -1 && distance > 50)
			{
				min = distance;
			}
			NxtContext.setStatusText(String.valueOf(distance) + ": " + String.valueOf(min));
			
			/*Break the search loop when a full 360 search is done*/
			if(gear.getDirection() == 180.0)
			{
				searchHalf = true;
			}
			else if (gear.getDirection() == 0 && searchHalf)
			{
				break;
			}
		}
		return min;
	}	
	

	/*Robot turns to the direction of the closet object*/
	public void steerToMin(int min)
	{
		int distance = 0;
		gear.right();
		
		/*The robot turns until the minimum distance is found again*/
		while(true)
		{	
			distance = uSensor.getDistance();
			NxtContext.setStatusText(String.valueOf(distance) + ": " + String.valueOf(min));
			if (distance == min)
			{
				gear.stop();
				break;
			}
		}
	}

	/*Robot goes to the closest object. It stops when the distance is smaller than 5*/
	public void goToMin(UltrasonicSensor uSensor)
	{
		int distance = 0;
		gear.forward();	
		while(true)
		{
			distance = uSensor.getDistance();
			NxtContext.setStatusText(String.valueOf(distance));
			if (distance < 5)
			{
				gear.stop();
				break;
			}
		}
	}
	
	/*Simulation environment*/
	static
	{
		Point[] mesh =
		    {
		      new Point(25, 0), new Point(12, 25), new Point(-12, 25), new Point (0, 25),
		      new Point(-25, 0), new Point(-12, -25), new Point(12, -25), new Point (0, -25)
		    };
		
		NxtContext.showStatusBar(50);
		NxtContext.setStartPosition(25, 25);
		NxtContext.setStartDirection(0);
		
		/*Create 3 obstacles with random positions*/
		Random rd = new Random();
		int x = 0;
		int y = 0;
		for (int i = 0; i < 3; i++)
		{
			x = rd.nextInt(450);
			y = rd.nextInt(450);
			NxtContext.useTarget("sprites/squaretarget.gif", mesh, x, y);
			NxtContext.useObstacle("sprites/squaretarget.gif", x, y);
		}
	}
}
