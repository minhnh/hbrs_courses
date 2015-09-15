package Ex4;

import java.awt.Point;
import java.util.Random;

import ch.aplu.robotsim.*;

public class Ex4_2 implements TouchListener
{
	Gear gear = new Gear();
	UltrasonicSensor uSensor = new UltrasonicSensor(SensorPort.S1);
	
	Ex4_2()
	{
		NxtRobot robot = new NxtRobot();
		TouchSensor tSensor = new TouchSensor(SensorPort.S2);
		tSensor.addTouchListener(this);
		
		robot.addPart(gear);
		robot.addPart(uSensor);
		robot.addPart(tSensor);
		

		gear.right();
		int min = search();
		
		steerToMin(min);
		
		gear.forward();
		goToMin(uSensor);

	}
	
	public static void main(String[] args) 
	{
		new Ex4_2();
	}
	
	public void pressed(SensorPort arg0) 
	{
		gear.forward();
	}

	public void released(SensorPort arg0) 
	{
		gear.right();
		int min = search();
		
		steerToMin(min);
		
		gear.forward();
		goToMin(uSensor);
	}
	
	public int search()
	{
		int distance = 0;
		int min = 500;
		boolean searchHalf = false;
		
		while(true)
		{	
			distance = uSensor.getDistance();
			
			if(distance < min && distance != -1 && distance > 50)
			{
				min = distance;
			}
			NxtContext.setStatusText(String.valueOf(distance) + ": " + String.valueOf(min));
			
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
	


	public void steerToMin(int min)
	{
		int distance = 0;
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
