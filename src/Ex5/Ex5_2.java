package Ex5;

import ch.aplu.robotsim.*;

public class Ex5_2
{
	Gear gear;
	boolean returnTrip = false;
	
	Ex5_2()
	{
		NxtRobot robot = new NxtRobot();
		gear = new Gear();
		LightSensor sensor1 = new LightSensor(SensorPort.S1);
		LightSensor sensor2 = new LightSensor(SensorPort.S2);
		TouchSensor tSensor = new TouchSensor(SensorPort.S1);
		int triggerLevel = 500;
		
		robot.addPart(gear);
		robot.addPart(sensor1);
		robot.addPart(sensor2);
		robot.addPart(tSensor);
		
		gear.forward();
		
		while(true)
		{
			if (tSensor.isPressed())
			{
				gear.right(1500);
				gear.forward();
				returnTrip = !returnTrip;
			}
			
			if (sensor1.getValue() < triggerLevel && sensor2.getValue() < triggerLevel)
			{
				if(returnTrip)
				{
					gear.left();
				}
				else
				{
					gear.right();
				}
				
			}
			else if (sensor1.getValue() > triggerLevel && sensor2.getValue() > triggerLevel)
			{
				gear.forward();
			}
			else if (sensor1.getValue() < triggerLevel)
			{
				gear.left();
				NxtContext.setStatusText("Left");
			}
			else if (sensor2.getValue() < triggerLevel)
			{
				gear.right();
				NxtContext.setStatusText("right");
			}
			NxtContext.setStatusText(String.valueOf(tSensor.isPressed()));
		}
	}
	
	public static void main(String[] args) 
	{
		new Ex5_2();
	}
	
	static
	{
		NxtContext.useBackground("sprites/track.png");
		NxtContext.useObstacle("sprites/wall1.gif", 80, 450);
		NxtContext.useObstacle("sprites/wall2.gif", 430, 85);
		NxtContext.setStartPosition(80, 400);
		NxtContext.showStatusBar(50);
	}
}
