package Ex5;

import ch.aplu.robotsim.*;

public class Ex5_2
{
	Gear gear;
	boolean returnTrip = false;		//Indicate the direction of travel
	
	Ex5_2()
	{
		/*Instancing robot and part objects*/
		NxtRobot robot = new NxtRobot();
		gear = new Gear();
		LightSensor sensor1 = new LightSensor(SensorPort.S1);
		LightSensor sensor2 = new LightSensor(SensorPort.S2);
		TouchSensor tSensor = new TouchSensor(SensorPort.S1);
		int triggerLevel = 500;
		
		/*Add parts to robot*/
		robot.addPart(gear);
		robot.addPart(sensor1);
		robot.addPart(sensor2);
		robot.addPart(tSensor);
		
		gear.forward();		//Start the robot
		
		while(true)
		{
			/*If the touch sensor is pressed, the robot turns 180 and then goes forward*/
			if (tSensor.isPressed())
			{
				gear.right(1500);
				gear.forward();
				returnTrip = !returnTrip;		//Set the current traveling direction
			}
			
			/*If both light sensors are dark, turn the robot until both sensors are light
			 *Turing direction is determined by the direction of travel*/
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
			/*If both light sensors are bright, the robot goes forward*/
			else if (sensor1.getValue() > triggerLevel && sensor2.getValue() > triggerLevel)
			{
				gear.forward();
			}
			/*If right light sensors is dark, turn the robot to the left*/
			else if (sensor1.getValue() < triggerLevel)
			{
				gear.left();
			}
			/*If left light sensors is dark, turn the robot to the right*/
			else if (sensor2.getValue() < triggerLevel)
			{
				gear.right();
			}
			NxtContext.setStatusText(String.valueOf(tSensor.isPressed()));
		}
	}
	
	public static void main(String[] args) 
	{
		new Ex5_2();
	}
	
	/*Simulation environment*/
	static
	{
		NxtContext.useBackground("sprites/track.png");
		NxtContext.useObstacle("sprites/wall1.gif", 80, 450);
		NxtContext.useObstacle("sprites/wall2.gif", 430, 85);
		NxtContext.setStartPosition(80, 400);
		NxtContext.showStatusBar(50);
	}
}
