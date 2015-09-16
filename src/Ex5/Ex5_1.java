package Ex5;

import ch.aplu.robotsim.*;

public class Ex5_1 
{
	Ex5_1()
	{
		/*Instancing robot and part objects*/
		NxtRobot robot = new NxtRobot();
		Gear gear = new Gear();
		LightSensor lSensor = new LightSensor(SensorPort.S1);
		TouchSensor tSensor = new TouchSensor(SensorPort.S4);	//S4: touch sensor at the rear
		
		/*Add parts to robot*/
		robot.addPart(gear);
		robot.addPart(lSensor);
		robot.addPart(tSensor);
		
		gear.forward();	//Start the robot
		
		while(true)
		{	
			/*If the touch sensor is pressed, the robot stops*/
			if(tSensor.isPressed())
			{
				gear.stop();
			}
			/*If the light sensor is dark, the robot moves backward*/
			else if(lSensor.getValue() < 500)
			{
				gear.backward();
			}
			
		}
	}
	
	public static void main(String[] args) 
	{
		new Ex5_1();
	}
	
	/*Simulation environment*/
	static
	{
		NxtContext.showStatusBar(50);
		NxtContext.setStartPosition(200, 250);
		NxtContext.setStartDirection(0);
		NxtContext.useBackground("sprites/blackPanels.gif");
		NxtContext.useObstacle("sprites/wall2.gif", 100, 250);
	}
}
