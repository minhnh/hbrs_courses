package Ex5;

import ch.aplu.robotsim.*;

public class Ex5_1 
{
	Ex5_1()
	{
		NxtRobot robot = new NxtRobot();
		Gear gear = new Gear();
		LightSensor lSensor = new LightSensor(SensorPort.S1);
		TouchSensor tSensor = new TouchSensor(SensorPort.S4);
		
		robot.addPart(gear);
		robot.addPart(lSensor);
		robot.addPart(tSensor);
		
		gear.forward();
		
		while(true)
		{
			if(tSensor.isPressed())
			{
				gear.stop();
			}
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
	
	static
	{
		NxtContext.showStatusBar(50);
		NxtContext.setStartPosition(200, 250);
		NxtContext.setStartDirection(0);
		NxtContext.useBackground("sprites/blackPanels.gif");
		NxtContext.useObstacle("sprites/wall2.gif", 100, 250);
	}
}
