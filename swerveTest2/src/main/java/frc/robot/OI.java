package frc.robot;

import frc.robot.commands.ZeroHeading;
import frc.robot.resources.TecbotController;
import frc.robot.resources.TecbotController.ButtonType;

public class OI {
    public static OI instance;
    private static TecbotController pilot;

    public OI(){
        pilot = new TecbotController(RobotMap.pilotPort);
    }

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
  
        return instance;
    }

    public static TecbotController getPilot(){
        return pilot;
    }


    public void configureButtonBindings(){
        pilot.whenPressed(ButtonType.A, new ZeroHeading());
    }
}
