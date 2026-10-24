package team.gif.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UI {
    public UI() {
        
    }

    public void update() {

        SmartDashboard.putBoolean("Limit Switch", Robot.limitSwitch.isPressed());
        SmartDashboard.putNumber("Pigeon Heading", Robot.pigeon.get180Heading());

        SmartDashboard.putNumber("NEO Position", Robot.neoMotor.getPosition());
    }
}
