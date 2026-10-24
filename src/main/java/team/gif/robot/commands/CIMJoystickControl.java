package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CIMJoystickControl extends Command {
    public CIMJoystickControl() {
        super();
        addRequirements(Robot.cimMotor);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double percent = -Robot.oi.driver.getRightY();

        if (Math.abs(percent) < Constants.Joystick.DEADBAND) {
            percent = 0;
        }

        Robot.cimMotor.setPercent(percent);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.cimMotor.setPercent(0);
    }
}
