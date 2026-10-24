package team.gif.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ArcadeDrive extends Command {
    public ArcadeDrive() {
        super();
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double forward = -Robot.oi.driver.getLeftY();
        double rotation = -Robot.oi.driver.getLeftX();

        forward = MathUtil.applyDeadband(forward, Constants.Joystick.DEADBAND);
        rotation = MathUtil.applyDeadband(rotation, Constants.Joystick.DEADBAND);

        Robot.drivetrain.arcadeDrive(forward, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.drivetrain.arcadeDrive(0, 0);
    }
}
