package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class StopModules extends Command {
    public StopModules() {
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Robot.swerveDrive.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}