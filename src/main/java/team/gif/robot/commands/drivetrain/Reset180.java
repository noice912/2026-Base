package team.gif.robot.commands.drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class Reset180 extends Command {
    public Reset180() {}

    @Override
    public void initialize() {
        Robot.pigeon.resetPigeonPosition(180);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
