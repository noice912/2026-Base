package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class RunCIMMotor extends Command {
    private final double percent;

    public RunCIMMotor(double percent) {
        super();
        this.percent = percent;
        addRequirements(Robot.cimMotor);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
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
