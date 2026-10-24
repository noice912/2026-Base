package team.gif.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TimeStamp extends Command {

    public TimeStamp() {
        super();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        System.out.println(Timer.getFPGATimestamp());
    }

    @Override
    public boolean isFinished() {

        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}

