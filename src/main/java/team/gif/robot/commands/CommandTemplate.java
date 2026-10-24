package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandTemplate extends Command {

    public CommandTemplate() {
        super();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
