package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class RunNEOVoltage extends Command {
    private final double volts;

    public RunNEOVoltage(double volts) {
        super();
        this.volts = volts;
        addRequirements(Robot.neoMotor);
    }

    
    @Override
    public void initialize() {}


    @Override
    public void execute() {
        Robot.neoMotor.setVoltage(volts);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void end(boolean interrupted) {
        Robot.neoMotor.setVoltage(0);
    }
}
