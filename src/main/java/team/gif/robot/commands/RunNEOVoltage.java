package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class RunNEOVoltage extends Command {
    private final double volts;

    /**
     * Objective 1.6 - Runs the NEO motor at a voltage until the command ends
     * @param volts positive = forward, negative = backward
     */
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

    //
    @Override
    public boolean isFinished() {
        return false; // keep running until the button is released
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.neoMotor.setVoltage(0); // stop the motor when the button is released
    }
}
