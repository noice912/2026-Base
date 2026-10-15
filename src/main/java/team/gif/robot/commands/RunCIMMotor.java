package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class RunCIMMotor extends Command {
    private final double percent;

    /**
     * Runs the CIM motor at a percent output until the command ends
     * @param percent positive = forward, negative = backward
     */
    public RunCIMMotor(double percent) {
        super();
        this.percent = percent;
        addRequirements(Robot.cimMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // Set every loop so the limit switch (objective 1.10) is checked while the button is held
        Robot.cimMotor.setPercent(percent);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false; // keep running until the button is released
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.cimMotor.setPercent(0); // stop the motor when the button is released
    }
}
