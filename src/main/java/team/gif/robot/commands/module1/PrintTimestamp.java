package team.gif.robot.commands.module1;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Objective 1.1 - Prints a timestamp to the console when a button is pressed
 */
public class PrintTimestamp extends Command {

    public PrintTimestamp() {
        super();
        // no addRequirements() - printing doesn't use any subsystem hardware
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // FPGA timestamp = seconds since the robot code started
        System.out.println("Button pressed at: " + Timer.getFPGATimestamp() + " s");
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return true; // only needs to print once
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Allow this command to run while the robot is disabled
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
