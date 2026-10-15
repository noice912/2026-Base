package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

/**
 * Objective 1.5 - Controls the CIM motor with percent output from the driver's right joystick (Y axis)
 */
public class CIMJoystickControl extends Command {

    public CIMJoystickControl() {
        super();
        addRequirements(Robot.cimMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // Pushing the stick forward gives a negative Y, so flip it so forward = positive
        double percent = -Robot.oi.driver.getRightY();

        // Ignore small stick movements so the motor doesn't creep when the stick is centered
        if (Math.abs(percent) < Constants.Joystick.DEADBAND) {
            percent = 0;
        }

        Robot.cimMotor.setPercent(percent);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false; // default command, runs whenever nothing else is using the CIM motor
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.cimMotor.setPercent(0);
    }
}
