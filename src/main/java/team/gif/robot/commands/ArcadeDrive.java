package team.gif.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

/**
 * Objective 1.12 - Drives the drivetrain with the driver's left joystick using arcade drive
 */
public class ArcadeDrive extends Command {

    public ArcadeDrive() {
        super();
        addRequirements(Robot.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // Pushing the stick forward gives a negative Y, so flip it so forward = positive
        double forward = -Robot.oi.driver.getLeftY();
        // Pushing the stick left gives a negative X, flip it so left = positive (counterclockwise)
        double rotation = -Robot.oi.driver.getLeftX();

        forward = MathUtil.applyDeadband(forward, Constants.Joystick.DEADBAND);
        rotation = MathUtil.applyDeadband(rotation, Constants.Joystick.DEADBAND);

        Robot.drivetrain.arcadeDrive(forward, rotation);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false; // default command, always drives with the joystick
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.drivetrain.arcadeDrive(0, 0);
    }
}
