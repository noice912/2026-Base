package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class DriveSwerve extends Command {
    private final SlewRateLimiter forwardLimiter;
    private final SlewRateLimiter strafeLimiter;
    private final SlewRateLimiter turnLimiter;

    public DriveSwerve() {
        this.forwardLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
        this.strafeLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
        this.turnLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND_SQUARED);
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
            double forwardSign;
            double strafeSign;

            double forward = -Robot.oi.driver.getLeftY();
            forward = (Math.abs(forward) > Constants.Joystick.DEADBAND) ? forward : 0.0;

            double strafe = -Robot.oi.driver.getLeftX();
            strafe = (Math.abs(strafe) > Constants.Joystick.DEADBAND) ? strafe : 0.0;

            double rot = -Robot.oi.driver.getRightX();
            rot = (Math.abs(rot) > Constants.Joystick.DEADBAND) ? rot : 0.0;

            forwardSign = forward/Math.abs(forward);
            strafeSign = strafe/Math.abs(strafe);
            forward = Math.abs(forward) * forward;
            strafe = Math.abs(strafe) * strafe;

            forward = .5 * Math.sqrt(2 + forward*forward - strafe*strafe + 2*forward*Math.sqrt(2)) -
                    .5 * Math.sqrt(2 + forward*forward - strafe*strafe - 2*forward*Math.sqrt(2));

            strafe = .5 * Math.sqrt(2 - forward*forward + strafe*strafe + 2*strafe*Math.sqrt(2)) -
                    .5 * Math.sqrt(2 - forward*forward + strafe*strafe - 2*strafe*Math.sqrt(2));

            if( Double.isNaN(forward) )
                forward = forwardSign;
            if( Double.isNaN(strafe) )
                strafe = strafeSign;

            forward = forwardLimiter.calculate(forward) * Robot.swerveDrive.getDrivePace().getValue();
            strafe = strafeLimiter.calculate(strafe) * Robot.swerveDrive.getDrivePace().getValue();

            if (rot < 0 ) {
                rot = rot * -rot;
            } else {
                rot = rot * rot;
            }

             rot = turnLimiter.calculate(rot) * Robot.swerveConfig.constants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

            Robot.swerveDrive.drive(forward, strafe, rot);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
