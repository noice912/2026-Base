package team.gif.robot.subsystems.drivers.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import team.gif.robot.Constants;
import team.gif.robot.subsystems.drivers.swerve.utilities.DriveMotor;
import team.gif.robot.subsystems.drivers.swerve.utilities.Encoder;
import team.gif.robot.subsystems.drivers.swerve.utilities.TurnMotor;

public class SwerveModule {
    private final DriveMotor driveMotor;
    private final TurnMotor turnMotor;
    private final Encoder encoder;

    private final double P;
    private final SimpleMotorFeedforward driveFF;
    private final double turnFF;

    private final double turnOffset;

    private final double maxDriveTemp;

    private double targetAngle = 0;
    private double targetVelocity = 0;

    public SwerveModule(
            DriveMotor driveMotor,
            TurnMotor turnMotor,
            Encoder encoder,
            boolean isTurnInverted,
            boolean isDriveInverted,
            double turningOffset,
            SimpleMotorFeedforward driveFF,
            double turnFF,
            double P,
            double maxDriveTemp
    ) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.encoder = encoder;

        this.driveMotor.configure(isDriveInverted);
        this.turnMotor.configure(isTurnInverted);
        this.encoder.configure();

        this.turnOffset = turningOffset;
        this.P = P;

        this.driveFF = driveFF;
        this.turnFF = turnFF;

        this.maxDriveTemp = maxDriveTemp;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(),
                new Rotation2d(getTurningHeading()));
    }

    public double getRawHeading() {
        return encoder.getTicks();
    }

    public double encoderDegrees() {
        return encoder.getDegrees();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity();
    }
    public double getTurningHeading() {
        double heading = Units.degreesToRadians(getTurningHeadingDegrees());
        heading %= 2 * Math.PI;
        return heading;
    }

    public double getTurningHeadingDegrees() {
        return encoder.getDegrees() - turnOffset;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }
    public void resetWheel() {
        final double error = getTurningHeading();
        final double ff = turnFF * Math.abs(error) / error;
        final double turnOutput = ff + (P * error);

        turnMotor.set(turnOutput);
    }

    private double findRevAngle(double radians) {
        return (Math.PI * 2 + radians) % (2 * Math.PI) - Math.PI;
    }

    private double getDistance(double setpoint, double position) {
        return Math.abs(setpoint - position);
    }

    private SwerveModuleState optimizeState(SwerveModuleState original) {
        double position = getTurningHeading();
        double setpoint = original.angle.getRadians();
        double forward = setpoint + (2 * Math.PI);
        double reverse = setpoint - (2 * Math.PI);
        double antisetpoint = findRevAngle(setpoint);
        double antiforward = antisetpoint + (2 * Math.PI);
        double antireverse = antisetpoint - (2 * Math.PI);

        double[] alternatives = { forward, reverse, antisetpoint, antiforward, antireverse };
        double min = setpoint;
        double minDistance = getDistance(setpoint, position);
        int minIndex = -1;
        for (int i = 0; i < alternatives.length; i++) {
            double dist = getDistance(alternatives[i], position);
            if (dist < minDistance) {
                min = alternatives[i];
                minDistance = dist;
                minIndex = i;
            }
        }

        double speed = original.speedMetersPerSecond;
        if (minIndex > 1) {
            speed *= -1;
        }

        return new SwerveModuleState(speed, new Rotation2d(min));
    }

    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState stateOptimized = optimizeState(state);
        targetAngle = stateOptimized.angle.getRadians();
        targetVelocity = stateOptimized.speedMetersPerSecond;

        double driveOutput = driveFF.calculate(stateOptimized.speedMetersPerSecond);
        final double error = getTurningHeading() - stateOptimized.angle.getRadians();

        final double ff = turnFF * Math.abs(error) / error;
        final double turnOutput = ff + (P * error);
        driveMotor.setVoltage(driveOutput);
        turnMotor.set(turnOutput);
    }

    public void setDesiredState(SwerveModuleState state,  boolean moveCW) {
        state = optimizeState(state);
        targetAngle = state.angle.getRadians();
        targetVelocity = state.speedMetersPerSecond;
        double driveOutput = driveFF.calculate(state.speedMetersPerSecond);

        double error = getTurningHeading() - state.angle.getRadians();

        if (moveCW) {
            error = Math.abs(error);
        } else {
            error = Math.abs(error) * -1;
        }

        final double ff = turnFF * Math.abs(error) / error;
        final double turnOutput = ff + (P * error);
        driveMotor.setVoltage(driveOutput);
        turnMotor.set(turnOutput);
    }

    public void turnHoldZero() {
        double error = getTurningHeading();
        final double ff = turnFF * Math.abs(error) / error;
        final double turnOutput = ff + (P * error);
        turnMotor.set(turnOutput);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition(), new Rotation2d(getTurningHeading()));
    }

    public void resetDriveEncoders() {
        driveMotor.resetEncoder();
    }

    public boolean isDriveMotorHot() {
        return driveMotor.getTemp() >= maxDriveTemp;
    }
}
