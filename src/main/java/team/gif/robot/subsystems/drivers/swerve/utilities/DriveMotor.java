package team.gif.robot.subsystems.drivers.swerve.utilities;

public interface DriveMotor {
    void configure(boolean inverted);

    double getTemp();

    double getVelocity();

    double getPosition();

    double getOutput();

    void set(double percentOutput);

    void setVoltage(double voltage);

    double getVoltage();

    void resetEncoder();

}
