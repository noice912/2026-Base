package team.gif.robot.subsystems.drivers.swerve.utilities;

public interface TurnMotor {
    void configure(boolean inverted);

    double getOutput();

    double getVoltage();

    void set(double percentOutput);

    void setVoltage(double voltage);
}
