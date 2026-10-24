package team.gif.robot.subsystems.drivers.swerve.utilities;

public interface Encoder {

    void configure();

    double getTicks();

    double getDegrees();

    double getRadians();

    double getVelocity();

    void reset();

}
