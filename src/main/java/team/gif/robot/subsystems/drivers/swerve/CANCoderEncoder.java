package team.gif.robot.subsystems.drivers.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import team.gif.robot.subsystems.drivers.swerve.utilities.Encoder;

public class CANCoderEncoder implements Encoder {
    private final CANcoder encoder;

    public CANCoderEncoder(int id) {
        encoder = new CANcoder(id);
    }

    public void configure() {
        MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(0.5);
        encoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(encoderConfig));
    }

    public double getTicks() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getDegrees() {
        return getTicks() * 360;
    }

    public double getRadians() {
        return getTicks() * 2 * Math.PI;
    }

    public double getVelocity() {
        return encoder.getVelocity().getValueAsDouble();
    }

    public void reset() {
        encoder.setPosition(0);
    }
}
