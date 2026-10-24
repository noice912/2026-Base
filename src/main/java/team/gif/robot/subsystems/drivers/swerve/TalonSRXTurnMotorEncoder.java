package team.gif.robot.subsystems.drivers.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import team.gif.robot.subsystems.drivers.swerve.utilities.Encoder;
import team.gif.robot.subsystems.drivers.swerve.utilities.TurnMotor;

public class TalonSRXTurnMotorEncoder implements TurnMotor, Encoder {
    private final TalonSRX motor;

    public TalonSRXTurnMotorEncoder(int id) {
        motor = new TalonSRX(id);
    }

    public void configure(boolean inverted) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(inverted);
    }

    public void configure() {
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        motor.configFeedbackNotContinuous(true, 0);
        motor.configSelectedFeedbackCoefficient(1);
        motor.configVoltageCompSaturation(12);
        motor.enableVoltageCompensation(true);
    }

    public double getOutput() {
        return motor.getMotorOutputPercent();
    }

    public double getVoltage() {
        return motor.getMotorOutputVoltage();
    }

    public void set(double percentOutput) {
        motor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setVoltage(double voltage) {
        motor.set(ControlMode.PercentOutput, voltage / motor.getBusVoltage());
    }

    public double getTicks() {
        return motor.getSelectedSensorPosition();
    }

    public double getDegrees() {
        return motor.getSelectedSensorPosition() / 4096 * 360 * -1;
    }

    public double getRadians() {
        return motor.getSelectedSensorPosition() / 4096 * 2 * Math.PI * -1;
    }

    public double getVelocity() {
        return 0;
    }

    public void reset() {
        motor.setSelectedSensorPosition(0);
    }
}
