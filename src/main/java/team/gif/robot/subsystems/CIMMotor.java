package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

/**
 * CIM motor driven by a TalonSRX
 */
public class CIMMotor extends SubsystemBase {
    private final TalonSRX motor;

    public CIMMotor() {
        motor = new TalonSRX(RobotMap.CIM_MOTOR_ID);
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Runs the motor using percent output
     * @param percent -1.0 (full backward) to 1.0 (full forward), 0 stops the motor
     */
    public void setPercent(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }
}
