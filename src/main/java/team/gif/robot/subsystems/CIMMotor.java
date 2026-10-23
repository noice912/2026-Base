package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

public class CIMMotor extends SubsystemBase {
    private final TalonSRX motor;

    public CIMMotor() {
        motor = new TalonSRX(RobotMap.CIM_MOTOR_ID);
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void setPercent(double percent) {
        if (Robot.limitSwitch.isPressed()) {
            percent = percent * Constants.CIM_LIMIT_SWITCH_SCALE;
        }

        motor.set(ControlMode.PercentOutput, percent);
    }
}
