package team.gif.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

/**
 * NEO motor driven by a SparkMAX
 */
public class NEOMotor extends SubsystemBase {
    private final SparkMax motor;

    public NEOMotor() {
        motor = new SparkMax(RobotMap.NEO_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Runs the motor using voltage control
     * @param volts -12 (full backward) to 12 (full forward), 0 stops the motor
     */
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
