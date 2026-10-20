package team.gif.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

/**
 * NEO motor driven by a SparkMAX, configured following
 * https://docs.revrobotics.com/revlib/spark/configuring-a-spark
 */
public class NEOMotor extends SubsystemBase {
    private final SparkMax motor;

    public NEOMotor() {
        // The motor type is set here, not in the config. A NEO is brushless,
        // and running a brushless motor in brushed mode can permanently damage it.
        motor = new SparkMax(RobotMap.NEO_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);

        // Reset to safe defaults, then apply our settings. Persist them so the SparkMAX
        // keeps this configuration after a power cycle (brownout or breaker trip).
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Runs the motor using voltage control
     * @param volts -12 (full backward) to 12 (full forward), 0 stops the motor
     */
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
