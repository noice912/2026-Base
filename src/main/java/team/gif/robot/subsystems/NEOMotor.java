package team.gif.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
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
    private final RelativeEncoder encoder;

    public NEOMotor() {

        motor = new SparkMax(RobotMap.NEO_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motorConfig.inverted(true);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // The NEO's built-in encoder, counts in motor rotations. Start at 0 when the robot code boots.
        encoder = motor.getEncoder();
        encoder.setPosition(0);
    }

    /**
     * Runs the motor using percent output
     * @param percent -1.0 (full backward) to 1.0 (full forward), 0 stops the motor
     */
    public void setPercent(double percent) {
        motor.set(percent);
    }

    /**
     * @return the motor's position in rotations
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * @return the motor's velocity in RPM (rotations per minute)
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }


    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

}
