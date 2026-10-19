package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

/**
 * NEO motor driven by a SparkMAX.
 *
 * Our SparkMAX has 2023 firmware, which REVLib 2026 refuses to talk to over CAN
 * ("The SPARK firmware is too old and needs to be updated"). So instead of REVLib, the
 * motor is controlled with a PWM signal from the roboRIO, which works with any firmware.
 *
 * With PWM, brake/coast mode is set with the button on the SparkMAX, not in code.
 */
public class NEOMotor extends SubsystemBase {
    private final PWMSparkMax motor;

    public NEOMotor() {
        motor = new PWMSparkMax(RobotMap.NEO_PWM_PORT);
    }

    /**
     * Runs the motor using voltage control. WPILib measures the battery voltage and
     * adjusts the output so the motor gets this many volts.
     * @param volts -12 (full backward) to 12 (full forward), 0 stops the motor
     */
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
