package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

/**
 * Piston controlled by a double solenoid on the Pneumatics Control Module (PCM)
 */
public class Piston extends SubsystemBase {
    private final DoubleSolenoid solenoid;

    public Piston() {
        solenoid = new DoubleSolenoid(RobotMap.PCM_ID, PneumaticsModuleType.CTREPCM,
                RobotMap.SOLENOID_FORWARD_CHANNEL, RobotMap.SOLENOID_REVERSE_CHANNEL);

        // A double solenoid starts "off", and toggle() does nothing while it's off,
        // so give it a starting direction
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Switches the solenoid between forward and reverse
     */
    public void toggle() {
        solenoid.toggle();
    }
}
