package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Piston extends SubsystemBase {
    private final DoubleSolenoid solenoid;

    public Piston() {
        solenoid = new DoubleSolenoid(RobotMap.PCM_ID, PneumaticsModuleType.CTREPCM,
                RobotMap.SOLENOID_FORWARD_CHANNEL, RobotMap.SOLENOID_REVERSE_CHANNEL);

        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggle() {
        solenoid.toggle();
    }
}
