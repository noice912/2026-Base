package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class LimitSwitch extends SubsystemBase {
    private final DigitalInput limitSwitch;


    public LimitSwitch() {
        limitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH_ID);
    }


    public boolean isPressed() {
        return limitSwitch.get();
    }
}
