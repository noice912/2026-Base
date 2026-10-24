package team.gif.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.robot.subsystems.CIMMotor;
import team.gif.robot.subsystems.LimitSwitch;
import team.gif.robot.subsystems.drivers.Pigeon;

public class Robot extends TimedRobot {
    private static Command autonomousCommand;
    private RobotContainer robotContainer;
    public static OI oi;

    public static Pigeon pigeon;
    public static LimitSwitch limitSwitch;
    public static CIMMotor cimMotor;
    private boolean lastLimitSwitchState;

    public static UI ui;

    public Robot() {
        robotContainer = new RobotContainer();

        pigeon = new Pigeon(RobotMap.PIGEON_ID);

        limitSwitch = new LimitSwitch();
        lastLimitSwitchState = limitSwitch.isPressed();
        System.out.println("Limit switch pressed: " + lastLimitSwitchState);

        cimMotor = new CIMMotor();

        oi = new OI();
        ui = new UI();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        ui.update();

        System.out.println(pigeon.get180Heading());

        boolean limitSwitchState = limitSwitch.isPressed();
        if (limitSwitchState != lastLimitSwitchState) {
            System.out.println("Limit switch pressed: " + limitSwitchState);
            lastLimitSwitchState = limitSwitchState;
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
