package team.gif.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.robot.commands.drivetrain.DriveSwerve;
import team.gif.robot.subsystems.drivers.Pigeon2_0;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConfiguration;
import team.gif.robot.subsystems.drivers.swerve.SwerveDrivetrain;
import team.gif.robot.subsystems.drivers.swerve.TalonFXDriveMotor;
import team.gif.robot.subsystems.drivers.swerve.TalonFXTurnMotor;
import team.gif.robot.subsystems.drivers.swerve.CANCoderEncoder;

public class Robot extends TimedRobot {
    private static Command autonomousCommand;
    private RobotContainer robotContainer;
    public static OI oi;

    public static Pigeon2_0 pigeon;

    public static SwerveConfiguration swerveConfig;
    public static SwerveDrivetrain swerveDrive;

    public static UI ui;

    public static final boolean enableSwerveDebug = true;
    public static final boolean fullDashboard = true;

    public Robot() {
        robotContainer = new RobotContainer();

        pigeon = new Pigeon2_0(RobotMap.PIGEON_ID);

        swerveConfig = new SwerveConfiguration(new RobotMap.Mk4Map(), new Constants.Mk4Constants(), TalonFXDriveMotor::new, TalonFXTurnMotor::new, CANCoderEncoder::new);
        swerveDrive = new SwerveDrivetrain(swerveConfig);
        swerveDrive.setDefaultCommand(new DriveSwerve());
        swerveDrive.enableDebugMode();

        oi = new OI();
        ui = new UI();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        ui.update();
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
