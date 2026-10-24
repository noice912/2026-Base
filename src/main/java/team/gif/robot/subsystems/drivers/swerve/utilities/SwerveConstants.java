package team.gif.robot.subsystems.drivers.swerve.utilities;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.robot.subsystems.drivers.swerve.SwerveDrivetrain;

public abstract class SwerveConstants {
    public double MODULE_GEAR_RATIO = 1;
    public double TURNING_MOTOR_GEAR_RATIO = 1;
    public double WHEEL_DIAMETER_INCHES = 4;
    public double DRIVE_ENCODER_CPR = 1;
    public double COEFFICIENT_OF_FRICTION = 1;

    public double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 5;

    public double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;

    public double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 10;

    public double MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND_SQUARED = 5 * Math.PI;

    public boolean FL_DRIVE_INVERTED = true;
    public boolean FR_DRIVE_INVERTED = true;
    public boolean RL_DRIVE_INVERTED = true;
    public boolean RR_DRIVE_INVERTED = true;

    public boolean FL_TURN_INVERTED = true;
    public boolean FR_TURN_INVERTED = true;
    public boolean RL_TURN_INVERTED = true;
    public boolean RR_TURN_INVERTED = true;

    public DCMotor PATHPLANNER_MOTOR_TYPE;
    public int PATHPLANNER_CURRENT_LIMIT;

    public double MAX_DRIVE_TEMP =  85;

    public double WHEEL_DIAMETER_METERS;

    public double DRIVE_ENCODER_ROT_2_METER;
    public double DRIVE_ENCODER_RPM_2_METER_PER_SEC;
    public double TURNING_ENCODER_ROT_TO_RAD;
    public double TURNING_ENCODER_RPM_2_RAD_PER_SECOND;
    public ModuleConfig PATHPLANNER_MODULE_CONFIG;

    public double FRONT_LEFT_OFFSET = 0;
    public double FRONT_RIGHT_OFFSET = 0;
    public double REAR_LEFT_OFFSET = 0;
    public double REAR_RIGHT_OFFSET = 0;
    public double TRACK_LENGTH_INCHES;
    public double TRACK_WIDTH_INCHES;
    public double COAST_DRIVE_PERC = 0.6;
    public double BOOST_DRIVE_PERC = 1;
    public double SLOW_DRIVE_PERC = 0.3;
    public double MASS_KG;
    public double MOI_KGM2;

    public double TRACK_LENGTH_METERS;
    public double TRACK_WIDTH_METERS;
    public SwerveDriveKinematics DRIVE_KINEMATICS;
    public double SLOW_SPEED_METERS_PER_SECOND;
    public double COAST_SPEED_METERS_PER_SECOND;
    public double BOOST_SPEED_METERS_PER_SECOND;

    public double TURN_P;
    public double TURN_FF;
    public SimpleMotorFeedforward FL_DRIVE_FF;
    public SimpleMotorFeedforward FR_DRIVE_FF;
    public SimpleMotorFeedforward RL_DRIVE_FF;
    public SimpleMotorFeedforward RR_DRIVE_FF;
    public double AUTO_P_FORWARD = 2.5;
    public double AUTO_P_ROTATION = 2.5;

    public double FL_P = Double.NaN;
    public double FR_P = Double.NaN;
    public double RL_P = Double.NaN;
    public double RR_P = Double.NaN;
    public double FL_FF = Double.NaN;
    public double FR_FF = Double.NaN;
    public double RL_FF = Double.NaN;
    public double RR_FF = Double.NaN;
    public PPHolonomicDriveController AUTO_DRIVE_CONTROLLER;

    public SwerveConstants() {
        setConfiguration();
        calculate();
    }

    protected abstract void setConfiguration();

    private void calculate() {
        WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
        DRIVE_ENCODER_ROT_2_METER = (Math.PI * WHEEL_DIAMETER_METERS) / (MODULE_GEAR_RATIO * DRIVE_ENCODER_CPR);
        DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
        TURNING_ENCODER_RPM_2_RAD_PER_SECOND = TURNING_ENCODER_ROT_TO_RAD / 60;

        TRACK_LENGTH_METERS = Units.inchesToMeters(TRACK_LENGTH_INCHES);
        TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH_INCHES);
        DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(TRACK_LENGTH_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(TRACK_LENGTH_METERS / 2, -TRACK_WIDTH_METERS / 2),
                new Translation2d(-TRACK_LENGTH_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(-TRACK_LENGTH_METERS / 2, -TRACK_WIDTH_METERS / 2)
        );
        SLOW_SPEED_METERS_PER_SECOND = SLOW_DRIVE_PERC * PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        COAST_SPEED_METERS_PER_SECOND = COAST_DRIVE_PERC * PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        BOOST_SPEED_METERS_PER_SECOND = BOOST_DRIVE_PERC * PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

        FL_P = Double.isNaN(FL_P) ? TURN_P : FL_P;
        FR_P = Double.isNaN(FR_P) ? TURN_P : FR_P;
        RL_P = Double.isNaN(RL_P) ? TURN_P : RL_P;
        RR_P = Double.isNaN(RR_P) ? TURN_P : RR_P;
        FL_FF = Double.isNaN(FL_FF) ? TURN_FF : FL_FF;
        FR_FF = Double.isNaN(FR_FF) ? TURN_FF : FR_FF;
        RL_FF = Double.isNaN(RL_FF) ? TURN_FF : RL_FF;
        RR_FF = Double.isNaN(RR_FF) ? TURN_FF : RR_FF;
        AUTO_DRIVE_CONTROLLER = new PPHolonomicDriveController(
                new PIDConstants(AUTO_P_FORWARD, 0.0, 0.0),
                new PIDConstants(AUTO_P_ROTATION, 0.0, 0.0)
        );

        PATHPLANNER_MODULE_CONFIG = new ModuleConfig(
                WHEEL_DIAMETER_METERS,
                PHYSICAL_MAX_SPEED_METERS_PER_SECOND,
                COEFFICIENT_OF_FRICTION,
                PATHPLANNER_MOTOR_TYPE,
                PATHPLANNER_CURRENT_LIMIT,
                1
        );
    }
}