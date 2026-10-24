package team.gif.robot.subsystems.drivers.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.lib.LimelightHelpers;
import team.gif.lib.drivePace;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.drivers.swerve.utilities.DriveMotor;
import team.gif.robot.subsystems.drivers.swerve.utilities.DriveMotorFactory;
import team.gif.robot.subsystems.drivers.swerve.utilities.Encoder;
import team.gif.robot.subsystems.drivers.swerve.utilities.EncoderFactory;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConfiguration;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConstants;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveMap;
import team.gif.robot.subsystems.drivers.swerve.utilities.TurnMotor;
import team.gif.robot.subsystems.drivers.swerve.utilities.TurnMotorFactory;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;


public class SwerveDrivetrain extends SubsystemBase {
    private final SwerveMap deviceMap;
    private final SwerveConstants constants;
    private final DriveMotorFactory driveMotorFactory;
    private final TurnMotorFactory turnMotorFactory;
    private final EncoderFactory encoderFactory;

    public SwerveModule fL;
    public SwerveModule fR;
    public SwerveModule rL;
    public SwerveModule rR;

    private DriveMotor fLDriveMotor;
    private DriveMotor fRDriveMotor;
    private DriveMotor rLDriveMotor;
    private DriveMotor rRDriveMotor;

    private TurnMotor fLTurnMotor;
    private TurnMotor fRTurnMotor;
    private TurnMotor rLTurnMotor;
    private TurnMotor rRTurnMotor;

    private Encoder fLEncoder;
    private Encoder fREncoder;
    private Encoder rLEncoder;
    private Encoder rREncoder;

    public SwerveDrivePoseEstimator poseEstimator;
    private drivePace drivePace;

    public boolean limelightEnabled = true;
    public String[] limelightNames = new String[] {};
    
    public boolean debugMode = false;

    private static final StructArrayPublisher<SwerveModuleState> targetPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("TargetSwerveState", SwerveModuleState.struct).publish();
    private static final StructArrayPublisher<SwerveModuleState> actualPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("ActualSwerveState", SwerveModuleState.struct).publish();
    private static final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("EstimatedPose", Pose2d.struct).publish();
    private static final StructPublisher<ChassisSpeeds> chassisSpeedsStructPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("ChassisSpeeds", ChassisSpeeds.struct).publish();


    public SwerveDrivetrain(SwerveConfiguration config) {
        super();

        this.deviceMap = config.deviceMap;
        this.constants = config.constants;
        this.driveMotorFactory = config.driveMotorFactory;
        this.turnMotorFactory = config.turnMotorFactory;
        this.encoderFactory = config.encoderFactory;

        configModules();

        resetDriveEncoders();

        poseEstimator = new SwerveDrivePoseEstimator(constants.DRIVE_KINEMATICS, Robot.pigeon.getRotation2d(), getPosition(), new Pose2d(0, 0, new Rotation2d(0)));

        drivePace = team.gif.lib.drivePace.COAST_FR;

        configPathPlanner();
    }

    @Override
    public void periodic() {
        poseEstimator.update(
            Robot.pigeon.getRotation2d(),
            getPosition()
        );

        if (debugMode) {
            posePublisher.set(poseEstimator.getEstimatedPosition());
            updateShuffleboardDebug();
        }
    }

    public void setLimelightEnabled(boolean enabled) {
        limelightEnabled = enabled;
    }

    public boolean getLimelightEnabled() {
        return limelightEnabled;
    }

    public void addLimelight(String newLimelightName) {
        String[] newArray = new String[limelightNames.length + 1];
        System.arraycopy(limelightNames, 0, newArray, 0, limelightNames.length);
        newArray[newArray.length - 1] = newLimelightName;
        limelightNames = newArray;
    }

    public String[] getLimelightNames() {
        return limelightNames;
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(Robot.pigeon.getRotation2d(), new SwerveModulePosition[]{fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()}, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeed() {
        SwerveModuleState frontLeftState = new SwerveModuleState(fL.getDriveVelocity(), Rotation2d.fromDegrees(fL.getTurningHeadingDegrees()));
        SwerveModuleState frontRightState = new SwerveModuleState(fR.getDriveVelocity(), Rotation2d.fromDegrees(fR.getTurningHeadingDegrees()));
        SwerveModuleState rearLeft = new SwerveModuleState(rL.getDriveVelocity(), Rotation2d.fromDegrees(rL.getTurningHeadingDegrees()));
        SwerveModuleState rearRight = new SwerveModuleState(rR.getDriveVelocity(), Rotation2d.fromDegrees(rR.getTurningHeadingDegrees()));

        ChassisSpeeds speed = constants.DRIVE_KINEMATICS.toChassisSpeeds(frontLeftState, frontRightState, rearLeft, rearRight);

        if (debugMode) {
            chassisSpeedsStructPublisher.set(speed);
        }

        return speed;
    }

    public void drive(double x, double y, double rot) {
        SwerveModuleState[] swerveModuleStates =
                constants.DRIVE_KINEMATICS.toSwerveModuleStates(
                        drivePace.getIsFieldRelative() ?
                                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, Robot.pigeon.getRotation2d())
                                : new ChassisSpeeds(x, y, rot));
        if (debugMode) {
            SwerveModuleState[] actualStates = { fL.getState(), fR.getState(), rL.getState(), rR.getState()};
            targetPublisher.set(swerveModuleStates);
            actualPublisher.set(actualStates);
        }
        setModuleStates(swerveModuleStates);
    }

    public void setMaxDrive() {
        fLDriveMotor.set(1);
        fRDriveMotor.set(1);
        rLDriveMotor.set(1);
        rRDriveMotor.set(1);
    }

    public void stopDrive() {
        drive(0,0,0);
    }

     public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, drivePace.getValue()
        );

        fL.setDesiredState(desiredStates[0]);
        fR.setDesiredState(desiredStates[1]);
        rL.setDesiredState(desiredStates[2]);
        rR.setDesiredState(desiredStates[3]);
    }

    public void setModuleChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = constants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, drivePace.getValue()
        );

        for (SwerveModuleState state : swerveModuleStates) {
            state.speedMetersPerSecond = Math.min(state.speedMetersPerSecond, drivePace.getValue());
        }

        fL.setDesiredState(swerveModuleStates[0]);
        fR.setDesiredState(swerveModuleStates[1]);
        rL.setDesiredState(swerveModuleStates[2]);
        rR.setDesiredState(swerveModuleStates[3]);

        if(debugMode) {
            SwerveModuleState[] actualStates = { fL.getState(), fR.getState(), rL.getState(), rR.getState()};
            actualPublisher.set(actualStates);
            chassisSpeedsStructPublisher.set(chassisSpeeds);
            targetPublisher.set(swerveModuleStates);
        }
    }

    public void modulesTo90() {
        SwerveModuleState state90 = new SwerveModuleState(0, Rotation2d.fromDegrees(90));
        fL.setDesiredState(state90, true);
        fR.setDesiredState(state90, false);
        rL.setDesiredState(state90, true);
        rR.setDesiredState(state90, false);

    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void stopModules() {
        fL.stop();
        fR.stop();
        rR.stop();
        rL.stop();
    }

    public SwerveModulePosition[] getPosition() {

        return new SwerveModulePosition[]{fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()};
    }

    public void resetDriveEncoders() {
        fL.resetDriveEncoders();
        fR.resetDriveEncoders();
        rL.resetDriveEncoders();
        rR.resetDriveEncoders();
    }

    public void setDrivePace(drivePace drivePace) {
        this.drivePace = drivePace;
    }

    public drivePace getDrivePace() {
        return drivePace;
    }

    public double getPoseX() {
        return getPose().getX();
    }

    public double getPoseY() {
        return getPose().getY();
    }

    public SwerveConstants getConstants() {
        return constants;
    }

    private void configModules() {

        fLDriveMotor = driveMotorFactory.create(deviceMap.FRONT_LEFT_DRIVE_MOTOR_ID, constants);
        fRDriveMotor = driveMotorFactory.create(deviceMap.FRONT_RIGHT_DRIVE_MOTOR_ID, constants);
        rLDriveMotor = driveMotorFactory.create(deviceMap.REAR_LEFT_DRIVE_MOTOR_ID, constants);
        rRDriveMotor = driveMotorFactory.create(deviceMap.REAR_RIGHT_DRIVE_MOTOR_ID, constants);

        fLTurnMotor = turnMotorFactory.create(deviceMap.FRONT_LEFT_TURNING_MOTOR_ID);
        fRTurnMotor = turnMotorFactory.create(deviceMap.FRONT_RIGHT_TURNING_MOTOR_ID);
        rLTurnMotor = turnMotorFactory.create(deviceMap.REAR_LEFT_TURNING_MOTOR_ID);
        rRTurnMotor = turnMotorFactory.create(deviceMap.REAR_RIGHT_TURNING_MOTOR_ID);

        if(encoderFactory == null && fLTurnMotor instanceof TalonSRXTurnMotorEncoder) {
            fLEncoder = (Encoder) fLTurnMotor;
            fREncoder = (Encoder) fRTurnMotor;
            rLEncoder = (Encoder) rLTurnMotor;
            rREncoder = (Encoder) rRTurnMotor;
        } else {
            assert encoderFactory != null;
            fLEncoder = encoderFactory.create(deviceMap.FRONT_LEFT_ENCODER_ID);
            fREncoder = encoderFactory.create(deviceMap.FRONT_RIGHT_ENCODER_ID);
            rLEncoder = encoderFactory.create(deviceMap.REAR_LEFT_ENCODER_ID);
            rREncoder = encoderFactory.create(deviceMap.REAR_RIGHT_ENCODER_ID);
        }


        fL = new SwerveModule(
                fLDriveMotor,
                fLTurnMotor,
                fLEncoder,
                constants.FL_TURN_INVERTED,
                constants.FL_DRIVE_INVERTED,
                constants.FRONT_LEFT_OFFSET,
                constants.FL_DRIVE_FF,
                constants.FL_FF,
                constants.FL_P,
                constants.MAX_DRIVE_TEMP
        );

        fR = new SwerveModule (
                fRDriveMotor,
                fRTurnMotor,
                fREncoder,
                constants.FR_TURN_INVERTED,
                constants.FR_DRIVE_INVERTED,
                constants.FRONT_RIGHT_OFFSET,
                constants.FR_DRIVE_FF,
                constants.FR_FF,
                constants.FR_P,
                constants.MAX_DRIVE_TEMP
                );

        rL = new SwerveModule (
                rLDriveMotor,
                rLTurnMotor,
                rLEncoder,
                constants.RL_TURN_INVERTED,
                constants.RL_DRIVE_INVERTED,
                constants.REAR_LEFT_OFFSET,
                constants.RL_DRIVE_FF,
                constants.RL_FF,
                constants.RL_P,
                constants.MAX_DRIVE_TEMP
        );

        rR = new SwerveModule (
                rRDriveMotor,
                rRTurnMotor,
                rREncoder,
                constants.RR_TURN_INVERTED,
                constants.RR_DRIVE_INVERTED,
                constants.REAR_RIGHT_OFFSET,
                constants.RR_DRIVE_FF,
                constants.RR_FF,
                constants.RR_P,
                constants.MAX_DRIVE_TEMP
        );
    }

    private void configPathPlanner() {
        RobotConfig ppConfig;
        try{
            ppConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            ModuleConfig moduleConfig = constants.PATHPLANNER_MODULE_CONFIG;
            ppConfig = new RobotConfig(constants.MASS_KG, constants.MOI_KGM2, moduleConfig, constants.TRACK_WIDTH_METERS);
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeed,
                this::setModuleChassisSpeeds,
                constants.AUTO_DRIVE_CONTROLLER,
                ppConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if( alliance.isPresent() ){
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );
    }

    public double fLDriveTemp() { return fLDriveMotor.getTemp(); }
    public double fRDriveTemp() { return fRDriveMotor.getTemp(); }
    public double rLDriveTemp() { return rLDriveMotor.getTemp(); }
    public double rRDriveTemp() { return rRDriveMotor.getTemp(); }

    public void enableDebugMode() {
        debugMode = true;
        SmartDashboard.putData("Swerve/FL Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", fL::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData("Swerve/FR Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", fR::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData("Swerve/RL Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", rL::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData("Swerve/RR Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", rR::getTurningHeadingDegrees, null);
        });

        SmartDashboard.putData("Swerve/Actual Swerve State", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> fL.getTurningHeading(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> fL.getDriveVelocity(), null);

                builder.addDoubleProperty("Front Right Angle", () -> fR.getTurningHeading(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> fR.getDriveVelocity(), null);

                builder.addDoubleProperty("Back Left Angle", () -> rL.getTurningHeading(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> rL.getDriveVelocity(), null);

                builder.addDoubleProperty("Back Right Angle", () -> rR.getTurningHeading(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> rR.getDriveVelocity(), null);

                builder.addDoubleProperty("Robot Angle", () -> Units.degreesToRadians(Robot.pigeon.get360Heading()), null);
            }
        });

        SmartDashboard.putData("Swerve/Target Swerve State", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", fL::getTargetAngle, null);
                builder.addDoubleProperty("Front Left Velocity", fL::getTargetVelocity, null);

                builder.addDoubleProperty("Front Right Angle", fR::getTargetAngle, null);
                builder.addDoubleProperty("Front Right Velocity", fR::getTargetVelocity, null);

                builder.addDoubleProperty("Back Left Angle", rL::getTargetAngle, null);
                builder.addDoubleProperty("Back Left Velocity", rL::getTargetVelocity, null);

                builder.addDoubleProperty("Back Right Angle", rR::getTargetAngle, null);
                builder.addDoubleProperty("Back Right Velocity", rR::getTargetVelocity, null);

                builder.addDoubleProperty("Robot Angle", () -> Units.degreesToRadians(Robot.pigeon.get360Heading()), null);
            }
        });
    }
    private void updateShuffleboardDebug() {
        SmartDashboard.putNumber("Swerve/FR Raw Degrees", fR.encoderDegrees());
        SmartDashboard.putNumber("Swerve/FL Raw Degrees", fL.encoderDegrees());
        SmartDashboard.putNumber("Swerve/RR Raw Degrees", rR.encoderDegrees());
        SmartDashboard.putNumber("Swerve/RL Raw Degrees", rL.encoderDegrees());

        SmartDashboard.putNumber("Swerve/FL Raw Encoder", fLEncoder.getTicks());
        SmartDashboard.putNumber("Swerve/FR Raw Encoder", fREncoder.getTicks());
        SmartDashboard.putNumber("Swerve/RL Raw Encoder", rLEncoder.getTicks());
        SmartDashboard.putNumber("Swerve/RR Raw Encoder", rREncoder.getTicks());

        SmartDashboard.putNumber("Swerve/FR Raw Radians", fR.getTurningHeading());
        SmartDashboard.putNumber("Swerve/FL Raw Radians", fL.getTurningHeading());
        SmartDashboard.putNumber("Swerve/RR Raw Radians", rR.getTurningHeading());
        SmartDashboard.putNumber("Swerve/RL Raw Radians", rL.getTurningHeading());

        SmartDashboard.putNumber("Swerve/FL Drive Encoder", fLDriveMotor.getPosition());
        SmartDashboard.putNumber("Swerve/FR Drive Encoder", fRDriveMotor.getPosition());
        SmartDashboard.putNumber("Swerve/RL Drive Encoder", rLDriveMotor.getPosition());
        SmartDashboard.putNumber("Swerve/RR Drive Encoder", rRDriveMotor.getPosition());

        SmartDashboard.putNumber("Swerve/FL Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber("Swerve/FR Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber("Swerve/RL Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber("Swerve/RR Rot Output", fLTurnMotor.getOutput());
    }

    public SysIdRoutine getSysIdRoutine(String motors) {
        MutVoltage voltMut = Volts.mutable(0);


        if (motors.equals("drive")) {
            MutDistance posMut = Meters.mutable(0);
            MutLinearVelocity vMut= MetersPerSecond.mutable(0);

            return new SysIdRoutine(new SysIdRoutine.Config(null, voltMut.mut_replace(5, Volts), null),
                    new SysIdRoutine.Mechanism(
                            voltage -> {
                                fLDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                fRDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                rLDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                rRDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                            },
                            log -> {
                                log.motor("fLDrive")
                                        .voltage(voltMut.mut_replace(fLDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(fLDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(fLDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("fRDrive")
                                        .voltage(voltMut.mut_replace(fRDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(fRDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(fRDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("rLDrive")
                                        .voltage(voltMut.mut_replace(rLDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(rLDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(rLDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("rRDrive")
                                        .voltage(voltMut.mut_replace(rRDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(rRDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(rRDriveMotor.getVelocity(), MetersPerSecond));

                            },
                            this));
        } else if (motors.equals("turn")) {
            MutAngle thetaMut = Radians.mutable(0);
            MutAngularVelocity thetaVMut = RadiansPerSecond.mutable(0);

            return new SysIdRoutine(new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                    voltage -> {
                            fLTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            fRTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            rLTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            rRTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                        }, log -> {
                            log.motor("fLTurn")
                                    .voltage(voltMut.mut_replace(fLTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(fLEncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(fLEncoder.getVelocity(), RadiansPerSecond));
                            log.motor("fRTurn")
                                    .voltage(voltMut.mut_replace(fRTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(fREncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(fREncoder.getVelocity(), RadiansPerSecond));
                            log.motor("rLTurn")
                                    .voltage(voltMut.mut_replace(rLTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(rLEncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(rLEncoder.getVelocity(), RadiansPerSecond));
                            log.motor("rRTurn")
                                    .voltage(voltMut.mut_replace(rRTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(rREncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(rREncoder.getVelocity(), RadiansPerSecond));
                    }, this));

        } else {
            DriverStation.reportError("Invalid motor type at SwerveDrivetrainMk4.getSysIdRoutine", false);
            return null;
        }
    }

    public Command sysIdQuasistatic(String motor, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(motor).quasistatic(direction);
    }

    public Command sysIdDynamic(String motor, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(motor).dynamic(direction);
    }
}