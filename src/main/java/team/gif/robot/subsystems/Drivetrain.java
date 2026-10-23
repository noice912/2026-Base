package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
    private final TalonSRX leftFront;
    private final TalonSRX leftRear;
    private final TalonSRX rightFront;
    private final TalonSRX rightRear;
    private final DifferentialDrive drive;

    public Drivetrain() {
        leftFront = new TalonSRX(RobotMap.DRIVE_LEFT_FRONT_ID);
        leftRear = new TalonSRX(RobotMap.DRIVE_LEFT_REAR_ID);
        rightFront = new TalonSRX(RobotMap.DRIVE_RIGHT_FRONT_ID);
        rightRear = new TalonSRX(RobotMap.DRIVE_RIGHT_REAR_ID);

        for (TalonSRX motor : new TalonSRX[]{leftFront, leftRear, rightFront, rightRear}) {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
        }

        leftRear.follow(leftFront);
        rightRear.follow(rightFront);

        rightFront.setInverted(true);
        leftRear.setInverted(InvertType.FollowMaster);
        rightRear.setInverted(InvertType.FollowMaster);

        drive = new DifferentialDrive(
                speed -> leftFront.set(ControlMode.PercentOutput, speed),
                speed -> rightFront.set(ControlMode.PercentOutput, speed));
    }

    public void arcadeDrive(double forward, double rotation) {
        drive.arcadeDrive(forward, rotation);
    }
}
