package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

/**
 * Objective 1.12 - 4 motor differential (tank-style) drivetrain, 2 motors per side
 */
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

        // The rear motors copy whatever their front motor on the same side does
        leftRear.follow(leftFront);
        rightRear.follow(rightFront);

        // The right side motors face the opposite direction, so invert them to make "forward" match the left side
        rightFront.setInverted(true);
        leftRear.setInverted(InvertType.FollowMaster);
        rightRear.setInverted(InvertType.FollowMaster);

        // DifferentialDrive only needs to command the two front (leader) motors
        drive = new DifferentialDrive(
                speed -> leftFront.set(ControlMode.PercentOutput, speed),
                speed -> rightFront.set(ControlMode.PercentOutput, speed));
    }

    /**
     * Drives the robot using arcade drive (one value for forward/backward, one for turning)
     * @param forward -1.0 (backward) to 1.0 (forward)
     * @param rotation -1.0 (turn right) to 1.0 (turn left)
     */
    public void arcadeDrive(double forward, double rotation) {
        drive.arcadeDrive(forward, rotation);
    }
}
