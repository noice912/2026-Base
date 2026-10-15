package team.gif.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class NEORunAtVelocity extends Command {
    private final double targetRPM;
    private final PIDController pid;

    /**
     * Objective 1.9 - Runs the NEO at a velocity (RPM) until the command ends
     * @param targetRPM target speed in rotations per minute
     */
    public NEORunAtVelocity(double targetRPM) {
        super();
        this.targetRPM = targetRPM;
        pid = new PIDController(Constants.NEO_VELOCITY_KP, Constants.NEO_VELOCITY_KI, Constants.NEO_VELOCITY_KD);
        addRequirements(Robot.neoMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(targetRPM);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // Feedforward: the voltage we expect to need for this RPM
        double feedforward = targetRPM * Constants.NEO_VELOCITY_KV;

        // PID: extra voltage to correct for the difference between the target and the actual RPM
        double correction = pid.calculate(Robot.neoMotor.getVelocity());

        Robot.neoMotor.setVoltage(feedforward + correction);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false; // keep running until the button is released
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.neoMotor.setVoltage(0);
    }
}
