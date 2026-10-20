package team.gif.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class NEOGoToPosition extends Command {
    private final double setpoint;
    private final PIDController pid;

    /**
     * Objective 1.8 - Drives the NEO to a position using a PID controller, then holds it there
     * @param setpoint target position in motor rotations
     */
    public NEOGoToPosition(double setpoint) {
        super();
        this.setpoint = setpoint;
        pid = new PIDController(Constants.NEO_POSITION_KP, Constants.NEO_POSITION_KI, Constants.NEO_POSITION_KD);
        addRequirements(Robot.neoMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(setpoint);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // PID output is based on how far the motor is from the setpoint
        double output = pid.calculate(Robot.neoMotor.getPosition());

        // Limit the speed so the motor doesn't slam to the setpoint
        output = MathUtil.clamp(output, -Constants.NEO_POSITION_MAX_OUTPUT, Constants.NEO_POSITION_MAX_OUTPUT);

        Robot.neoMotor.setPercent(output);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false; // keep holding the position until another NEO command takes over
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.neoMotor.setPercent(0);
    }
}
