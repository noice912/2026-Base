package team.gif.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class NEORunAtVelocity extends Command {
    private final double targetRPM;
    private final PIDController pid;

    public NEORunAtVelocity(double targetRPM) {
        super();
        this.targetRPM = targetRPM;
        pid = new PIDController(Constants.NEO_VELOCITY_KP, Constants.NEO_VELOCITY_KI, Constants.NEO_VELOCITY_KD);
        addRequirements(Robot.neoMotor);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(targetRPM);
    }

    @Override
    public void execute() {
        double feedforward = targetRPM * Constants.NEO_VELOCITY_KV;

        double correction = pid.calculate(Robot.neoMotor.getVelocity());

        Robot.neoMotor.setVoltage(feedforward + correction);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.neoMotor.setVoltage(0);
    }
}
