package team.gif.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class NEOGoToPosition extends Command {
    private final double setpoint;
    private final PIDController pid;

    public NEOGoToPosition(double setpoint) {
        super();
        this.setpoint = setpoint;
        pid = new PIDController(Constants.NEO_POSITION_KP, Constants.NEO_POSITION_KI, Constants.NEO_POSITION_KD);
        addRequirements(Robot.neoMotor);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        double output = pid.calculate(Robot.neoMotor.getPosition());

        output = MathUtil.clamp(output, -Constants.NEO_POSITION_MAX_OUTPUT, Constants.NEO_POSITION_MAX_OUTPUT);

        Robot.neoMotor.setPercent(output);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.neoMotor.setPercent(0);
    }
}
