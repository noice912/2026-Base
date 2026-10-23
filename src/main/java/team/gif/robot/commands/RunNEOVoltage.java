package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class RunNEOVoltage extends Command {


    public RunNEOVoltage() {
        super();

        addRequirements(Robot.neoMotor);
    }

    
    @Override
    public void initialize() {}


    @Override
    public void execute() {
        Robot.neoMotor.setVoltage(Constants.SPARK_MOTOR_VOLTAGE);
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
