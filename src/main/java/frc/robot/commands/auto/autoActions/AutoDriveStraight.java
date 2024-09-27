package frc.robot.commands.auto.autoActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoAction;
import frc.robot.commands.auto.autoCommands.AutoDriveStraightCommand;

public class AutoDriveStraight implements AutoAction {

    private double timeout;
    private double distance;
    private double throttle;

    public AutoDriveStraight(double distance, double throttle, double timeout) {
        this.distance = distance;
        this.throttle = throttle;
        this.timeout = timeout;
    }

    @Override
    public Command getCommand() {
        return new AutoDriveStraightCommand(distance, timeout, throttle);
    }
    
}
