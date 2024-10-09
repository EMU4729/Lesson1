package frc.robot.commands.auto.autoFactories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.commands.auto.autoCommands.AutoTurnCommand;

public class AutoTurn implements AutoFactory {

    private double timeout;
    private Rotation2d targetAngle;

    public AutoTurn(Rotation2d targetAngle, double timeout) {
        this.targetAngle = targetAngle;
        this.timeout = timeout;
    }
    
    @Override
    public Command getCommand() {
        return new AutoTurnCommand(targetAngle, timeout);
    }
    
}
