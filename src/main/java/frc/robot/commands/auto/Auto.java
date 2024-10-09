package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto extends SequentialCommandGroup {
    public Auto(AutoFactory[] steps) {
        Command[] commands = new Command[steps.length];
        for (int i = 0; i < steps.length; i++) {
            commands[i] = steps[i].getCommand();
        }
        addCommands(commands);
    }
}
