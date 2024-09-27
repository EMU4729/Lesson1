package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto extends SequentialCommandGroup {
    public Auto(AutoAction[] actions) {
        Command[] commands = new Command[actions.length];
        for (int i = 0; i < actions.length; i++) {
            commands[i] = actions[i].getCommand();
        }
        addCommands(commands);
    }
}
