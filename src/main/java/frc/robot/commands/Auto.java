package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto extends SequentialCommandGroup {
    public Auto(double distance, double timeout, double throttle) {

        addCommands(
            new DriveDistanceCommand(distance, timeout, throttle)
        );
    }
}
