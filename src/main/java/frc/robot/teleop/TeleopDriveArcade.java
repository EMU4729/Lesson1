package frc.robot.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;

/**
 * The Arcade Teleop command
 */
public class TeleopDriveArcade extends Command {

  public TeleopDriveArcade() {
  }

  public TeleopDriveArcade(double[][] settings) {

    addRequirements(Subsystems.diffDrive);
  }

  @Override
  public void execute() {
    double throttle = 0;
    double steering = 0;

    throttle = OI.pilot.getLeftY();
    steering = OI.pilot.getRightX();
    
    Subsystems.diffDrive.arcade(throttle, steering);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}