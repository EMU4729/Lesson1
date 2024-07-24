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

    throttle = OI.pilot.getLeftY()*0.3;
    steering = OI.pilot.getRightX()*0.3;
    steering *= (0.7 + 0.3 * throttle);

    //TODO tackle the following tasks one at a time in order
    //you may make any private functions needed
    //you may google and ask questions as needed.

    //TODO Impliment the diffDrive subsystem and make it drive
    //TODO Should inputs be linear, what would change if they were not
    //TODO How much power will get the robot moving 
    //TODO Can you make the robot jump straight to this power
    //TODO What happens if you set that too high
    //TODO How can you fix this
    //TODO Make the robot turn, now drive at speed and make it turn, what changes
    //TODO How can you fix this problem

    Subsystems.diffDrive.arcade(throttle, steering);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}