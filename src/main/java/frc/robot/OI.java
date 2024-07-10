package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/**
 * OI - Use this class to access and initialize all controller-related stuff.
 */
public class OI {
  /**
   * Checks if the pilot is moving the robot.
   * 
   * @return true if the pilot is moving the robot, false otherwise.
   */
  public static boolean pilotIsActive() {
    return Math.abs(pilot.getLeftY()) > 0.1 ||
        Math.abs(pilot.getRightY()) > 0.1 ||
        Math.abs(pilot.getRightX()) > 0.1;
  }

  /** The pilot's controller */
  public static final CommandXboxController pilot = new CommandXboxController(
      Constants.OperatorConstants.PILOT_XBOX_CONTROLLER_PORT);

  /** The copilot's controller */
  //public static final CommandXboxController copilot = new CommandXboxController(
  //    OIConstants.COPILOT_XBOX_CONTROLLER_PORT);
}
