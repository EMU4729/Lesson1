package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Drive Subsystem.
 * Handles all drive functionality.
 */
public class DifferentialDriveSub extends SubsystemBase {
  //TODO initialise the motors

  public DifferentialDriveSub() {
    //TODO make two motors on each side run as a pair
  }

  @Override
  public void periodic() {  }

  /**
   * Arcade drive.
   * 
   * @param throttle The speed
   * @param steering The steering
   */
  public void arcade(double throttle, double steering) {
    // TODO take the throttle and steering and calc the power for each side (tank drive)
  }

  /** Stops all motors. */
  public void off() {
    // TODO make this function stop the motors
  }

}