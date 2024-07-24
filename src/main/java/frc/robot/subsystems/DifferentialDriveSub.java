package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Drive Subsystem.
 * Handles all drive functionality.
 */
public class DifferentialDriveSub extends SubsystemBase {
  //TODO initialise the motors (4 of them)
  //private final WPI_TalonSRX name = new WPI_TalonSRX(id);

  public DifferentialDriveSub() {
    //TODO make two motors on each side run as a pair
    //b.follow(a)
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
    // a.set(speed -1->1)
  }

  /** Stops all motors. */
  public void off() {
    // TODO make this function stop the motors
  }

}