package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Drive Subsystem.
 * Handles all drive functionality.
 */
public class DifferentialDriveSub extends SubsystemBase {
  //TODO initialise the motors (4 of them)
  //private final WPI_TalonSRX a = new WPI_TalonSRX(Constants.MOTOR_ID_LM);
  
  public DifferentialDriveSub() {
    //TODO make two motors on each side run as a pair
    //b.follow(a);

    //a.setSafetyEnabled(true);
    //a.setInverted(true); // which side // add to both motors on the side
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
    MathUtil.clamp(throttle, -0.2, 0.2);
    MathUtil.clamp(steering, -0.2, 0.2);
    // TODO take the throttle and steering and calc the power for each side (tank drive)
    //a.set(power -1->1);

  }

  /** Stops all motors. */
  public void off() {
    // TODO make this function stop the motors
  }

}