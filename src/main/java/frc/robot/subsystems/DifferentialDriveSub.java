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
  private final WPI_TalonSRX l = new WPI_TalonSRX(Constants.MOTOR_ID_LM);
  private final WPI_TalonSRX l2 = new WPI_TalonSRX(Constants.MOTOR_ID_LS);

  private final WPI_TalonSRX r = new WPI_TalonSRX(Constants.MOTOR_ID_RM);
  private final WPI_TalonSRX r2 = new WPI_TalonSRX(Constants.MOTOR_ID_RS);

  public DifferentialDriveSub() {
    //TODO make two motors on each side run as a pair
    l2.follow(l);
    r2.follow(r); 

    l2.setSafetyEnabled(true);
    l.setSafetyEnabled(true);
    
    r.setInverted(true);
    r2.setInverted(true);
    r2.setSafetyEnabled(true);
    r.setSafetyEnabled(true);
    
    // which side // add to both motors on the side
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
    l.set(throttle+steering);
    r.set(throttle-steering);


  }

  /** Stops all motors. */
  public void off() {
    // TODO make this function stop the motors
  }

}