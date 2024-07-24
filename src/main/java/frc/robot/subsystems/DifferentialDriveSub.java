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
  private final WPI_TalonSRX MotorLM = new WPI_TalonSRX(Constants.MOTOR_ID_LM);
  private final WPI_TalonSRX MotorLS = new WPI_TalonSRX(Constants.MOTOR_ID_LS);
  private final WPI_TalonSRX MotorRM = new WPI_TalonSRX(Constants.MOTOR_ID_RM);
  private final WPI_TalonSRX MotorRS = new WPI_TalonSRX(Constants.MOTOR_ID_RS);
  
  public DifferentialDriveSub() {
    //TODO make two motors on each side run as a pair
    MotorLS.follow(MotorLM);
    MotorRS.follow(MotorRM);

    MotorLM.setSafetyEnabled(true);
    MotorRM.setSafetyEnabled(true);
    
    MotorLM.setInverted(true); // which side // add to both motors on the side
    MotorLS.setInverted(true); // which side // add to both motors on the side
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
    double powl = MathUtil.clamp(throttle + steering, -0.3, 0.3);
    double powr = MathUtil.clamp(throttle - steering, -0.3, 0.3);
    // TODO take the throttle and steering and calc the power for each side (tank drive)
    MotorLM.set(powl);
    MotorRM.set(powr);

  }

  /** Stops all motors. */
  public void off() {
    MotorLM.set(0);
    MotorLM.set(0);
    // TODO make this function stop the motors
  }

}