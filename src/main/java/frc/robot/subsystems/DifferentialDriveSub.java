package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Drive Subsystem.
 * Handles all drive functionality.
 */
public class DifferentialDriveSub extends SubsystemBase {
  //TODO initialise the motors (4 of them)
  //private final WPI_TalonSRX a = new WPI_TalonSRX(Constants.MOTOR_ID_LM);
  private final WPI_TalonSRX a = new WPI_TalonSRX(Constants.MOTOR_ID_LM);
  private final WPI_TalonSRX a2 = new WPI_TalonSRX(Constants.MOTOR_ID_LS);
  private final WPI_TalonSRX b = new WPI_TalonSRX(Constants.MOTOR_ID_RM);
  private final WPI_TalonSRX b2 = new WPI_TalonSRX(Constants.MOTOR_ID_RS);

  private final ADIS16470_IMU imu = new ADIS16470_IMU();
  private final Encoder leftEncoder = Constants.ENCODER_ID_L.build();
  private final Encoder rightEncoder = Constants.ENCODER_ID_R.build();
  
  public final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
      Constants.KINEMATICS,
      Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ)),
      0, 0, new Pose2d());
  
  public DifferentialDriveSub() {
    //TODO make two motors on each side run as a pair
    //b.follow(a);
    a2.follow(a);
    b2.follow(b);    

    a.setSafetyEnabled(true);
    // a2.setSafetyEnabled(true);
    b.setSafetyEnabled(true);
    // b2.setSafetyEnabled(true);

    //a.setInverted(true); // which side // add to both motors on the side
    b.setInverted(true);
    b2.setInverted(true);
  }

  @Override
  public void periodic() {
    poseEstimator.update(
        Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ)),
        leftEncoder.getDistance(),
        rightEncoder.getDistance());
    
    if (!Variables.driveEncodersDetected) {
      var pos = getEstimatedPose();
      if (Math.abs(pos.getX()) > 0.01 &&
          Math.abs(pos.getY()) > 0.01) {
        Variables.driveEncodersDetected = true;
      }
    }
  }

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
    a.set(throttle+steering);
    b.set(throttle-steering);

  }

  /** Stops all motors. */
  public void off() {
    // TODO make this function stop the motors
    a.set(0);
    b.set(0);
  }
  
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }
}