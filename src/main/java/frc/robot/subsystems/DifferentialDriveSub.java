package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Drive Subsystem.
 * Handles all drive functionality.
 */
public class DifferentialDriveSub extends SubsystemBase {
  //Intiialise motors
  public final PWMTalonSRX lm = new PWMTalonSRX(Constants.MOTOR_ID_LM);
  public final PWMTalonSRX ls = new PWMTalonSRX(Constants.MOTOR_ID_LS);
  public final PWMTalonSRX rm = new PWMTalonSRX(Constants.MOTOR_ID_RM);
  public final PWMTalonSRX rs = new PWMTalonSRX(Constants.MOTOR_ID_RS);
  //initialise Encoders and Gyro
  private final ADIS16470_IMU imu = new ADIS16470_IMU();
  private final Encoder leftEncoder = Constants.ENCODER_ID_L.build();
  private final Encoder rightEncoder = Constants.ENCODER_ID_R.build();
  

  
  public final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
      Constants.KINEMATICS,
      Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ)),
      0, 0, new Pose2d());
  
  public DifferentialDriveSub() {
    //let slave motors follow master motors
    ls.addFollower(lm);
    rs.addFollower(rm);    

    lm.setSafetyEnabled(true);
    // a2.setSafetyEnabled(true);
    rm.setSafetyEnabled(true);
    // b2.setSafetyEnabled(true);

    //a.setInverted(true); // which side // add to both motors on the side
    rm.setInverted(true);
    rs.setInverted(true);

    

    
    
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
  
  public double getHeading(){
    return Math.IEEEremainder(imu.getAngle(), 360);
  }
  /**
   * Arcade drive.
   * 
   * @param throttle The speed
   * @param steering The steering
   */
  public void arcade(double throttle, double steering) {
    //throttle = MathUtil.clamp(throttle, -0.2, 0.2);
    //steering = MathUtil.clamp(steering, -0.2, 0.2);

    
    lm.set(throttle+steering);
    rm.set(throttle-steering);

  }

  /** Stops all motors. */
  public void off() {
    
    lm.set(0);
    rm.set(0);
  }


// Reset the encoders
public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
}


  
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }
}