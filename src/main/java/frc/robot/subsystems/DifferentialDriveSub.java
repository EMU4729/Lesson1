package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Variables;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.Encoder;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Drive Subsystem.
 * Handles all drive functionality.
 */
public class DifferentialDriveSub extends SubsystemBase {
  // initialise the motors (4 of them)
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
    a2.follow(a);
    b2.follow(b);    
        
    a.setSafetyEnabled(true);
    b.setSafetyEnabled(true);
        
    // invert the correct side to account for physically inverted motor directions
    if (Variables.driveDirection) {
      b.setInverted(true);
      b2.setInverted(true);
    } else {
      a.setInverted(true);
      a2.setInverted(true);
    }
  }

  public final SysIdRoutine sysIdDrive = new SysIdRoutine(
    new SysIdRoutine.Config(
      Units.Volts.per(Units.Second).of(0.2),
      Units.Volt.of(0.4),
      Units.Second.of(6)),
    new SysIdRoutine.Mechanism(new Consumer<Measure<Voltage>>() {
      @Override
      public void accept(Measure<Voltage> t) {
        arcade(t.in(Units.Volt), 0);
      }
    }, new Consumer<SysIdRoutineLog>() {
      @Override
      public void accept(SysIdRoutineLog t) {
        t.motor("drive")
          .voltage(Units.Volt.of(getDriveThrottle()))
          .linearPosition(Units.Meter.of(leftEncoder.getDistance()))
          .linearVelocity(Units.MetersPerSecond.of(1234))
          .linearAcceleration(Units.MetersPerSecondPerSecond.of(imu.getAccelX()));
      }
    }, this));

  public final SysIdRoutine sysIdTurn = new SysIdRoutine(
    new SysIdRoutine.Config(
      Units.Volts.per(Units.Second).of(0.1),
      Units.Volt.of(0.4),
      Units.Second.of(6)),
    new SysIdRoutine.Mechanism(new Consumer<Measure<Voltage>>() {
      @Override
      public void accept(Measure<Voltage> t) {
        arcade(0, t.in(Units.Volt));
      }
    }, new Consumer<SysIdRoutineLog>() {
      @Override
      public void accept(SysIdRoutineLog t) {
        t.motor("turn")
          .voltage(Units.Volt.of(getTurnThrottle()))
          .angularPosition(Units.Degrees.of(imu.getAngle(imu.getYawAxis())))
          .angularVelocity(Units.DegreesPerSecond.of(imu.getRate(imu.getYawAxis())))
          .angularAcceleration(Units.DegreesPerSecond.per(Units.Second).of(imu.getYFilteredAccelAngle()));
      }
    }, this));
  
  @Override
  public void periodic() {
    // update the pose estimator
    poseEstimator.update(
        Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ)),
        leftEncoder.getDistance(),
        rightEncoder.getDistance());
  }

  /**
   * Arcade drive.
   * 
   * @param driveThrottle The speed
   * @param turnThrottle The steering
   */
  public void arcade(double driveThrottle, double turnThrottle) {
    driveThrottle = MathUtil.clamp(driveThrottle, -0.4, 0.4);
    turnThrottle = MathUtil.clamp(turnThrottle, -0.6, 0.6);

    a.set(driveThrottle+turnThrottle);
    b.set(driveThrottle-turnThrottle);
  }
  
  /** Stops all motors. */
  public void off() {
    a.set(0);
    b.set(0);
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }
  
  public double getTurnThrottle() {
    return a.get() - b.get();
  }
  public double getDriveThrottle() {
    return (a.get() + b.get()) / 2;
  }
}
