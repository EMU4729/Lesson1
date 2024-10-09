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
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.Encoder;
import java.util.function.Consumer;
import static frc.robot.constants.Constants.*;

/**
 * Drive Subsystem.
 * Handles all drive functionality.
 */
public class DifferentialDriveSub extends SubsystemBase {
  // initialise the motors (4 of them)
  private final Talon leftMain = MOTOR_LM_SUPPLIER.get();
  private final Talon leftSlave =  MOTOR_LS_SUPPLIER.get();
  private final Talon rightMain =  MOTOR_RM_SUPPLIER.get();
  private final Talon rightSlave =  MOTOR_RS_SUPPLIER.get();

  private final ADIS16470_IMU imu = new ADIS16470_IMU();
  private final Encoder leftEncoder = ENCODER_ID_L.build();
  private final Encoder rightEncoder = ENCODER_ID_R.build();

  private double driveThrottle;
  private double turnThrottle;
  
  public final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
    KINEMATICS,
    Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ)),
    leftEncoder.getDistance(), rightEncoder.getDistance(), new Pose2d());
  
  public DifferentialDriveSub() {
    leftSlave.addFollower(leftMain);
    rightSlave.addFollower(rightMain);    
    leftMain.setSafetyEnabled(true);
    rightMain.setSafetyEnabled(true);

    leftEncoder.setDistancePerPulse(1);
    rightEncoder.setDistancePerPulse(1);
  }

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
    if (USE_CLAMPING) {
      driveThrottle = MathUtil.clamp(driveThrottle, -0.4, 0.4);
      turnThrottle = MathUtil.clamp(turnThrottle, -0.8, 0.8);
    }
    this.driveThrottle = driveThrottle;
    this.turnThrottle = turnThrottle;

    leftMain.set(driveThrottle+turnThrottle);
    rightMain.set(driveThrottle-turnThrottle);
  }
  
  /** Stops all motors. */
  public void off() {
    leftMain.set(0);
    rightMain.set(0);
    driveThrottle = 0;
    turnThrottle = 0;
  }

  public final SysIdRoutine sysIdDrive = new SysIdRoutine(
    new SysIdRoutine.Config(
      Units.Volts.per(Units.Second).of(0.2),
      Units.Volt.of(0.4),
      Units.Second.of(6)),
    new SysIdRoutine.Mechanism(new Consumer<Measure<Voltage>>() {
      @Override
      public void accept(Measure<Voltage> value) {
        arcade(value.in(Units.Volt), 0);
      }
    }, new Consumer<SysIdRoutineLog>() {
      @Override
      public void accept(SysIdRoutineLog log) {
        log.motor("drive")
          .voltage(Units.Volt.of(driveThrottle))
          .linearPosition(Units.Meter.of(leftEncoder.getDistance()))
          .linearVelocity(Units.MetersPerSecond.of(leftEncoder.getRate()))
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
      public void accept(Measure<Voltage> value) {
        arcade(0, value.in(Units.Volt));
      }
    }, new Consumer<SysIdRoutineLog>() {
      @Override
      public void accept(SysIdRoutineLog log) {
        log.motor("turn")
          .voltage(Units.Volt.of(turnThrottle))
          .angularPosition(Units.Degrees.of(imu.getAngle(imu.getYawAxis())))
          .angularVelocity(Units.DegreesPerSecond.of(imu.getRate(imu.getYawAxis())))
          .angularAcceleration(Units.DegreesPerSecond.per(Units.Second).of(imu.getYFilteredAccelAngle()));
      }
    }, this));
  
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }
  public double getTurnThrottle() {
    return turnThrottle;
  }
  public double getDriveThrottle() {
    return driveThrottle;
  }
}
