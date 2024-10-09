// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.utils.builders.EncoderBuilder;
import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.TalonPwm;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final FileConstants FILE = new FileConstants();
  public static final LoggerConstants LOGGER = new LoggerConstants();

  public static class OperatorConstants {
    public static final int PILOT_XBOX_CONTROLLER_PORT = 0;
  }
  
  public static final boolean USE_CLAMPING = true;

  public static final MotorSupplier<Talon> MOTOR_LM_SUPPLIER = new TalonPwm(2).withInvert().withSafety();
  public static final MotorSupplier<Talon> MOTOR_LS_SUPPLIER = new TalonPwm(3);
  public static final MotorSupplier<Talon> MOTOR_RM_SUPPLIER = new TalonPwm(0).withSafety();
  public static final MotorSupplier<Talon> MOTOR_RS_SUPPLIER = new TalonPwm(1).withInvert();

  public static final EncoderBuilder ENCODER_ID_L = new EncoderBuilder(new int[] { 20, 19 }, 59.883 / 256. / 1000);
  public static final EncoderBuilder ENCODER_ID_R = new EncoderBuilder(new int[] { 18, 17 }, 59.883 / 256. / 1000).withInvert();

  public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(0.55);

}
