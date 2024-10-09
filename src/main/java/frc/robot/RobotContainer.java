// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.auto.Auto;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.commands.auto.autoFactories.AutoDriveStraight;
import frc.robot.commands.auto.autoFactories.AutoTurn;
import frc.robot.teleop.TeleopDriveArcade;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TeleopDriveArcade teleopCommand = new TeleopDriveArcade();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    OI.pilot.x().onTrue(Subsystems.diffDrive.sysIdTurn.quasistatic(SysIdRoutine.Direction.kForward));
    OI.pilot.y().onTrue(Subsystems.diffDrive.sysIdTurn.quasistatic(SysIdRoutine.Direction.kReverse));
    OI.pilot.a().onTrue(Subsystems.diffDrive.sysIdTurn.dynamic(SysIdRoutine.Direction.kForward));
    OI.pilot.b().onTrue(Subsystems.diffDrive.sysIdTurn.dynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getTeleopCommand() {
    return teleopCommand;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new Auto(new AutoFactory[] {
      // new AutoDriveStraight(1, 0.2, 1),
      // new AutoTurn(Rotation2d.fromDegrees(90), 5),
    });
  }
}
