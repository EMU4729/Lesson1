// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.teleop.TeleopDriveArcade;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(Subsystems.m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(Subsystems.m_exampleSubsystem));

    OI.pilot.start().onTrue(new InstantCommand(() -> Variables.invertDriveDirection = !Variables.invertDriveDirection));
    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // OI.pilot.b().whileTrue(Subsystems.m_exampleSubsystem.exampleMethodCommand());
    OI.pilot.leftTrigger().whileTrue(new StartEndCommand(
      () -> Subsystems.someSubsystem.set(Variables.driveSpeed),
      Subsystems.someSubsystem::stop,
      Subsystems.someSubsystem)
    );
    
    OI.pilot.rightTrigger().whileTrue(new StartEndCommand(
      () -> Subsystems.someSubsystem.set(-Variables.driveSpeed),
      Subsystems.someSubsystem::stop,
      Subsystems.someSubsystem)
    );
    
    OI.pilot.x().onTrue(new InstantCommand(() -> {
      Variables.driveSpeed = MathUtil.clamp(Variables.driveSpeed + 0.1, 0, 1);
      Subsystems.someSubsystem.updateSpeed(Variables.driveSpeed);
      System.out.println("Changed speed to " + Variables.driveSpeed);
    }));
    
    OI.pilot.y().onTrue(new InstantCommand(() -> {
      Variables.driveSpeed = MathUtil.clamp(Variables.driveSpeed - 0.1, 0, 1);
      Subsystems.someSubsystem.updateSpeed(Variables.driveSpeed);
      System.out.println("Changed speed to " + Variables.driveSpeed);
    }));
    
    
    OI.pilot.a().onTrue(new InstantCommand(() -> {
      Variables.driveSpeed = MathUtil.clamp(Variables.driveBackspin + 0.02, 0, 1);
      Subsystems.someSubsystem.updateSpeed(Variables.driveSpeed);
      System.out.println("Changed backspin to " + Variables.driveBackspin);
    }));
    
    OI.pilot.b().onTrue(new InstantCommand(() -> {
      Variables.driveSpeed = MathUtil.clamp(Variables.driveBackspin - 0.02, 0, 1);
      Subsystems.someSubsystem.updateSpeed(Variables.driveSpeed);
      System.out.println("Changed backspin to " + Variables.driveBackspin);
    }));
     
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
    // An example command will be run in autonomous
    return Autos.exampleAuto(Subsystems.m_exampleSubsystem);
  }
}
