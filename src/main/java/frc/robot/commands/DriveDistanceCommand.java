// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu  .wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

/** An example command that uses an example subsystem. */
public class DriveDistanceCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private PIDController pidController = new PIDController(1, 0, 0);

  private Timer timer = new Timer();

  private double timeout;
  private double distance;
  private double throttle;

  private Pose2d startPose;
  private double TargetAngle;

  private boolean isFinished = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveDistanceCommand(double distance, double timeout, double throttle) {
    this.timeout = timeout;
    this.distance = distance;
    this.throttle = throttle;

    this.TargetAngle = TargetAngle;

    pidController = new PIDController(0.05, 0.0, 0.02);
  
  addRequirements(Subsystems.diffDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    startPose = Subsystems.diffDrive.getEstimatedPose();
    TargetAngle = 90.0;
    pidController.setSetpoint(startPose.getRotation().getRadians());
    pidController.setTolerance(Rotation2d.fromDegrees(1).getRadians());
   
    
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isFinished) {
      return;
    }

    double currentAngle = Subsystems.diffDrive.getHeading();

    double pidOutput = pidController.calculate(currentAngle, TargetAngle);
    Subsystems.diffDrive.arcade(throttle, pidOutput);
    
    if (timer.hasElapsed(timeout)) {
      Subsystems.diffDrive.off();
      isFinished = true;
      return;
    }

    var pose = Subsystems.diffDrive.getEstimatedPose();
    
    if (pose.getTranslation().getDistance(startPose.getTranslation()) >= distance) {
      
      Subsystems.diffDrive.off();
      isFinished = true;
      return;
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
