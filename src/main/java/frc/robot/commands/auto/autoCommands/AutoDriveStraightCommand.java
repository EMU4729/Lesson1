package frc.robot.commands.auto.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class AutoDriveStraightCommand extends Command {

  private final PIDController pidController = new PIDController(1, 0, 0);

  private Timer timer = new Timer();

  private final double timeout;
  private final double distance;
  private final double throttle;

  private Pose2d startPose;

  private boolean isFinished = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDriveStraightCommand(double distance, double timeout, double throttle) {
    this.timeout = timeout;
    this.distance = distance;
    this.throttle = throttle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPose = Subsystems.diffDrive.getEstimatedPose();
    pidController.setSetpoint(startPose.getRotation().getRadians());
    pidController.setTolerance(Rotation2d.fromDegrees(1).getRadians());
    Subsystems.diffDrive.arcade(throttle, 0);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isFinished) {
      return;
    }

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

    var steering = -pidController.calculate(pose.getRotation().getRadians());
    
    Subsystems.diffDrive.arcade(throttle, steering);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    pidController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
