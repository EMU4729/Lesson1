package frc.robot.commands.auto.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class AutoTurnCommand extends Command {

  private final PIDController pidController = new PIDController(1, 0, 0);

  private Timer timer = new Timer();

  private final double timeout;
  private final Rotation2d targetAngle;

  private boolean isFinished = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoTurnCommand(Rotation2d targetAngle, double timeout) {
    this.timeout = timeout;
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(0);
    pidController.setTolerance(Rotation2d.fromDegrees(1).getRadians());
    Subsystems.diffDrive.arcade(0, 0);
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

    var rotation = Subsystems.diffDrive.getEstimatedPose().getRotation();
    var angleDiff = targetAngle.minus(rotation).getRadians();
    var steering = pidController.calculate(angleDiff);
    if (Math.abs(steering) <= 1e-2) {
      Subsystems.diffDrive.off();
      isFinished = true;
      return;
    }

    System.out.println("a:" + Rotation2d.fromRadians(angleDiff).getDegrees() + " s:" + steering);
    
    Subsystems.diffDrive.arcade(0, steering);
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
