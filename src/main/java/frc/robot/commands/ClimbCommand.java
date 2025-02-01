package frc.robot.commands;

import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {

  private Climb climbSubsystem;
  private double speed;

  public ClimbCommand(Climb climbSubsystem, double speed) {
    this.climbSubsystem = climbSubsystem;
    this.speed = speed;

    addRequirements(climbSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
  }

  public void execute() {
    // Stops motor when Ultrasonic sensor and Encoder detect end position
    if (climbSubsystem.overThreshold() && climbSubsystem.isClimbPosition()) {
      cancel();
    } else {
      climbSubsystem.startMotor(speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
