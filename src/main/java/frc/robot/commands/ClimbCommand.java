package frc.robot.commands;

import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {

  private Climb ClimbSubsystem;

  public ClimbCommand(Climb ClimbSubsystem, boolean motorOn) {
    this.ClimbSubsystem = ClimbSubsystem;
    addRequirements(ClimbSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    ClimbSubsystem.motorController(true);
  }

  @Override
  public void execute() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
