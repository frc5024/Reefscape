package frc.robot.commands;

import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {

  private Climb ClimbSubsystem;
  private double speed;

  public ClimbCommand(Climb ClimbSubsystem, double speed) {
    this.ClimbSubsystem = ClimbSubsystem;
    this.speed = speed;

    addRequirements(ClimbSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    // ClimbSubsystem.linebreak();
    @SuppressWarnings("resource")
    
    DigitalInput limitSwitch = new DigitalInput(0);
    if (limitSwitch.get()) {
      // We are going up and limit is tripped so stop
      System.out.println("LIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMIT");
  } else {
      // We are going up but limit is not tripped so go at commanded speed
      
  }
  }

  @Override
  public void execute() {
    ClimbSubsystem.startMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    ClimbSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
