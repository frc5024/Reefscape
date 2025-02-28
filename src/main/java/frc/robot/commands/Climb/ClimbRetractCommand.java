package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbRetractCommand extends Command {

    private Climb climbSubsystem;

    public ClimbRetractCommand(Climb climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        climbSubsystem.retracting();
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        /* Checks if climb arm is at retracted position */
        return false;
    }
}