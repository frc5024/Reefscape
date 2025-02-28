package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbExtendoCommand extends Command {

    private Climb climbSubsystem;

    public ClimbExtendoCommand(Climb climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        climbSubsystem.extending();
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        /* Checls if climb arm is at extended position */
        return false;
    }
}