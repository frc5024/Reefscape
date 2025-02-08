package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbCancelCommand extends Command {

    private Climb climbSubsystem;

    public ClimbCancelCommand(Climb climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        // stops when encoder reaches extended position
        if (climbSubsystem.isExtendoPosition()) {
            climbSubsystem.stopMotor();
        } else {
            climbSubsystem.cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}