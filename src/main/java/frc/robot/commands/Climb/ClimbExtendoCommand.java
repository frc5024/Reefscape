package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbExtendoCommand extends Command {

    private Climb climbSubsystem;
    private boolean forward;

    public ClimbExtendoCommand(Climb climbSubsystem, boolean forward) {
        this.climbSubsystem = climbSubsystem;
        this.forward = forward;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        // stops when encoder detects extended position
        if (forward == true) {
            if (climbSubsystem.isExtendoPosition()) {
                climbSubsystem.stopMotor();
            } else {
                climbSubsystem.extending();
            }
        } else {
            // retracting the arm goes back to the default climb position for now
            if (climbSubsystem.isClimbPosition()) {
                climbSubsystem.stopMotor();
            } else {
                climbSubsystem.retracting();
            }
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