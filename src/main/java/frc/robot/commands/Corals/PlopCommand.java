
package frc.robot.commands.Corals;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.Coral;

public class PlopCommand extends Command {

    private final Coral coralSubsystem;

    // constructor for plopCommand
    public PlopCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }

    // Called when the command is initially scheduled, if line is broken, set state
    // to HOLDING
    @Override
    public void initialize() {
        coralSubsystem.set(CoralConstants.plopSpeed);

    }

    // execute, once button is pressed, startPlop()
    @Override
    public void execute() {

    }

    // end, when command ends, set IDLE
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
    }

    // if line is not broken, return true, else return false
    @Override
    public boolean isFinished() {
        return !coralSubsystem.isLineBroken();
    }
}
