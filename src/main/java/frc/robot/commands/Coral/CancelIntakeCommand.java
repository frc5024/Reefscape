
package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

public class CancelIntakeCommand extends Command {

    private final Coral coralSubsystem;
    // private static DigitalInput linebreakBottom;

    // constructor for cancelIntakeCommand
    public CancelIntakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }

    // initialize, when command starts, set state to IDLE
    @Override
    public void initialize() {
        coralSubsystem.setIdle();
    }

    // execute
    @Override
    public void execute() {

    }

    // end
    @Override
    public void end(boolean interrupted) {

    }

    // always return true for isFinished()
    @Override
    public boolean isFinished() {
        return true;
    }

}