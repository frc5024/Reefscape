
package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Rumble;

public class IntakeCommand extends Command {

    private final Coral coralSubsystem;

    Rumble rumble = Rumble.getInstance();

    // constructor for IntakeCommand
    public IntakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }

    // initialize, when command starts, if line is not broken, set state to IDLE
    @Override
    public void initialize() {
        if (!coralSubsystem.isLineBroken()) {
            coralSubsystem.set(CoralConstants.intakeSpeed);
        }
    }

    // execute, if button is pressed, startIntake()
    @Override
    public void execute() {
    }

    // end, when command ends, set Idle
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();

        if (!interrupted)
            rumble.staticRumble(true);
    }

    // if line is broken, return true, else return false
    @Override
    public boolean isFinished() {
        return coralSubsystem.isLineBroken();
    }
}