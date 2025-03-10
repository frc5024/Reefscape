package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.Coral;

public class LowerRampCommand extends Command {

    private final Coral coralSubsystem;

    public LowerRampCommand(Coral coralSubsystem) {

        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }

    // when start, rotate -plopspeed, lowers the ramp
    public void initialize() {
        coralSubsystem.set(CoralConstants.rampSpeed);
    }

    // when button pressed, rotate servo 90 degrees (lowers the ramp)
    public void execute() {
    }

    // when command ends, set idle
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}