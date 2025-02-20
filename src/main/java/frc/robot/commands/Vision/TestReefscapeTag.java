package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Limelight;

public class TestReefscapeTag extends Command {
    private final Limelight limelight;
    private final Coral coralSubsystem;

    private final double targetID = 7;

    private Command intake;

    public TestReefscapeTag(Limelight limelight, Coral coralSubsystem) {
        this.limelight = limelight;
        this.coralSubsystem = coralSubsystem;
        intake = coralSubsystem.intakeCommand();

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // checks to see if april tag seen is required tag
        if (limelight.getAprilTagID() == targetID) {
            intake.schedule();
        } else {
            intake.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
    }
}
