package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class isPathRun extends Command {

    private final Limelight limelight;

    public isPathRun(Limelight limelight) {

        this.limelight = limelight;
    }

    public void initialize() {

    }

    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        limelight.pathIsDone(false);
        System.out.println("Path Is DONE");
    }

    @Override
    public boolean isFinished() {
        return limelight.getPathIsDone();
    }
}
