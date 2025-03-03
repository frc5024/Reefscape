package frc.robot.commands.vision;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * A command that runs pathfindThenFollowPath based on the current drive mode.
 */
public class PathFinderAndFollowCommand extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final String pathName;

    private Command commandGroup;
    private Command followPathCommand;

    /**
     * Creates a new PathFinderAndFollow command.
     *
     * @param stationModeSupplier a supplier for the drive mode type
     */
    public PathFinderAndFollowCommand(SwerveDriveSubsystem swerveDrive, String pathName) {
        this.swerveDrive = swerveDrive;
        this.pathName = pathName;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (this.commandGroup != null)
            this.commandGroup.cancel();
        this.swerveDrive.resetDrivePID();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        // zero drive pid since we are driving closed loop
        this.swerveDrive.zeroDrivePID();

        schedulePathCommand();

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        try {

            return followPathCommand.isFinished();

        } catch (Exception e) {
            return true;
        }
    }

    /** Runs a new autonomous path based on the current drive mode. */
    public void schedulePathCommand() {
        try {

            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(this.pathName);

            this.followPathCommand = AutoBuilder.pathfindThenFollowPath(pathPlannerPath,
                    frc.robot.autonomous.AutoBuilder.CONSTRAINTS);
            this.commandGroup = Commands.sequence(this.followPathCommand);
            this.commandGroup.schedule();

        } catch (Exception e) {
        }
    }
}
