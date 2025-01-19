package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class DriveProcessorCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Command commandGroup;
    private Command followPathCommand;

    private final PathConstraints CONSTRAINTS = new PathConstraints(4.5, 4.0, Units.degreesToRadians(540),
            Units.degreesToRadians(720));

    /**
     * 
     */
    public DriveProcessorCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        this.commandGroup.cancel();
        this.swerveDriveSubsystem.resetDrivePID();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        // zero drive pid since we are driving closed loop
        this.swerveDriveSubsystem.zeroDrivePID();

        schedulePathCommand();

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return followPathCommand.isFinished();
    }

    /**
     * 
     */
    public void schedulePathCommand() {
        try {

            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile("DriveProcessor - Path");

            this.followPathCommand = AutoBuilder.pathfindThenFollowPath(pathPlannerPath, CONSTRAINTS);
            this.commandGroup = Commands.sequence(this.followPathCommand);
            this.commandGroup.schedule();

        } catch (Exception e) {
        }
    }
}
