package frc.robot.commands.vision;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Drives to processor station based on pathplanner plan
 */
public class DriveProcessorCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Command followPathCommand;

    /**
     * 
     */
    public DriveProcessorCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        addRequirements(this.swerveDriveSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        if (this.followPathCommand != null) {
            this.followPathCommand.end(interrupted);
            Logger.recordOutput("Commands/Active Command", "");
        }

        this.swerveDriveSubsystem.resetDrivePID();
    }

    @Override
    public void execute() {
        if (this.followPathCommand != null) {
            this.followPathCommand.execute();
        }
    }

    @Override
    public void initialize() {
        // zero drive pid since we are driving closed loop
        this.swerveDriveSubsystem.zeroDrivePID();

        this.followPathCommand = getFollowPathCommand();

        if (this.followPathCommand != null) {
            this.followPathCommand.initialize();
            Logger.recordOutput("Commands/Active Command", this.getName());
        }
    }

    @Override
    public boolean isFinished() {
        return this.followPathCommand != null ? this.followPathCommand.isFinished() : true;
    }

    /**
     * 
     */
    private Command getFollowPathCommand() {
        try {

            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile("DriveProcessor - Path");

            return AutoBuilder.pathfindThenFollowPath(pathPlannerPath, frc.robot.autonomous.AutoBuilder.CONSTRAINTS);

        } catch (Exception e) {
            return null;
        }
    }
}
