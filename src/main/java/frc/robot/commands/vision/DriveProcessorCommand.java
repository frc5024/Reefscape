package frc.robot.commands.vision;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Drives to processor station based on pathplanner plan
 */
public class DriveProcessorCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Command commandGroup;
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
        super.end(interrupted);
        if (this.commandGroup != null)
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
        try {

            return this.followPathCommand.isFinished();

        } catch (Exception e) {
            return true;
        }
    }

    /**
     * 
     */
    private void schedulePathCommand() {
        try {

            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile("DriveProcessor - Path");

            this.followPathCommand = AutoBuilder.pathfindThenFollowPath(pathPlannerPath,
                    frc.robot.autonomous.AutoBuilder.CONSTRAINTS);
            this.commandGroup = Commands.sequence(this.followPathCommand);
            this.commandGroup.schedule();

        } catch (Exception e) {
        }
    }
}
