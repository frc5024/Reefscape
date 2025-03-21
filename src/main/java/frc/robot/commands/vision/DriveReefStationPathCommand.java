package frc.robot.commands.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Drives to reef station based on game piece mode settings
 */
public class DriveReefStationPathCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final Supplier<Integer> stationSupplier;
    private final Supplier<String> gamePieceName;

    private Command followPathCommand;

    /**
     * Drives to reef station based on pose and pole selection from elastic input
     */
    public DriveReefStationPathCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Integer> stationSupplier,
            Supplier<String> gamePieceName) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.stationSupplier = stationSupplier;
        this.gamePieceName = gamePieceName;

        addRequirements(this.swerveDriveSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        if (this.followPathCommand != null) {
            Logger.recordOutput("Commands/Active Command", "");
            this.followPathCommand.end(interrupted);
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

        this.followPathCommand = getfollowPathCommand();
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
    private Command getfollowPathCommand() {
        int reefStationIndex = this.stationSupplier.get();
        String gamePieceName = this.gamePieceName.get();

        try {
            PathPlannerPath pathPlannerPath = PathPlannerPath
                    .fromPathFile("DriveReef" + reefStationIndex + " - " + gamePieceName);

            return AutoBuilder.pathfindThenFollowPath(pathPlannerPath,
                    frc.robot.autonomous.AutoBuilder.CONSTRAINTS);
        } catch (Exception e) {
            return null;
        }
    }
}
