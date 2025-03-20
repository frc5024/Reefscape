package frc.robot.commands.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Drives to reef station based on game piece mode settings
 */
public class DriveReefStationPathCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final Supplier<Integer> stationSupplier;
    private final Supplier<String> gamePieceName;

    private Command commandGroup;
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
    public void schedulePathCommand() {
        try {

            int reefStationIndex = this.stationSupplier.get();
            String gamePieceName = this.gamePieceName.get();
            PathPlannerPath pathPlannerPath = PathPlannerPath
                    .fromPathFile("DriveReef" + reefStationIndex + " - " + gamePieceName);

            this.followPathCommand = AutoBuilder.pathfindThenFollowPath(pathPlannerPath,
                    frc.robot.autonomous.AutoBuilder.CONSTRAINTS);
            this.commandGroup = Commands.sequence(this.followPathCommand);
            this.commandGroup.schedule();

        } catch (Exception e) {
        }
    }
}
