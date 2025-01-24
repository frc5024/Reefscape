package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class DriveNearestCoralStationCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Command commandGroup;
    private Command followPathCommand;

    private final PathConstraints CONSTRAINTS = new PathConstraints(4.5, 4.0, Units.degreesToRadians(540),
            Units.degreesToRadians(720));

    /**
     * 
     */
    public DriveNearestCoralStationCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
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

            return followPathCommand.isFinished();

        } catch (Exception e) {
            return true;
        }
    }

    /**
     * 
     */
    public void schedulePathCommand() {
        try {

            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(getNearestCoralStation());

            this.followPathCommand = AutoBuilder.pathfindThenFollowPath(pathPlannerPath, CONSTRAINTS);
            this.commandGroup = Commands.sequence(this.followPathCommand);
            this.commandGroup.schedule();

        } catch (Exception e) {
        }
    }

    /**
     * 
     */
    private String getNearestCoralStation() {
        Pose2d currentPose = this.swerveDriveSubsystem.getPose();
        double leftStationDistance = Math.hypot(currentPose.getX() - FieldConstants.CORAL_STATION_POSES[0].getX(),
                currentPose.getY() - FieldConstants.CORAL_STATION_POSES[0].getY());
        double rightStationDistance = Math.hypot(currentPose.getX() - FieldConstants.CORAL_STATION_POSES[1].getX(),
                currentPose.getY() - FieldConstants.CORAL_STATION_POSES[1].getY());

        return leftStationDistance < rightStationDistance ? "DriveLeftCoraltation - Path"
                : "DriveRightCoralStation - Path";
    }
}
