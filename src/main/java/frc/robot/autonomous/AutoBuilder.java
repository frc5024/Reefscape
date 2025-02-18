package frc.robot.autonomous;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.FeedForwardCharacterizationCommand;
import frc.robot.commands.WheelRadiusCharacterizationCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.LocalADStarAK;

/**
 * 
 */
public class AutoBuilder extends com.pathplanner.lib.auto.AutoBuilder {
    /* Subsystems */
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
    private final CoralIntakeSubsystem coralIntakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    /* Constraints */
    public static final PathConstraints CONSTRAINTS = new PathConstraints(
            4.5, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    /* Autonomous Chooser */
    private LoggedDashboardChooser<Command> autonomousChooser;

    /**
     * 
     */
    public AutoBuilder(SwerveDriveSubsystem swerveDriveSubsystem, AlgaeIntakeSubsystem algaeIntakeSubsystem,
            CoralIntakeSubsystem coralIntakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    /**
     * 
     */
    public void configureAutonomous() {
        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this.swerveDriveSubsystem::getPose,
                this.swerveDriveSubsystem::setPose,
                this.swerveDriveSubsystem::getChassisSpeeds,
                this.swerveDriveSubsystem::drive,
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                Constants.PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this.swerveDriveSubsystem);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });

        // Setup the chooser
        this.autonomousChooser = new LoggedDashboardChooser<Command>("Auto Routine", AutoBuilder.buildAutoChooser());

        if (RobotConstants.TUNING_MODE) {
            this.autonomousChooser.addOption("Drive Characterization",
                    new FeedForwardCharacterizationCommand(this.swerveDriveSubsystem).get());
            this.autonomousChooser.addOption("Wheel Radius Characterization",
                    new WheelRadiusCharacterizationCommand(this.swerveDriveSubsystem).get());
        }

        // Load Game Autos
        this.autonomousChooser.addOption("Drive Away", new PathPlannerAuto("DriveAway"));
        this.autonomousChooser.addOption("Clear Algae",
                new ClearAlgae(this.algaeIntakeSubsystem, this.coralIntakeSubsystem, this.elevatorSubsystem)
                        .getAutoCommand());
        this.autonomousChooser.addOption("Grab Algae", new GrabAlgae(this.algaeIntakeSubsystem).getAutoCommand());
    }

    /**
     * 
     */
    public LoggedDashboardChooser<Command> getAutonomousChooser() {
        return this.autonomousChooser;
    }

    /**
     * 
     */
    public static Command getPathFindingCommand(Pose2d targetPose) {
        Command pathFindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                CONSTRAINTS,
                0.0);

        return pathFindingCommand;
    }
}
