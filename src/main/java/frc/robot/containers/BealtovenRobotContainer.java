package frc.robot.containers;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.modules.algae.AlgaeModuleIOSim;
import frc.robot.modules.coral.CoralModuleIOSim;
import frc.robot.modules.elevator.ElevatorModuleIOSim;
import frc.robot.modules.gyro.GyroModuleIONavX;
import frc.robot.modules.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SwerveModuleBuilder;

/**
 * 
 */
public class BealtovenRobotContainer extends RobotContainer {
    /**
     * 
     */
    public BealtovenRobotContainer() {
        super();

        SwerveModuleBuilder[] swerveModuleConfigs = getModuleConfigs();

        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroModuleIONavX(),
                new SwerveModuleIOTalonFX(swerveModuleConfigs[0]),
                new SwerveModuleIOTalonFX(swerveModuleConfigs[1]),
                new SwerveModuleIOTalonFX(swerveModuleConfigs[2]),
                new SwerveModuleIOTalonFX(swerveModuleConfigs[3]));

        this.visionSubsystem = new VisionSubsystem(VisionConstants.BEALTOVEN_CAMERAS, this.swerveDriveSubsystem,
                this.swerveDriveSubsystem::getPose, this.swerveDriveSubsystem::getRotation);

        this.algaeSubsystem = new AlgaeSubsystem(new AlgaeModuleIOSim());
        this.coralSubsystem = new CoralSubsystem(new CoralModuleIOSim());
        this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorModuleIOSim(), this.algaeSubsystem::hasAlgae,
                this.coralSubsystem::hasCoral);

        registerNamedCommands();
        configureAutoBuilder();
        configureButtonBindings();
    }

    @Override
    public void registerNamedCommands() {
        NamedCommands.registerCommand("ElevatorL0", new InstantCommand(() -> {
            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_BOTTOM);
        }));
        NamedCommands.registerCommand("ElevatorL4", new InstantCommand(() -> {
            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_CORAL_4);
        }));
        NamedCommands.registerCommand("EjectCoral", new InstantCommand(() -> {
            this.coralSubsystem.addAction(CoralSubsystem.Action.EJECT);
        }));
        NamedCommands.registerCommand("IntakeCoral", new InstantCommand(() -> {
            this.coralSubsystem.addAction(CoralSubsystem.Action.INTAKE);
        }));
        NamedCommands.registerCommand("WaitForEject", new WaitUntilCommand(this.coralSubsystem::hasEjected));
        NamedCommands.registerCommand("WaitForElevator", new WaitUntilCommand(this.elevatorSubsystem::atGoal));
        NamedCommands.registerCommand("WaitForIntake", new WaitUntilCommand(this.coralSubsystem::hasCoral));
    }

    /**
     * Be sure to update SwerveConstants to match robot
     */
    private SwerveModuleBuilder[] getModuleConfigs() {
        SwerveModuleBuilder frontLeft = new SwerveModuleBuilder(41, 42, 4,
                Rotation2d.fromRotations(-0.4775390625), false, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder frontRight = new SwerveModuleBuilder(11, 12, 1,
                Rotation2d.fromRotations(0.37744140625), true, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder backLeft = new SwerveModuleBuilder(31, 32, 3,
                Rotation2d.fromRotations(0.238525390625), false, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder backRight = new SwerveModuleBuilder(21, 22, 2,
                Rotation2d.fromRotations(-0.474365234375), true, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);

        return new SwerveModuleBuilder[] { frontLeft, frontRight, backLeft, backRight };
    }

    /**
     * Maple Sim Routines not used for Real Robot
     */
    public void displaySimFieldToAdvantageScope() {
    }

    public void resetSimulationField(Pose2d pose2d) {
    }
}
