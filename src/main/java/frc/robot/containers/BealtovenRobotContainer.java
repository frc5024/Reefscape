package frc.robot.containers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.SwerveDriveCommands;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TuningSwerveCommand;
import frc.robot.modules.gyro.GyroModuleIONavX;
import frc.robot.modules.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.SwerveDriveSubsystem;
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

        // this.visionSubsystem = new VisionSubsystem(this.swerveDriveSubsystem,
        // this.swerveDriveSubsystem::getPose, this.swerveDriveSubsystem::getRotation);

        // registerNamedCommands();
        // configureAutoBuilder();
        configureButtonBindings();
    }

    @Override
    protected void configureButtonBindings() {
        CommandXboxController commandXboxController = new CommandXboxController(0);

        Command closedLoopDrive = SwerveDriveCommands.closedLoopDrive(swerveDriveSubsystem,
                () -> -commandXboxController.getLeftY(),
                () -> -commandXboxController.getLeftX(),
                () -> -commandXboxController.getRightX());

        Command openLoopDrive = new TeleopDriveCommand(this.swerveDriveSubsystem,
                () -> this.swerveDriveSubsystem.getPose().getRotation(),
                () -> commandXboxController.getLeftY(),
                () -> commandXboxController.getLeftX(),
                () -> commandXboxController.getRightX());

        Command tuningSwerveCommand = new TuningSwerveCommand(swerveDriveSubsystem,
                () -> -commandXboxController.getLeftY(),
                () -> -commandXboxController.getLeftX(),
                () -> -commandXboxController.getRightX(),
                commandXboxController);

        // Default command, normal field-relative drive
        swerveDriveSubsystem.setDefaultCommand(RobotConstants.TUNING_MODE ? tuningSwerveCommand : closedLoopDrive);
    }

    @Override
    public void registerNamedCommands() {
    }

    /**
     * Be sure to update SwerveConstants to match robot
     */
    private SwerveModuleBuilder[] getModuleConfigs() {
        SwerveModuleBuilder frontLeft = new SwerveModuleBuilder(41, 42, 4,
                Rotation2d.fromDegrees(155.566406), true, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder frontRight = new SwerveModuleBuilder(11, 12, 1,
                Rotation2d.fromDegrees(-317.021484), true, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder backLeft = new SwerveModuleBuilder(31, 32, 3,
                Rotation2d.fromDegrees(-72.861328), true, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder backRight = new SwerveModuleBuilder(21, 22, 2,
                Rotation2d.fromDegrees(-202.763672 + 180), true, true, SwerveConstants.cotsDriveConstants,
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
