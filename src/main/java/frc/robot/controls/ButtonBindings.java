package frc.robot.controls;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.elevator.SetElevatorSetpointCmd;
import frc.robot.commands.vision.GoToSetPositionPerTagCmd;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class ButtonBindings {
    private final int DRIVER_PORT = 0;
    private final int OPERATOR_PORT = 1;
    private final int TEST_PORT = 2;

    /* Controllers */
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final CommandXboxController testController;

    /* Subsystems */
    Coral coralSubsystem;
    Elevator elevatorSubsystem;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final Limelight limelightSubsystem;

    /* Variables */
    boolean visionMode = true;
    String mode;

    /**
     * 
     */
    public ButtonBindings(SwerveDriveSubsystem swerveDriveSubsystem, Coral coralSubsystem, Elevator elevatorSubsystem,
            Limelight limelightSubsystem) {
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        this.driverController = setDriverBindingsController();
        this.operatorController = setOperatorBindingsController();
        this.testController = setTestBindings();
    }

    /**
     * 
     */
    private CommandXboxController setDriverBindingsController() {
        CommandXboxController commandXboxController = new CommandXboxController(DRIVER_PORT);

        // Toggle game piece modes
        commandXboxController.back()
                .whileTrue(runOnce(() -> GameData.getInstance().toggleGamePieceMode()));

        // switch from robot relative to field relative
        commandXboxController.start()
                .whileTrue(either(
                        runOnce(this.swerveDriveSubsystem::disableFieldRelative,
                                this.swerveDriveSubsystem),
                        runOnce(this.swerveDriveSubsystem::enableFieldRelative,
                                this.swerveDriveSubsystem),
                        this.swerveDriveSubsystem::isFieldRelative));

        commandXboxController.x().onTrue(new InstantCommand(() -> this.swerveDriveSubsystem.zeroHeading()));
        commandXboxController.a().onTrue(new InstantCommand(() -> toggleVisionMode()));
        commandXboxController.b().whileTrue(
                new ConditionalCommand(this.elevatorSubsystem.goToModePosition(), new InstantCommand(),
                        () -> this.visionMode));

        commandXboxController.rightBumper().whileTrue(this.coralSubsystem.intakeCommand());
        commandXboxController.rightBumper().whileTrue(this.elevatorSubsystem.bottomElevator());

        commandXboxController.leftBumper()
                .onTrue(new InstantCommand(() -> this.swerveDriveSubsystem.isSlowMode = true));
        commandXboxController.leftBumper()
                .onFalse(new InstantCommand(() -> this.swerveDriveSubsystem.isSlowMode = false));

        commandXboxController.rightTrigger()
                .whileTrue(
                        new ConditionalCommand(
                                new GoToSetPositionPerTagCmd(limelightSubsystem, this.swerveDriveSubsystem,
                                        -FieldConstants.REEF_POLE_RIGHT_OFFSET),
                                new InstantCommand(), () -> visionMode));
        commandXboxController.leftTrigger()
                .whileTrue(new ConditionalCommand(
                        new GoToSetPositionPerTagCmd(limelightSubsystem, this.swerveDriveSubsystem,
                                FieldConstants.REEF_POLE_LEFT_OFFSET),
                        new InstantCommand(), () -> visionMode));

        // manual
        commandXboxController.rightTrigger()
                .onTrue(new ConditionalCommand(new InstantCommand(), coralSubsystem.outtakeCommand(),
                        () -> visionMode));

        return commandXboxController;
    }

    /**
     * 
     */
    private CommandXboxController setOperatorBindingsController() {
        CommandXboxController commandXboxController = new CommandXboxController(OPERATOR_PORT);

        commandXboxController.a().whileTrue(this.elevatorSubsystem.bottomElevator());
        commandXboxController.b().onTrue(coralSubsystem.lowerRampCommand());

        commandXboxController.rightTrigger().onTrue(this.coralSubsystem.lowerRampCommand());

        commandXboxController.povLeft()
                .whileTrue(new SetElevatorSetpointCmd(this.elevatorSubsystem, ElevatorConstants.L1Position));
        commandXboxController.povDown()
                .whileTrue(new SetElevatorSetpointCmd(this.elevatorSubsystem, ElevatorConstants.L2Position));
        commandXboxController.povRight()
                .whileTrue(new SetElevatorSetpointCmd(this.elevatorSubsystem, ElevatorConstants.L3Position));
        commandXboxController.povUp()
                .whileTrue(new SetElevatorSetpointCmd(this.elevatorSubsystem, ElevatorConstants.L4Position));

        commandXboxController.start().whileTrue(this.elevatorSubsystem.slowL2());

        return commandXboxController;
    }

    /**
     * 
     */
    private CommandXboxController setTestBindings() {
        CommandXboxController commandXboxController = new CommandXboxController(TEST_PORT);

        return commandXboxController;
    }

    /**
     * 
     */
    private void toggleVisionMode() {
        visionMode = !visionMode;

        if (visionMode) {
            mode = "Vision";
        } else {
            mode = "Manual";
        }

        SmartDashboard.putString("Robot Mode", mode);
    }

    /**
     * 
     */
    public CommandXboxController getDriverController() {
        return this.driverController;
    }

    public CommandXboxController getOperatorController() {
        return this.operatorController;
    }

    public CommandXboxController getTestController() {
        return this.testController;
    }
}
