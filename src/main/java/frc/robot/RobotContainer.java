package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.leds.LEDPreset;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Vision.autoSetPositionTagID;
import frc.robot.commands.Vision.goToSetPositionPerTagCmd;
import frc.robot.commands.Vision.isPathRun;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final Climb m_climbSubsystem = Climb.getInstance();
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Limelight limelightSubsystem = new Limelight();
    private final Coral coralSubsystem = Coral.getInstance();
    private final Elevator elevatorSubsystem = Elevator.getInstance();
    private final LEDs s_LEDs = LEDs.getInstance();

    // Drive Controls
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // Vision Variables
    boolean visionMode = true;
    public String mode;

    // Auto
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis), () -> false // true =
                                                                                                          // robotcentric
        ));

        s_LEDs.setDefaultCommand().schedule();

        configureBindings();

        // Auto Commands
        // Vision
        NamedCommands.registerCommand("DriveRightTag",
                new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve, Constants.Vision.rightOffset));

        NamedCommands.registerCommand("DriveLeftTag",
                new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve, Constants.Vision.leftOffset));

        NamedCommands.registerCommand("DriveLeftTag11",
                new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.leftOffset, 11));

        NamedCommands.registerCommand("DriveRightTag11",
                new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.rightOffset, 11));

        NamedCommands.registerCommand("DriveRightTag10",
                new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.rightOffset, 10));

        NamedCommands.registerCommand("DriveLeftTag10",
                new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.leftOffset, 10));

        NamedCommands.registerCommand("DriveLeftTag9",
                new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.leftOffset, 9));

        NamedCommands.registerCommand("DriveRightTag9",
                new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.rightOffset, 9));

        NamedCommands.registerCommand("DriveRightTag8",
                new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.rightOffset, 8));

        NamedCommands.registerCommand("DriveLeftTag8",
                new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.leftOffset, 8));

        NamedCommands.registerCommand("DriveRightTag6",
                new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.rightOffset, 6));

        NamedCommands.registerCommand("DriveLeftTag6",
                new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.leftOffset, 6));

        NamedCommands.registerCommand("elevatorMode", elevatorSubsystem.goToModePosition());

        // Coral
        NamedCommands.registerCommand("ScoreCoral", coralSubsystem.outtakeAutoCommand());
        NamedCommands.registerCommand("IntakeCoral", coralSubsystem.intakeCommand());

        NamedCommands.registerCommand("Confirm Vision", new InstantCommand(() -> Limelight.done = true));
        NamedCommands.registerCommand("Wait For Vision", new isPathRun(limelightSubsystem));

        // Elevator
        NamedCommands.registerCommand("L4", elevatorSubsystem.goToL4Position());
        NamedCommands.registerCommand("L3", elevatorSubsystem.goToL3Position());
        NamedCommands.registerCommand("L2", elevatorSubsystem.goToL2Position());
        NamedCommands.registerCommand("L1", elevatorSubsystem.goToL1Position());
        NamedCommands.registerCommand("ElevatorRoot", elevatorSubsystem.bottomAutoElevator());

        // Auto Chooser
        autoChooser = AutoBuilder.buildAutoChooser();

        autoChooser.addOption("Non-Processor side 2/3 piece (COMPLETE)",
                Commands.sequence(new PathPlannerAuto("Start 11R"), new PathPlannerAuto("11R TS 6R"),
                        new PathPlannerAuto("6R TS 6L")));

        autoChooser.addOption("Processor side 2/3 piece (COMPLETE)",
                Commands.sequence(new PathPlannerAuto("Start 9R"), new PathPlannerAuto("9R BS 8R"),
                        new PathPlannerAuto("8R BS 8L")));

        autoChooser.addOption("Middle 1 piece right (COMPLETE)", new PathPlannerAuto("MiddleRight"));
        autoChooser.addOption("Middle 1 piece left (COMPLETE)", new PathPlannerAuto("MiddleLeft"));

        autoChooser.addOption("Testing Elevatoring", new PathPlannerAuto("Start 11R"));

        SmartDashboard.putData("Auto/Chooser", autoChooser);

        if (coralSubsystem.isLineBroken()) {
            s_LEDs.setCommand(LEDPreset.Solid.kGreen);
        } else {
            s_LEDs.setCommand(LEDPreset.Solid.kRed);
        }
    }

    // Vision Mode
    private void toggleVisionMode() {
        visionMode = !visionMode;

        if (visionMode) {
            mode = "Vision";
        } else {
            mode = "Manual";
        }

        SmartDashboard.putString("Robot Mode", mode);
    }

    // Driver Controls
    private void configureBindings() {
        // Driver Controls
        // Drive

        driver.leftBumper().whileTrue(new InstantCommand(() -> s_Swerve.isSlowMode = true));
        driver.leftBumper().onFalse(new InstantCommand(() -> s_Swerve.isSlowMode = false));

        driver.x().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        driver.rightTrigger()
                .onTrue(new ConditionalCommand(new InstantCommand(), coralSubsystem.outtakeCommand(),
                        () -> visionMode));

        // Vision
        driver.a().onTrue(new InstantCommand(() -> toggleVisionMode()));

        driver.rightTrigger()
                .whileTrue(new ConditionalCommand(new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve,
                        Constants.Vision.rightOffset), new InstantCommand(), () -> visionMode));
        driver.leftTrigger()
                .whileTrue(new ConditionalCommand(new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve,
                        Constants.Vision.leftOffset), new InstantCommand(), () -> visionMode));

        driver.b().onTrue(coralSubsystem.outtakeAutoCommand());

        // driver.rightBumper()
        // .whileTrue(new scoreCoralVisionCmd(limelightSubsystem, s_Swerve,
        // Constants.Vision.rightOffset,
        // elevatorSubsystem, coralSubsystem));
        // driver.leftBumper()
        // .whileTrue(new scoreCoralVisionCmd(limelightSubsystem, s_Swerve,
        // Constants.Vision.leftOffset,
        // elevatorSubsystem, coralSubsystem));

        // Elevator
        driver.y().whileTrue(coralSubsystem.forcedOuttakeCommand());

        // Coral
        driver.rightBumper().whileTrue(coralSubsystem.intakeCommand());
        driver.rightBumper().onTrue(elevatorSubsystem.bottomElevator());
        driver.start().whileTrue(coralSubsystem.backwardsMotor());

        // driver.back().whileTrue(new PathPlannerAuto("Left AnyTag"));

        // Operator Controls

        // Elevator
        operator.a().onTrue(elevatorSubsystem.bottomElevator());

        // Coral
        // operator.b().onTrue(coralSubsystem.lowerRampCommand());
        // operator.rightTrigger().onTrue(coralSubsystem.lowerRampCommand());
        // operator.rightTrigger().onTrue(extendClimb());

        // CHANGING BINDINGS

        // Vision = set mode | Manual = go to position
        // operator.povLeft().onTrue(
        // new ConditionalCommand(
        // new SetElevatorModeCmd(elevatorSubsystem,
        // Constants.elevatorConstants.L1Position),
        // elevatorSubsystem.goToL1Position(), () -> visionMode));
        // operator.povDown().onTrue(
        // new ConditionalCommand(
        // new SetElevatorModeCmd(elevatorSubsystem,
        // Constants.elevatorConstants.L2Position),
        // elevatorSubsystem.goToL2Position(), () -> visionMode));
        // operator.povRight().onTrue(
        // new ConditionalCommand(
        // new SetElevatorModeCmd(elevatorSubsystem,
        // Constants.elevatorConstants.L3Position),
        // elevatorSubsystem.goToL3Position(), () -> visionMode));
        // operator.povUp().onTrue(
        // new ConditionalCommand(
        // new SetElevatorModeCmd(elevatorSubsystem,
        // Constants.elevatorConstants.L4Position),
        // elevatorSubsystem.goToL4Position(), () -> visionMode));

        // delete once onTrue works
        operator.povLeft()
                .onTrue(elevatorSubsystem.goToL1Position());
        operator.povDown()
                .onTrue(elevatorSubsystem.goToL2Position());
        operator.povRight()
                .onTrue(elevatorSubsystem.goToL3Position());
        operator.povUp()
                .onTrue(elevatorSubsystem.goToL4Position());

        operator.start().whileTrue(elevatorSubsystem.slowL2());
        // operator.start().whileTrue(new

        operator.rightTrigger().whileTrue(m_climbSubsystem.climbCommand());
        operator.leftTrigger().whileTrue(m_climbSubsystem.extendingCommand());
    }

    public Command getAutonomousCommand() {

        // return Commands.sequence(new PathPlannerAuto("Copy of Start 11"));
        // return Commands.sequence(new PathPlannerAuto("11R"), new PathPlannerAuto("11R
        // TS 6"), new PathPlannerAuto("6L"),
        // new PathPlannerAuto("6L TS"), new PathPlannerAuto("6R"));
        return autoChooser.getSelected();
        // return new TwoPiece();
    }
}
