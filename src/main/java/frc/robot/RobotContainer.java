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
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Webcam;

public class RobotContainer {
    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final Climb m_climbSubsystem = Climb.getInstance();
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Limelight limelightSubsystem = Limelight.getInstance();
    private final Coral coralSubsystem = Coral.getInstance();
    private final Elevator elevatorSubsystem = Elevator.getInstance();
    private final LEDs s_LEDs = LEDs.getInstance();
    private final Webcam s_webcamSubsystem = Webcam.getInstance();
    private final Algae m_algaeSubsystem = Algae.getInstance();

    // Drive Controls
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final int elevatorAxis = XboxController.Axis.kLeftX.value;

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

        NamedCommands.registerCommand("Confirm Vision", new InstantCommand(() -> limelightSubsystem.pathIsDone(true))); // fix
        // better
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
                        new PathPlannerAuto("6R TS 6L"), new PathPlannerAuto("6L TS")));

        autoChooser.addOption("testing second half auto",
                Commands.sequence(new PathPlannerAuto("11R TS 6R"),
                        new PathPlannerAuto("6R TS 6L"), new PathPlannerAuto("6L TS")));

        autoChooser.addOption("Processor side 2/3 piece (COMPLETE)",
                Commands.sequence(new PathPlannerAuto("Start 9R"), new PathPlannerAuto("9R BS 8"),
                        new PathPlannerAuto("8R BS 8L"), new PathPlannerAuto("8L BS")));

        autoChooser.addOption("Middle 1 piece right (COMPLETE)", new PathPlannerAuto("MiddleRight"));
        autoChooser.addOption("Middle 1 piece left (COMPLETE)", new PathPlannerAuto("MiddleLeft"));

        autoChooser.addOption("Testing Elevatoring",
                Commands.sequence(new PathPlannerAuto("Start 11R"), elevatorSubsystem.bottomAutoElevator()));

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

        // Vision
        driver.a().onTrue(new InstantCommand(() -> toggleVisionMode()));
        // driver.a().onTrue(elevatorSubsystem.goToL1Position());

        // Elevator
        driver.y().whileTrue(coralSubsystem.forcedOuttakeCommand());

        // Coral
        driver.rightBumper().whileTrue(coralSubsystem.intakeCommand());
        driver.rightBumper().onTrue(elevatorSubsystem.bottomElevator());
        driver.b().whileTrue(coralSubsystem.backwardsMotor());
        // driver.start().onTrue(new InstantCommand(() ->
        // elevatorSubsystem.increaseMode()));
        // driver.back().onTrue(new InstantCommand(() ->
        // elevatorSubsystem.decreaseMode()));

        // scoring
        driver.rightTrigger()
                .whileTrue(new ConditionalCommand(Commands.sequence(
                        Commands.parallel(
                                new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve,
                                        Constants.Vision.rightOffset),
                                elevatorSubsystem.goToModePosition()),
                        coralSubsystem.outtakeCommand()).finallyDo((interrupted) -> {
                            elevatorSubsystem.bottomAutoElevator().schedule(); // Runs once when the button is released
                        }), new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve,
                                Constants.Vision.rightOffset),
                        () -> visionMode));

        driver.leftTrigger()
                .whileTrue(new ConditionalCommand(Commands.sequence(
                        Commands.parallel(
                                new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve,
                                        Constants.Vision.leftOffset),
                                elevatorSubsystem.goToModePosition()),
                        coralSubsystem.outtakeCommand()).finallyDo((interrupted) -> {
                            elevatorSubsystem.bottomAutoElevator().schedule(); // Runs once when the button is released
                        }), new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve,
                                Constants.Vision.leftOffset),
                        () -> visionMode));

        // driver.b()
        // .whileTrue(new SequentialCommandGroup(Commands.sequence(
        // elevatorSubsystem.goToL1Position(),
        // coralSubsystem.l1Command())));

        // driver.povUp().whileTrue(m_climbSubsystem.climbCommand());
        // driver.povDown().whileTrue(m_climbSubsystem.extendingCommand());

        // Operator Controls

        // Elevator
        operator.a().onTrue(elevatorSubsystem.bottomElevator());

        // operator.y().whileTrue(elevatorSubsystem.forcedUp());
        // operator.b().whileTrue(elevatorSubsystem.forcedDown());

        operator.povLeft()
                .onTrue(new ConditionalCommand(new InstantCommand(() -> elevatorSubsystem.setModeL1()),
                        elevatorSubsystem.goToL1Position(), () -> visionMode));
        operator.povDown()
                .onTrue(new ConditionalCommand(new InstantCommand(() -> elevatorSubsystem.setModeL2()),
                        elevatorSubsystem.goToL2Position(), () -> visionMode));
        operator.povRight()
                .onTrue(new ConditionalCommand(new InstantCommand(() -> elevatorSubsystem.setModeL3()),
                        elevatorSubsystem.goToL3Position(), () -> visionMode));
        operator.povUp()
                .onTrue(new ConditionalCommand(new InstantCommand(() -> elevatorSubsystem.setModeL4()),
                        elevatorSubsystem.goToL4Position(), () -> visionMode));

        operator.start().whileTrue(elevatorSubsystem.slowL2());

        operator.rightTrigger()
                .whileTrue(Commands.parallel(m_climbSubsystem.climbCommand(), coralSubsystem.outtakeCommand()));
        operator.leftTrigger()
                .whileTrue(Commands.parallel(m_climbSubsystem.extendingCommand(), coralSubsystem.outtakeCommand()));

        // extending
        operator.rightBumper().whileTrue(m_algaeSubsystem.algaeCommand(true));
        operator.leftBumper().whileTrue(m_algaeSubsystem.algaeCommand(false));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
