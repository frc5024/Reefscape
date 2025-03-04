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
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Vision.autoSetPositionTagID;
import frc.robot.commands.Vision.goToSetPositionPerTagCmd;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Rumble;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final Climb m_climbSubsystem = Climb.getInstance();
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Rumble rumble = Rumble.getInstance();
    private final Limelight limelightSubsystem = new Limelight();
    private final Coral coralSubsystem = new Coral();
    private final Elevator elevatorSubsystem = Elevator.getInstance();
    private final LEDs s_LEDs = LEDs.getInstance();

    // Drive Controls
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // Vision Variables
    boolean visionMode = true;
    String mode;

    // Auto
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis), () -> false // true = robotcentric
        ));

        // s_LEDs.setDefaultCommand().schedule();

        configureBindings();

        // Auto Commands
        // Vision
        NamedCommands.registerCommand("DriveRightTag", new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve, Constants.Vision.rightOffset));
        NamedCommands.registerCommand("DriveLeftTag", new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve, Constants.Vision.leftOffset));
        NamedCommands.registerCommand("DriveLeftTag11", new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.leftOffset, 11));
        NamedCommands.registerCommand("DriveRightTag11", new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.rightOffset, 11));
        NamedCommands.registerCommand("DriveRightTag6", new autoSetPositionTagID(limelightSubsystem, s_Swerve, Constants.Vision.rightOffset, 6));

        // Coral
        NamedCommands.registerCommand("ScoreCoral", coralSubsystem.outtakeCommand());
        NamedCommands.registerCommand("IntakeCoral", coralSubsystem.intakeCommand());

        // Elevator
        NamedCommands.registerCommand("L4", elevatorSubsystem.goToL4Position());
        NamedCommands.registerCommand("L3", elevatorSubsystem.goToL3Position());
        NamedCommands.registerCommand("L2", elevatorSubsystem.goToL2Position());
        NamedCommands.registerCommand("L1", elevatorSubsystem.goToL1Position());
        NamedCommands.registerCommand("ElevatorRoot", elevatorSubsystem.bottomElevator());

        // Auto Chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto/Chooser", autoChooser);
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
        driver.leftBumper().onTrue(new InstantCommand(() -> s_Swerve.isSlowMode = true));
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

        // Climb
        driver.y().whileTrue(m_climbSubsystem.climbCommand());
        driver.rightTrigger().whileTrue(m_climbSubsystem.extendingCommand());
        driver.leftTrigger().whileTrue(m_climbSubsystem.retractingCommand());

        // Elevator
        driver.b().whileTrue(new ConditionalCommand(elevatorSubsystem.goToModePosition(), new InstantCommand(), () -> visionMode));

        // Coral
        driver.rightBumper().whileTrue(coralSubsystem.intakeCommand());
        driver.rightBumper().whileTrue(elevatorSubsystem.bottomElevator()); // fix to onTrue

        // Operator Controls

        // Elevator
        operator.a().whileTrue(elevatorSubsystem.bottomElevator()); // fix to onTrue

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
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(new PathPlannerAuto("Auto1"), new PathPlannerAuto("Auto2"));
        // autoChooser.
        // return autoChooser.getSelected();
        // return new TwoPiece();
    }
}
