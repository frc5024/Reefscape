package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Elevator.SetElevatorSetpointCmd;
import frc.robot.commands.Vision.goToSetPositionPerTagCmd;
import frc.robot.commands.Vision.goToSetPositionPerTagOnTrueCmd;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Swerve s_Swerve = Swerve.getInstance();
    private final Limelight limelightSubsystem = new Limelight();
    private final Coral coralSubsystem = new Coral();
    private final Elevator elevatorSubsystem = Elevator.getInstance();
    private final LEDs s_LEDs = LEDs.getInstance();

    boolean visionMode = false;
    String mode;

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis), () -> false // true =
                                                                                                          // robotcentric

        ));

        // s_LEDs.setDefaultCommand().schedule();

        configureBindings();

        NamedCommands.registerCommand("Test Drive to AT",
                new goToSetPositionPerTagOnTrueCmd(limelightSubsystem, s_Swerve, Constants.Vision.noOffset));

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto/Chooser", autoChooser);
    }

    private void toggleVisionMode() {
        visionMode = !visionMode;

        if (visionMode) {
            mode = "Vision";
        } else {
            mode = "Manual";
        }

        SmartDashboard.putString("Robot Mode", mode);

        configureBindings();
    }

    private void configureBindings() {
        // (driver)
        driver.x().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // driver.a().onTrue(new InstantCommand(() -> toggleVisionMode()));

        driver.leftBumper().onTrue(new InstantCommand(() -> s_Swerve.isSlowMode = true));
        driver.leftBumper().onFalse(new InstantCommand(() -> s_Swerve.isSlowMode = false));

        // two intake commands
        driver.rightBumper().whileTrue(coralSubsystem.intakeCommand());
        driver.rightBumper().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem,
                Constants.elevatorConstants.rootPosition));

        // (operator)
        operator.b().onTrue(coralSubsystem.lowerRampCommand());

        // potential binding fix
        // operator.b().onTrue(new ConditionalCommand(getAutonomousCommand(),
        // getAutonomousCommand(), () -> visionMode));

        // operator.a().onTrue(new SetElevatorSetpointCmd(elevatorSubsystem,
        // Constants.elevatorConstants.zeroPosition));

        driver.rightTrigger().whileTrue(new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve, 0));

        // (operator)
        driver.povLeft()
                .whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L1Position));
        driver.povDown()
                .whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L2Position));
        driver.povRight()
                .whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L3position));
        driver.povUp()
                .whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L4position));

        operator.rightTrigger().onTrue(coralSubsystem.lowerRampCommand());
        // operator.rightTrigger().onTrue(extendClimb());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
