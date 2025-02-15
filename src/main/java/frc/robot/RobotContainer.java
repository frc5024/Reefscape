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
import frc.robot.commands.Vision.FaceHeadingCmd;
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
    private final Elevator elevatorSubsystem = new Elevator();
    private final LEDs s_LEDs = LEDs.getInstance();

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

    private void configureBindings() {

        // Vision and swerve
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        driver.b().whileTrue(new goToSetPositionPerTagCmd(
                limelightSubsystem, s_Swerve, Constants.Vision.noOffset));

        // driver.b().whileTrue(new goToSetPositionPerTagOnTrueCmd(
        // limelightSubsystem, s_Swerve, Constants.Vision.noOffset));

        driver.x().whileTrue(new FaceHeadingCmd(s_Swerve));

        // Coral
        driver.rightBumper().onTrue(coralSubsystem.intakeCommand());
        driver.rightTrigger().onTrue(coralSubsystem.outtakeCommand());
        driver.leftBumper().onTrue(coralSubsystem.cancelIntakeCommand());
        driver.leftBumper().onTrue(coralSubsystem.lowerRampCommand());
        // driver.x().onTrue(coralSubsystem.plopCommand());

        // Elevator (operator)
        operator.b().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L1Position));
        operator.a().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L2Position));
        operator.x().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L3position));
        operator.y().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L4position));

        operator.rightBumper().onTrue(new InstantCommand(() -> elevatorSubsystem.zeroEncoderValue()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
