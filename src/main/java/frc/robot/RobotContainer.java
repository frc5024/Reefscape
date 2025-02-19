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
    private final Elevator elevatorSubsystem = new Elevator();
    private final LEDs s_LEDs = LEDs.getInstance();

    boolean visionMode = false;

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
        // (driver)
        driver.x().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        driver.a().onTrue(new InstantCommand(() -> visionMode = !visionMode));

        driver.leftBumper().onTrue(new InstantCommand(() -> s_Swerve.toggleSlowmode()));

        driver.rightBumper().whileTrue(coralSubsystem.intakeCommand());
        // driver.rightBumper().onTrue(new SetElevatorSetpointCmd(elevatorSubsystem,
        // Constants.elevatorConstants.zeroPosition));

        // (operator)
        operator.b().onTrue(coralSubsystem.lowerRampCommand());

        // vision
        if (visionMode) {
            // (driver)
            driver.b().whileTrue(new goToSetPositionPerTagCmd(
                    limelightSubsystem, s_Swerve, Constants.Vision.noOffset));

            // (operator)
        }

        // manual
        if (!visionMode) {
            // (driver)
            driver.rightTrigger().onTrue(coralSubsystem.outtakeCommand());

            // (operator)
            operator.b()
                    .whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L1Position));
            operator.a()
                    .whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L2Position));
            operator.x()
                    .whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L3position));
            operator.y()
                    .whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L4position));
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
