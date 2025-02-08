package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.goToSetPositionPerTagCmd;
import frc.robot.commands.SetElevatorSetpointCmd;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Coral;


public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final Swerve s_Swerve = Swerve.getInstance();

  private final Elevator elevatorSubsystem = new Elevator();

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final Coral coralSubsystem = new Coral();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis)
                ));


    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto/Chooser", autoChooser);
  }

  private void configureBindings() {
    //buttons for elevator positions
    operator.a().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L1Position));
    operator.x().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L2Position));
    operator.b().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L3position));
    operator.y().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L4position));

    driver.rightBumper().onTrue(coralSubsystem.intakeCommand());
    driver.b().onTrue(coralSubsystem.cancelIntakeCommand());
    driver.rightTrigger().onTrue(coralSubsystem.outtakeCommand());

    driver.leftBumper().onTrue(coralSubsystem.lowerRampCommand());
    
    driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
