package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.CommandGroups.CoralScored;
import frc.robot.commands.CommandGroups.LowerRampClimb;
import frc.robot.commands.SetElevatorSetpointCmd;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Coral;


public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  //private final Swerve s_Swerve = Swerve.getInstance();
  private final Elevator elevatorSubsystem = new Elevator();
  private final Coral coralSubsystem = new Coral();
  private final Climb climbSubsystem = new Climb();

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  public RobotContainer() {

    // s_Swerve.setDefaultCommand(
    //             new TeleopSwerve(
    //                     s_Swerve,
    //                     () -> -driver.getRawAxis(translationAxis),
    //                     () -> -driver.getRawAxis(strafeAxis),
    //                     () -> -driver.getRawAxis(rotationAxis),
    //                     () -> false // true = robotcentric

    //             ));


    configureBindings();

  }

  private void configureBindings() {
    //buttons for elevator positions
    //operator.b().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L1Position));

    //COMMAND GROUPS//
    operator.b().onTrue(new CoralScored(coralSubsystem, elevatorSubsystem));
    operator.rightTrigger().onTrue(new LowerRampClimb(coralSubsystem, climbSubsystem));


    operator.a().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L2Position));
    operator.x().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L4position));
    operator.y().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L3position));
    
    driver.a().onTrue(coralSubsystem.intakeCommand());
    driver.b().onTrue(coralSubsystem.outtakeCommand());
    driver.y().onTrue(coralSubsystem.cancelIntakeCommand());
    driver.x().onTrue(coralSubsystem.plopCommand());
    driver.rightBumper().onTrue(coralSubsystem.lowerRampCommand());

    //driver.rightTrigger().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.rootPosition));
    // driver.leftBumper().whileTrue(new RunCommand(() ->
    // { 
    //   elevatorSubsystem.controlMotor(0.2);
    // }).finallyDo( ()-> 
    // {
    //   elevatorSubsystem.controlMotor(0.0);
    // }));
    driver.rightBumper().whileTrue(new RunCommand(() -> elevatorSubsystem.controlMotor(-0.1)).finallyDo( ()-> elevatorSubsystem.controlMotor(0.0)));
    //operator.y().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.Algae2));
    //operator.x().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L4position));
    //driver.x().whileTrue(new elevatorCmd(elevatorSubsystem, true) );
    //driver.a().whileTrue(new elevatorCmd(elevatorSubsystem, false));
    //driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    //driver.getLeftX().whileTrue()
  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
