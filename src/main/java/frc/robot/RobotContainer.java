package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.SetElevatorSetpointCmd;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
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

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final Coral coralSubsystem = new Coral();

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
    driver.b().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L1Position));
    driver.a().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L2Position));
    driver.x().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L4position));
    driver.y().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.L3position));

    driver.rightBumper().onTrue(new InstantCommand (() -> elevatorSubsystem.zeroEncoderValue()));
  
    //driver.rightTrigger().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.rootPosition));
    // driver.leftBumper().whileTrue(new RunCommand(() ->
    // { 
    //   System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
    //   elevatorSubsystem.motor1Manual();
    // }).finallyDo( ()-> 
    // {
    //   System.out.println("DONEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
    //   elevatorSubsystem.elevatorMotor.set(0);
    // }));
    // driver.rightBumper().whileTrue(new RunCommand(() -> elevatorSubsystem.motor2Manual()).finallyDo( ()-> elevatorSubsystem.elevatorMotor2.set(0.0)));
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
