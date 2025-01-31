package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Coral;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.testIntakeCommand;
import frc.robot.commands.ServoCommand;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final Swerve s_Swerve = Swerve.getInstance();

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final Coral coralSubsystem = new Coral();

  public RobotContainer() {

    s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> false // true = robotcentric

                ));


    configureBindings();

  }

  private void configureBindings() {

    driver.a().whileTrue(new IntakeCommand(coralSubsystem));
    driver.b().onTrue(new OuttakeCommand(coralSubsystem));
    driver.x().whileTrue(new testIntakeCommand(coralSubsystem));
    driver.rightBumper().onTrue(new ServoCommand(coralSubsystem));
    driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
