package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestFlashLEDs;
import frc.robot.commands.TestLEDs;
import frc.robot.subsystems.AlgaeCommandBased;
import frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Swerve s_Swerve = Swerve.getInstance();
    // private final LEDs s_LEDs = LEDs.getInstance();
    private final AlgaeCommandBased s_Algae = AlgaeCommandBased.getInstance();

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // Driver/TestLEDs buttons
    private final Trigger changeRainbow = driver.x();// Sets Which button is pressed in order for the Command to funtion
    private final Trigger testFlash = driver.y();

    public RobotContainer() {

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis), () -> false // true =
                                                                                                          // robotcentric

        ));

        // Testing Algae Command - Uncomment this for manual control of Algae subsystem
        // s_Algae.setDefaultCommand(new AlgaeManualCommand(s_Algae, () ->
        // driver.getLeftTriggerAxis(), () -> driver.getRightTriggerAxis()));

        // s_Algae.setDefaultCommand(new AlgaeStateCommand(s_Algae, () ->
        // driver.getRightTriggerAxis()));

        configureBindings();
        // Sets the controllers triggers to be used by algae command

    }

    // Binding a key to each algae command
    private void configureBindings() {
        changeRainbow.whileTrue(new TestLEDs());
        testFlash.whileTrue(new TestFlashLEDs());
        driver.y().onTrue(s_Algae.intake());
        driver.x().onTrue(s_Algae.launch());
        driver.a().onTrue(s_Algae.drop());
        driver.b().whileTrue(s_Algae.cancel());

    }

    public Command getAutonomousCommand() {
        return Autos.exampleAuto(m_exampleSubsystem);
    }

}
