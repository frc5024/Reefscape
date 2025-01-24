package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestLEDs;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Swerve s_Swerve = Swerve.getInstance();
    private final LEDs s_LEDs = LEDs.getInstance();
    private final Shooter s_Shooter = Shooter.getInstance();

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

        configureBindings();

    }

    private void configureBindings() {
        changeRainbow.whileTrue(new TestLEDs());
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }

    public Command getAutonomousCommand() {
        return Autos.exampleAuto(m_exampleSubsystem);
    }
}
