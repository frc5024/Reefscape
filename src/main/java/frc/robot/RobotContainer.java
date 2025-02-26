package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LEDs;

public class RobotContainer {
    private final Climb m_climbSubsystem = Climb.getInstance();

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // private final Swerve s_Swerve = Swerve.getInstance();
    private final LEDs s_LEDs = LEDs.getInstance();

    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;

    public RobotContainer() {

        // s_Swerve.setDefaultCommand(
        // new TeleopSwerve(
        // s_Swerve,
        // () -> -driver.getRawAxis(translationAxis),
        // () -> -driver.getRawAxis(strafeAxis),
        // () -> -driver.getRawAxis(rotationAxis),
        // () -> false // true = robotcentric

        s_LEDs.setDefault();
        configureBindings();
    }

    private void configureBindings() {

        // Climb commands
        // driver.y().whileTrue(m_climbSubsystem.climbCommand(s_LEDs));
        driver.rightTrigger().whileTrue(m_climbSubsystem.extendingCommand());
        driver.leftTrigger().whileTrue(m_climbSubsystem.retractingCommand());
        // driver.b().onTrue(m_climbSubsystem.cancelCommand());
        // driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // driver.().onTrue(new SetLEDS(s_LEDs, LEDPreset.Solid.kBlue));// Sets to blue
        // driver.b().onTrue(new SetLEDSDefault(s_LEDs));// Sets to Default colour (Find
        // in Constants)
        // driver.x().onTrue(new FlashLEDS(s_LEDs, LEDPreset.Solid.kGreen, 1));//
        // Flashes Green for 1 second
    }
}
