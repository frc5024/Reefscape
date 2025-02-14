package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.leds.LEDPreset;
import frc.robot.commands.ClimbCancelCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbExtendoCommand;
import frc.robot.commands.LEDs.flashLEDS;
import frc.robot.commands.LEDs.setLEDS;
import frc.robot.commands.LEDs.setLEDSDefault;
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

        s_LEDs.setLEDSDefault();
        configureBindings();
    }

    private void configureBindings() {
        driver.a().onTrue(new setLEDS(s_LEDs, LEDPreset.Solid.kBlue));// Sets to blue
        driver.b().onTrue(new setLEDSDefault(s_LEDs));// Sets to Default colour (Find in Constants)
        driver.x().onTrue(new flashLEDS(s_LEDs, LEDPreset.Solid.kGreen, 1));// Flashes Green for 1 second

        driver.a().onTrue(new ClimbCommand(m_climbSubsystem));
        driver.x().onTrue(new ClimbExtendoCommand(m_climbSubsystem, true));
        driver.y().onTrue(new ClimbExtendoCommand(m_climbSubsystem, false));
        driver.b().onTrue(new ClimbCancelCommand(m_climbSubsystem));
        // driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }
}
