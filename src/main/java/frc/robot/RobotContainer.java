package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.leds.LEDPreset;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.LEDs.SetLEDS;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Swerve s_Swerve = Swerve.getInstance();
    private final LEDs s_LEDs = LEDs.getInstance();

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    public RobotContainer() {

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis), () -> false));

        s_LEDs.setDefault();
        configureBindings();
    }

    private void configureBindings() {
        // driver.a().whileTrue(new SetLEDS(s_LEDs, LEDPreset.Heartbeat.kBlue));// Sets
        // to blue
        // driver.b().whileTrue(new SetLEDSDefault(s_LEDs));// Sets to Default colour
        // (Find in Constants)
        // driver.x().whileTrue(new FlashLEDS(s_LEDs, LEDPreset.Solid.kGreen, 1));//
        // Flashes Green for 1 second
        driver.b().whileTrue(new SetLEDS(s_LEDs, LEDPreset.BeatsPerMinute.kRainbow));
        driver.x().whileTrue(new SetLEDS(s_LEDs, LEDPreset.Twinkles.kLava));
        driver.y().whileTrue(new SetLEDS(s_LEDs, LEDPreset.BeatsPerMinute.kOcean));
        driver.a().whileTrue(new SetLEDS(s_LEDs, LEDPreset.BeatsPerMinute.kParty));
    }
}
