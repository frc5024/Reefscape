package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.LEDs.flashLEDS;
import frc.robot.commands.LEDs.setLEDS;
import frc.robot.commands.LEDs.setLEDSDefault;
import frc.robot.subsystems.LEDPreset;
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

        s_LEDs.setLEDSDefault();
        configureBindings();
    }

    private void configureBindings() {
        driver.a().onTrue(new setLEDS(s_LEDs, LEDPreset.Solid.kBlue));// Sets to blue
        driver.b().onTrue(new setLEDSDefault(s_LEDs));// Sets to Default colour (Find in Constants)
        driver.x().onTrue(new flashLEDS(s_LEDs, LEDPreset.Solid.kGreen, 1));// Flashes Green for 1 second
    }
}
