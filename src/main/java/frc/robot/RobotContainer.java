package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.leds.LEDPreset;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Servo.ServoDesired;
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
        driver.a().onTrue(new ServoDesired(0, s_LEDs, LEDPreset.Solid.kBlue, 10));
        driver.b().onTrue(new ServoDesired(90, s_LEDs, LEDPreset.Solid.kBlue, 10));
        driver.x().onTrue(new ServoDesired(180, s_LEDs, LEDPreset.Solid.kBlue, 10));
        driver.y().onTrue(s_LEDs.setDefaultCommand());
    }
}