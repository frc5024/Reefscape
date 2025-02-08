package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Servo.ServoDesired;
import frc.robot.commands.Servo.ServoEase;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ServoTest;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Swerve s_Swerve = Swerve.getInstance();
    private final LEDs s_LEDs = LEDs.getInstance();
    private final ServoTest s_Servo = ServoTest.getInstance();
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // Driver/TestLEDs buttons
    // private final Trigger changeRainbow = driver.x();// Sets Which button is in
    // this case x
    // pressed in order for the Command to funtion
    // private final Trigger testFlash = driver.y();
    private final Trigger Servo90 = driver.a();// Sets to controller A
    private final Trigger Servo0 = driver.b();// Sets to controller B
    private final Trigger Servo180 = driver.x();// Sets to X
    private final Trigger ServoEase = driver.y();// Sets to Y
    // private final Trigger LED1 = driver.x();
    // private final Trigger LED2 = driver.y();
    // private final Trigger LED3 = driver.a();
    // private final Trigger LED4 = driver.b();

    public RobotContainer() {

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis), () -> false // true =
                                                                                                          // robotcentric

        ));

        configureBindings();

    }

    private void configureBindings() {
        // changeRainbow.whileTrue(new TestLEDs());
        // driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        Servo0.whileTrue(new ServoDesired(0));
        Servo90.whileTrue(new ServoDesired(90));
        Servo180.whileTrue(new ServoDesired(180));
        ServoEase.whileTrue(new ServoEase(180));
        // LED1.whileTrue(new TestFlashLEDs(s_LEDs, LEDPreset.Solid.kRed, 1000));
        // LED2.whileTrue(new TestFlashLEDs(s_LEDs, LEDPreset.Solid.kLawnGreen, 1000));
        // LED3.whileTrue(new TestFlashLEDs(s_LEDs, LEDPreset.Solid.kLime, 1000));
        // LED4.whileTrue(new TestFlashLEDs(s_LEDs, LEDPreset.Solid.kGreen, 1000));

    }
}
