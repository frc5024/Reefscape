package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.Servo0;
import frc.robot.commands.Servo90;
import frc.robot.commands.ServoEase;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ServoTest;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Swerve s_Swerve = Swerve.getInstance();
    private final LEDs s_LEDs = LEDs.getInstance();
    private final ServoTest s_Servo = ServoTest.getInstance();
    private final Shooter s_Shooter = Shooter.getInstance();

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // Driver/TestLEDs buttons
    // private final Trigger changeRainbow = driver.x();// Sets Which button is in
    // this case x
    // pressed in order for the Command to funtion
    // private final Trigger testFlash = driver.y();
    private final Trigger Servo90 = driver.a();
    private final Trigger Servo0 = driver.b();
    private final Trigger Servo180 = driver.x();
    private final Trigger ServoEase = driver.y();
    // private final Trigger LED1 = driver.x();
    // private final Trigger LED2 = driver.x();
    // private final Trigger LED3 = driver.x();
    // private final Trigger LED4 = driver.x();

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
        Servo0.whileTrue(new Servo0());
        Servo90.whileTrue(new Servo90());
        Servo180.whileTrue(new ServoEase(90));
        ServoEase.whileTrue(new ServoEase(180));
        // LED1.whileTrue(new TestFlashLEDs(s_LEDs, LEDPreset.Solid.kRed, 1000));
        // LED2.whileTrue(new TestFlashLEDs(s_LEDs, LEDPreset.Solid.kLawnGreen, 1000));
        // LED3.whileTrue(new TestFlashLEDs(s_LEDs, LEDPreset.Solid.kLime, 1000));
        // LED4.whileTrue(new TestFlashLEDs(s_LEDs, LEDPreset.Solid.kGreen, 1000));

    }

    public Command getAutonomousCommand() {
        return Autos.exampleAuto(m_exampleSubsystem);
    }
}
