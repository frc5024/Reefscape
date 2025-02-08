package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.goToSetPositionPerTagCmd;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

    private final CommandXboxController driver = new CommandXboxController(0);
    // private final CommandXboxController operator = new CommandXboxController(1);

    private final Swerve s_Swerve = Swerve.getInstance();
    private final Limelight limelightSubsystem = new Limelight();

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis), () -> false // true =
                                                                                                          // robotcentric

        ));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto/Chooser", autoChooser);
    }

    private void configureBindings() {

        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        driver.a().whileTrue(new goToSetPositionPerTagCmd(limelightSubsystem, s_Swerve, Constants.Vision.noOffset));

        // driver.a().whileTrue(new FaceHeadingCmd(s_Swerve));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
