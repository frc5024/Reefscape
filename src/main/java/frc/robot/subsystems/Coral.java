package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.commands.coral.CancelIntakeCommand;
import frc.robot.commands.coral.IntakeCommand;
import frc.robot.commands.coral.LowerRampCommand;
import frc.robot.commands.coral.OuttakeCommand;
import frc.robot.commands.coral.PlopCommand;

public class Coral extends SubsystemBase {
    // motor controller for coral
    private SparkFlex coralMotor;
    private SparkFlex coralMotorReversed;

    // private final SparkBaseConfig coralMotorReversedConfig = new
    // SparkFlexConfig()
    // .inverted(true)
    // .follow(coralConstants.coralMotorChannel);

    private static DigitalInput linebreak;

    // all constants for coral
    int coralMotorChannel = CoralConstants.coralMotorChannel;
    int coralMotorReversedChannel = CoralConstants.coralMotorReversedChannel;
    int linebreakChannel = CoralConstants.linebreakChannel;

    double intakeSpeed = CoralConstants.intakeSpeed;
    double outtakeSpeed = CoralConstants.outtakeSpeed;
    double plopSpeed = CoralConstants.plopSpeed;

    // double servoRotate = CoralConstants.servoRotate;
    // double servoReset = CoralConstants.servoReset;

    // shuffleboard tab for coral
    ShuffleboardTab tab = Shuffleboard.getTab("CoralMotors");
    GenericEntry intakeMotorSpeedEntry = tab.add("SET intake speed", intakeSpeed).getEntry();
    GenericEntry outtakeMotorSpeedEntry = tab.add("SET outtake speed", outtakeSpeed).getEntry();
    GenericEntry plopSpeedEntry = tab.add("SET plop speed", plopSpeed).getEntry();

    // constructor for coralMotor
    public Coral() {
        linebreak = new DigitalInput(CoralConstants.linebreakChannel);
        tab.addBoolean("linebreak", () -> linebreak.get());

        coralMotor = new SparkFlex(coralMotorChannel, SparkFlex.MotorType.kBrushless);
        tab.addDouble("motor speed", () -> coralMotor.get());

        coralMotorReversed = new SparkFlex(coralMotorReversedChannel, SparkFlex.MotorType.kBrushless);
        tab.addDouble("bottom motor speed", () -> coralMotorReversed.get());

        tab.addDouble("encoder value", () -> getEncoder());

        // this.coralMotorReversed.configure(coralMotorReversedConfig,
        // ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    // method for intaking coral, takes in a boolean to determine if the coral
    // should intake

    @Override
    public void periodic() {
        intakeSpeed = intakeMotorSpeedEntry.getDouble(intakeSpeed);
        outtakeSpeed = outtakeMotorSpeedEntry.getDouble(outtakeSpeed);
        plopSpeed = plopSpeedEntry.getDouble(plopSpeed);
    }

    // idle state, set motor to 0
    public void setIdle() {
        coralMotor.set(0);
        coralMotorReversed.set(0);
    }

    public boolean isLineBroken() {
        return linebreak.get();
    }

    public double getEncoder() {
        // Combine or choose one encoder value to return
        // return (coralMotor.getAbsoluteEncoder().getPosition() +
        // coralMotorReversed.getAbsoluteEncoder().getPosition()) / 2;
        return coralMotorReversed.getAbsoluteEncoder().getPosition(); // getExternalEncoder() or getAbsoluteEncoder(),
                                                                      // not sure yet
    }

    public void set(double speed) {
        setTop(-speed);
        setBottom(speed);
    }

    public void setTop(double speed) {
        coralMotor.set(speed);
    }

    public void setBottom(double speed) {
        coralMotorReversed.set(speed);
    }

    public Command plopCommand() {
        return new PlopCommand(this);
    }

    public Command forcedOuttakeCommand() {
        return new PlopCommand(this);
    }

    public Command intakeCommand() {
        return new IntakeCommand(this);
    }

    public Command outtakeCommand() {
        return new OuttakeCommand(this, false);
    }

    public Command outtakeAutoCommand() {
        return new OuttakeCommand(this, true);
    }

    public Command cancelIntakeCommand() {
        return new CancelIntakeCommand(this);
    }

    public Command backwardsMotor() {
        return new LowerRampCommand(this);
    }

    // ---------------SERVO STUFF --------------
    // rotate servo 90 degrees (lowers the ramp)
    // public void lowerRamp() {
    // rampServo.set(servoRotate);
    // }
    // rotates servo backwards 90 degrees (raises the ramp back to original
    // position)
    // public void resetRamp() {
    // rampServo.set(servoReset);
    // }
    // //sets servo to 0 (for initialization)
    // public void setRamp() {
    // rampServo.set(0);
    // }
}