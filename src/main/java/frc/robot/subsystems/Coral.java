package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Coral.CancelIntakeCommand;
import frc.robot.commands.Coral.ForcedOuttakeCmd;
import frc.robot.commands.Coral.IntakeCommand;
import frc.robot.commands.Coral.LowerRampCommand;
import frc.robot.commands.Coral.OuttakeCommand;
import frc.robot.commands.Coral.PlopCommand;

public class Coral extends SubsystemBase {
    private static Coral mInstance;

    public static Coral getInstance() {
        if (mInstance == null) {
            mInstance = new Coral();
        }
        return mInstance;
    }

    // motor controller for coral
    private SparkFlex coralMotor;
    private SparkFlex coralMotorReversed;

    // private final SparkBaseConfig coralMotorReversedConfig = new
    // SparkFlexConfig()
    // .inverted(true)
    // .follow(coralConstants.coralMotorChannel);

    private DigitalInput linebreak;

    // all constants for coral
    int coralMotorChannel = Constants.coralConstants.coralMotorChannel;
    int coralMotorReversedChannel = Constants.coralConstants.coralMotorReversedChannel;
    int linebreakChannel = Constants.coralConstants.linebreakChannel;

    double intakeSpeed = Constants.coralConstants.intakeSpeed;
    double outtakeSpeed = Constants.coralConstants.outtakeSpeed;
    double plopSpeed = Constants.coralConstants.plopSpeed;

    private Elevator elevatorSubsystem;

    // double servoRotate = Constants.coralConstants.servoRotate;
    // double servoReset = Constants.coralConstants.servoReset;

    // shuffleboard tab for coral
    ShuffleboardTab tab = Shuffleboard.getTab("CoralMotors");
    // GenericEntry intakeMotorSpeedEntry = tab.add("SET intake speed",
    // intakeSpeed).getEntry();
    // GenericEntry outtakeMotorSpeedEntry = tab.add("SET outtake speed",
    // outtakeSpeed).getEntry();
    // GenericEntry plopSpeedEntry = tab.add("SET plop speed",
    // plopSpeed).getEntry();

    // constructor for coralMotor
    private Coral() {
        linebreak = new DigitalInput(Constants.coralConstants.linebreakChannel);
        tab.addBoolean("linebreak (Has Coral)", () -> linebreak.get());

        coralMotor = new SparkFlex(coralMotorChannel, SparkFlex.MotorType.kBrushless);
        tab.addDouble("motor speed", () -> coralMotor.get());

        coralMotorReversed = new SparkFlex(coralMotorReversedChannel, SparkFlex.MotorType.kBrushless);
        tab.addDouble("bottom motor speed", () -> coralMotorReversed.get());

        tab.addDouble("encoder value", () -> getEncoder());
        tab.addDouble("applied output", () -> coralMotor.getAppliedOutput());

        // this.coralMotorReversed.configure(coralMotorReversedConfig,
        // ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    // method for intaking coral, takes in a boolean to determine if the coral
    // should intake

    @Override
    public void periodic() {
        // intakeSpeed = intakeMotorSpeedEntry.getDouble(intakeSpeed);
        // outtakeSpeed = outtakeMotorSpeedEntry.getDouble(outtakeSpeed);
        // plopSpeed = plopSpeedEntry.getDouble(plopSpeed);

        // Logger.recordOutput("Subsystems/Coral/HasCoral", linebreak.get());
        // Logger.recordOutput("Subsystems/Coral/Motor Position",
        // coralMotor.getEncoder().getPosition());
        // Logger.recordOutput("Subsystems/Coral/Motor Velocity",
        // coralMotor.getEncoder().getVelocity());
        // Logger.recordOutput("Subsystems/Coral/Motor Voltage",
        // coralMotor.getAppliedOutput());
    }

    // idle state, set motor to 0
    public void setIdle() {
        coralMotor.set(0);
        coralMotorReversed.set(0);
    }

    public boolean isLineBroken() {
        return linebreak.get();
    }

    // public boolean isBlocked() {
    // if(coralMotor.getAppliedOutput > number){
    // return true;
    // } else {
    // return false;
    // }
    // }

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
        return new ForcedOuttakeCmd(this, elevatorSubsystem);
    }

    public Command intakeCommand() {
        return new IntakeCommand(this);
    }

    public Command outtakeCommand() {
        return new OuttakeCommand(this, elevatorSubsystem, false);
    }

    public Command outtakeAutoCommand() {
        return new OuttakeCommand(this, elevatorSubsystem, true);
    }

    public Command cancelIntakeCommand() {
        return new CancelIntakeCommand(this);
    }

    public Command backwardsMotor() {
        return new LowerRampCommand(this);
    }

    // public Command l1Command() {
    // return new L1Command(this);
    // }

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