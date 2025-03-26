package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;
import frc.robot.commands.Climb.ArmOutCmd;
import frc.robot.commands.Climb.ClimbCommand;

public class Climb extends SubsystemBase {
    public static Climb mInstance = null;

    private TalonFX climbMotor;
    private DigitalInput limitSwitch = new DigitalInput(8);

    private Webcam webcamSubsystem;

    ShuffleboardTab tab = Shuffleboard.getTab("Climb");

    public boolean extended = false;

    // Creating the Climb instance
    public static Climb getInstance() {
        if (mInstance == null) {
            mInstance = new Climb();
        }
        return mInstance;
    }

    private Climb() {
        // Creating motor
        climbMotor = new TalonFX(Constants.ClimbConstants.climbMotorID);
        // Set to brake
        climbMotor.setNeutralMode(NeutralModeValue.Brake);

        // Shuffleboard tab displaying the encoder's position value as a double
        tab.addDouble("encoder value", () -> climbMotor.getPosition().getValueAsDouble());
        tab.addBoolean("is climbed?", () -> isLimitSwitchBroken());
        tab.addDouble("voltage", () -> climbMotor.getMotorVoltage().getValueAsDouble());
    }

    // arm retracts with blue LEDs
    public void retractArm() {
        climbMotor.set(Constants.ClimbConstants.climbSpeed);

        LEDs.getInstance().setCommand(LEDPreset.Rainbow.kConfetti).schedule();
    }

    public void extendArm() {
        // Sets motor to extending speed
        climbMotor.set(Constants.ClimbConstants.extendSpeed);
        LEDs.getInstance().setCommand(LEDPreset.Solid.kViolet).schedule();
    }

    public void moveMotor(double Speed) {
        climbMotor.set(Speed);
    }

    public void stopMotor() {
        climbMotor.set(0);
    }

    public void slowClimb() {
        climbMotor.set(0.3);
    }

    public boolean isLimitSwitchBroken() {
        return !limitSwitch.get();
    }

    // checks if arm is extended
    public boolean isEncoderExtended() {
        return climbMotor.getPosition().getValueAsDouble() >= Constants.ClimbConstants.extendedPosition;
    }

    public double getEncoder() {
        return climbMotor.getPosition().getValueAsDouble();
    }

    // Commands

    public Command climbCommand() {
        return new ClimbCommand(this);
    }

    public Command extendingCommand() {
        return new ArmOutCmd(this);
    }

    @Override
    public void periodic() {

        Logger.recordOutput("Subsystems/Climb/Is Climbed", isLimitSwitchBroken());
        Logger.recordOutput("Subsystems/Climb/Motor Velocity", climbMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Subsystems/Climb/Motor Voltage", climbMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Subsystems/Climb/Position", climbMotor.getPosition().getValueAsDouble());

        SmartDashboard.putBoolean("Climb LimitSwitch", isLimitSwitchBroken());
    }
}