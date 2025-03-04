package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.Climb.ClimbCommand;
import frc.robot.commands.Climb.ClimbExtendoCommand;
import frc.robot.commands.Climb.ClimbRetractCommand;

public class Climb extends SubsystemBase {
    public static Climb mInstance = null;

    private TalonFX climbMotor;
    private DigitalInput limitSwitch = new DigitalInput(0);

    // Shuffleboard
    ShuffleboardTab tab = Shuffleboard.getTab("Climb");
    // Not currently being used
    GenericEntry encoder = tab.add("climbSpeed", .35).getEntry();

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

        // Shuffleboard tab displaying the encoder's position value as a double
        tab.addDouble("encoder value", () -> climbMotor.getPosition().getValueAsDouble());
        tab.addBoolean("is climbed?", () -> isClimbed());
        tab.addDouble("voltage", () -> climbMotor.getMotorVoltage().getValueAsDouble());
    }

    public void climbing() {
        // Sets motor to climbing speed
        climbMotor.set(Constants.ClimbConstants.climbSpeed);
        // Schedules the LED command
        LEDs.getInstance().setCommand(LEDPreset.LightChase.kBlue).schedule();
        System.out.println(climbMotor.getMotorVoltage());
        // }
    }

    public void extending() {
        // Sets motor to extending speed
        climbMotor.set(Constants.ClimbConstants.extendoSpeed);
        LEDs.getInstance().setCommand(LEDPreset.Solid.kViolet).schedule();
    }

    public void retracting() {
        // Sets motor to reverse of extending speed
        climbMotor.set(-Constants.ClimbConstants.extendoSpeed);
        LEDs.getInstance().setCommand(LEDPreset.Solid.kBlue).schedule();
    }

    public void stopMotor() {
        // Stops motor
        climbMotor.set(0);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climb Motor Velocity", climbMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Climb Motor Voltage", climbMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Climb Motor Position", climbMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Is Climbed", isClimbed());
    }

    public boolean isClimbed() {
        return climbMotor.getPosition().getValueAsDouble() <= ClimbConstants.endPosition;
    }

    public boolean isExtendoPosition() {
        return climbMotor.getPosition().getValueAsDouble() >= ClimbConstants.extendoPosition;
    }

    // Creates new climb command
    public Command climbCommand() {
        // Only climbing uses LEDs in the command itself
        return new ClimbCommand(this);
    }

    // Creates new extending command
    public Command extendingCommand() {
        return new ClimbExtendoCommand(this);
    }

    // Creates new retracting command
    public Command retractingCommand() {
        return new ClimbRetractCommand(this);
    }
}