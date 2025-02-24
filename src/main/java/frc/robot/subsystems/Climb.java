package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;
import frc.robot.commands.Climb.ClimbCancelCommand;
import frc.robot.commands.Climb.ClimbCommand;
import frc.robot.commands.Climb.ClimbExtendoCommand;
import frc.robot.commands.Climb.ClimbRetractCommand;

public class Climb extends SubsystemBase {
    public static Climb mInstance = null;

    private TalonFX climbMotor;

    // Shuffleboard
    ShuffleboardTab tab = Shuffleboard.getTab("Climb");
    // Not currently being used
    GenericEntry encoder = tab.add("climbSpeed", .35).getEntry();

    // ULTRASONIC NOT CURRENTLY BEING USED

    // Ultrasonic
    // private final Ultrasonic m_ultrasonic = new
    // Ultrasonic(Constants.ClimbConstants.pingID,
    // Constants.ClimbConstants.echoID);
    // double distanceMillimetres;
    // double measurement;
    // MedianFilter filter = new MedianFilter(Constants.ClimbConstants.filterValue);

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

    }

    public void climbing() {
        // Sets motor to climbing speed
        climbMotor.set(Constants.ClimbConstants.climbSpeed);
        // Schedules the LED command
        LEDs.getInstance().setCommand(LEDPreset.LightChase.kBlue).schedule();

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
        LEDs.getInstance().setCommand(LEDPreset.Solid.kDarkBlue).schedule();
    }

    public void cancel() {
        // Sets motor to cancelling speed
        climbMotor.set(Constants.ClimbConstants.cancelSpeed);
        LEDs.getInstance().setCommand(LEDPreset.Strobe.kGold).schedule();

    }

    public void stopMotor() {
        // Stops motor
        climbMotor.set(0);
    }

    @Override
    public void periodic() {
        // Ultrasonic.setAutomaticMode(true);
        // m_ultrasonic.setEnabled(true);

        // distanceMillimetres = m_ultrasonic.getRangeMM();
        // // Calculates the average of the given values from the Ultrasonic sensor
        // measurement = filter.calculate(distanceMillimetres);

        // SmartDashboard.putBoolean("Over Threshold", overThreshold());
        // SmartDashboard.putNumber("Ultrasonic", measurement);
    }

    // public boolean overThreshold() {
    // // Returns true if the Ultrasonic sensor detects that it is a certain
    // distance
    // // above the ground
    // if (measurement >= Constants.ClimbConstants.ultrasonicThreshold) {
    // return true;
    // } else {
    // return false;
    // }
    // }

    // Booleans

    public boolean isClimbed() {
        // Returns true if the Encoder detects the motor is at climbed position
        if (climbMotor.getPosition().getValueAsDouble() <= Constants.ClimbConstants.endPosition) {
            // if (climbMotor.getPosition().getValueAsDouble() >=
            // Constants.ClimbConstants.liftoffPos
            // && !overThreshold()) {
            // System.out.println("CLIMB FAILED");
            // LEDs.getInstance().setCommand(LEDPreset.Strobe.kRed).schedule();
            // } else {
            // }
            return true;
        } else {
            return false;
        }
    }

    public boolean isExtendoPosition() {
        // Returns true if the Encoder detects the motor is at extended position
        if (climbMotor.getPosition().getValueAsDouble() >= Constants.ClimbConstants.extendoPosition) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isRetractPosition() {
        // Returns true if the Encoder detects the motor is at extended position
        if (climbMotor.getPosition().getValueAsDouble() <= Constants.ClimbConstants.endPosition) {
            return true;
        } else {
            return false;
        }
    }

    // Commands

    // Creates new climb command
    public Command climbCommand(LEDs LEDs) {
        // Only climbing uses LEDs in the command itself
        return new ClimbCommand(this, LEDs);
    }

    // Creates new extending command
    public Command extendingCommand() {
        return new ClimbExtendoCommand(this);
    }

    // Creates new retracting command
    public Command retractingCommand() {
        return new ClimbRetractCommand(this);
    }

    // Creates new cancel command
    public Command cancelCommand() {
        return new ClimbCancelCommand(this);
    }
}