package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    public static Climb mInstance = null;

    private TalonFX climbMotor;

    ShuffleboardTab tab = Shuffleboard.getTab("Climb");
    GenericEntry encoder = tab.add("climbSpeed", .35).getEntry();

    // Ultrasonic
    private final Ultrasonic m_ultrasonic = new Ultrasonic(Constants.ClimbConstants.pingID,
            Constants.ClimbConstants.echoID);
    double distanceMillimetres;
    double measurement;
    MedianFilter filter = new MedianFilter(Constants.ClimbConstants.filterValue);

    public static Climb getInstance() {
        if (mInstance == null) {
            mInstance = new Climb();
        }
        return mInstance;
    }

    public Climb() {
        climbMotor = new TalonFX(Constants.ClimbConstants.climbMotorID);
        tab.addDouble("encoder value", () -> climbMotor.getPosition().getValueAsDouble());

    }

    public void climbing() {
        // speed = encoder.getDouble(0);
        if (climbMotor.getPosition().getValueAsDouble() >= Constants.ClimbConstants.liftoffPos && !overThreshold()) {
            System.out.println("CLIMB FAILED");
            climbMotor.set(0);
            LEDs.getInstance().setCommand(LEDPreset.Strobe.kRed).schedule();

        } else {
            climbMotor.set(Constants.ClimbConstants.climbSpeed);
            LEDs.getInstance().setCommand(LEDPreset.Solid.kGreen).schedule();
        }
    }

    public void extending() {
        // speed = encoder.getDouble(0);
        climbMotor.set(Constants.ClimbConstants.extendoSpeed);
        LEDs.getInstance().setCommand(LEDPreset.Solid.kOrange).schedule();
    }

    public void retracting() {
        // speed = encoder.getDouble(0);
        climbMotor.set(-Constants.ClimbConstants.extendoSpeed);
        LEDs.getInstance().setCommand(LEDPreset.Solid.kBlue).schedule();
    }

    public void cancel() {
        climbMotor.set(Constants.ClimbConstants.cancelSpeed);
        LEDs.getInstance().setCommand(LEDPreset.Solid.kDarkRed).schedule();

    }

    public void stopMotor() {
        climbMotor.set(0);
    }

    @Override
    public void periodic() {
        Ultrasonic.setAutomaticMode(true);
        m_ultrasonic.setEnabled(true);

        distanceMillimetres = m_ultrasonic.getRangeMM();
        // Calculates the average of the given values from the Ultrasonic sensor
        measurement = filter.calculate(distanceMillimetres);

        SmartDashboard.putBoolean("Over Threshold", overThreshold());
        SmartDashboard.putNumber("Ultrasonic", measurement);
    }

    public boolean overThreshold() {
        // Returns true if the Ultrasonic sensor detects that it is a certain distance
        // above the ground
        if (measurement >= Constants.ClimbConstants.ultrasonicThreshold) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isClimbPosition() {
        // Returns true if the Encoder detects the motor is at end position
        if (climbMotor.getPosition().getValueAsDouble() >= Constants.ClimbConstants.endPosition) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isExtendoPosition() {
        // Returns true if the Encoder detects the motor is at end position
        if (climbMotor.getPosition().getValueAsDouble() <= Constants.ClimbConstants.extendoPosition) {
            return true;
        } else {
            return false;
        }
    }
}