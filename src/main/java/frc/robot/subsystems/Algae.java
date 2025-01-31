// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
    private DigitalInput linebreak;
    private DigitalInput limSwInput;
    private static Algae mInstance = null;
    public boolean sensorOutput;
    private LEDs LEDS = LEDs.getInstance();
    private SparkMax motor1;
    private SparkMax motor2;

    public static final Algae getInstance() {
        if (mInstance == null) {
            mInstance = new Algae();
        }

        return mInstance;
    }

    private Algae() {
        // linebreak = new DigitalInput(6);
        // limSwInput = new DigitalInput(5);
        motor1 = new SparkMax(3, MotorType.kBrushless);
        motor2 = new SparkMax(62, MotorType.kBrushless);

    }
    // Assigns sparkmax motors to variables

    public void setSpeed(Double speed) {
        motor1.set(speed);
        motor2.set(speed);

    }
    // Sets the motors speed to the setSpeed value

    /**
     * Example command factory method.
     *
     * @return a command
     */

    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> {
            /* one-time action goes here */
        });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // if (linebreak.get() == true) {
        // sensorOutput = true;
        // System.out.println("Linebreak: " + sensorOutput);

        // } else {
        // sensorOutput = false;
        // System.out.println("Linebreak: " + sensorOutput);
        // }

        // if (limSwInput.get() == true) {
        // // maybe don't invert this. will have to test with an actual sensor.
        // sensorOutput = false;
        // System.out.println("Limit Switch: " + sensorOutput);
        // LEDS.setLEDS(LEDPreset.Solid.kRed);
        // } else {
        // sensorOutput = true;
        // System.out.println("Limit Switch: " + sensorOutput);
        // LEDS.setLEDS(LEDPreset.Solid.kGreen);

        // }

        // if (motor1.get() < 0.01) {
        // LEDS.setLEDS(LEDPreset.Solid.kBlue);
        // }

        // if (motor1.get() > 0.01) {
        // LEDS.setLEDS(LEDPreset.Solid.kViolet);
        // }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
