// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private DigitalInput linebreak;
    private DigitalInput limSwInput;
    private static Shooter mInstance = null;
    public boolean sensorOutput;
    private LEDs LEDS = LEDs.getInstance();
    private boolean isRunning;

    public static final Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }

        return mInstance;
    }

    private Shooter() {
        linebreak = new DigitalInput(6);
        limSwInput = new DigitalInput(5);
    }

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

        if (linebreak.get()) {
            isRunning = false;
            sensorOutput = false;
            System.out.println("Linebreak: " + sensorOutput);
            LEDS.setLEDS(LEDPreset.Solid.kRed);

        } else {
            if (!isRunning) {
                sensorOutput = true;
                System.out.println("Linebreak: " + sensorOutput);
                LEDS.E5024LEDS(50).schedule();
                isRunning = true;
            }
        }

        if (limSwInput.get()) {
            // maybe don't invert this. will have to test with an actual sensor.
            isRunning = false;
            sensorOutput = false;
            System.out.println("Limit Switch: " + sensorOutput);
            LEDS.setLEDS(LEDPreset.Solid.kRed);
        } else {
            if (!isRunning) {
                sensorOutput = true;
                System.out.println("Limit Switch: " + sensorOutput);
                System.out.println("Limit Broken");
                LEDS.flashLEDS(LEDPreset.Solid.kGreen, 50).schedule();
                isRunning = true;
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
