// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Algae extends SubsystemBase {

    final double motorspeedintake = -0.5;
    final double motorspeedouttake = 0.1;
    final double motorspeedidle = 0;
    private DigitalInput linebreak;
    private static Algae mInstance = null;
    public boolean sensorOutput;
    private LEDs LEDS = LEDs.getInstance();
    private SparkMax motor1;
    private SparkMax motor2;
    private Servo pinMotor;
    private Timer algaeOuttakeTimer;
    // private Servo motorarm;

    private enum states {
        idle,
        intaking,
        holding,
        outaking
    }

    private states currentstate = states.idle;

    public static final Algae getInstance() {
        if (mInstance == null) {
            mInstance = new Algae();
        }

        return mInstance;
    }

    private Algae() {
        pinMotor = new Servo(Constants.Algaes.pinMotorPort);
        linebreak = new DigitalInput(Constants.Algaes.linebreakPort);
        // limSwInput = new DigitalInput(5);
        motor1 = new SparkMax(Constants.Algaes.motorID1, MotorType.kBrushless);
        motor2 = new SparkMax(Constants.Algaes.motorID2, MotorType.kBrushless);
        // motorarm = new Servo(Constants.Algae.AlgaeArmPort);
        algaeOuttakeTimer = new Timer();

    }

    public int checkPressTime(int presstime, double buttonaxis) {

        if ((buttonaxis) > 0.1) {
            presstime += 1;
        } else {
            presstime = 0;
        }

        return presstime;
    }

    // Assigns sparkmax motors to variables
    public void setSpeed(Double speed) {
        motor1.set(speed);
        motor2.set(speed);

    }

    // Sets the speed of the servo controlling the pin

    // Sets the speed of the servo controlling the pin
    public void setPin(int pinDirection) {

        pinMotor.set(pinDirection);

    }

    public void setLEDColor(states state) {

        if (state == states.idle) {
            LEDS.setLEDS(LEDPreset.Solid.kWhite);
        }
        if (state == states.intaking) {
            LEDS.setLEDS(LEDPreset.Solid.kGreen);
        }
        if (state == states.outaking) {
            LEDS.setLEDS(LEDPreset.Solid.kRed);
        }
        if (state == states.holding) {
            LEDS.setLEDS(LEDPreset.Solid.kBlue);
        }
    }
    // public void setArmMotorAngle(Double angle){
    // motorarm.setangle(angle);
    // }
    // Sets the motors speed to the setSpeed value

    /**
     * Example command factory method.
     *
     * @return a command
     */

    public void motorSpeedStateMachine(int presstime) {

        // When algae is detected inside robot and robot is not outaking, set state to
        // holding
        if ((linebreak.get() == true) && (currentstate != states.outaking)) {
            setSpeed(0.0);
            currentstate = states.holding;
        }

        // When algae is not detected inside robot and robot is not intaking, set state
        // to idle
        if ((linebreak.get() == false)
                && (currentstate != states.intaking) && (algaeOuttakeTimer.hasElapsed(1.5))) {
            setSpeed(0.0);
            currentstate = states.idle;
            algaeOuttakeTimer.stop();
        }
        // When button is pressed
        if ((presstime == 1)) {

            // When idling, sets state to intaking and enables motors
            if (currentstate == states.idle) {
                setSpeed(Constants.Algaes.intakeSpeed);
                currentstate = states.intaking;
            }

            // When intaking, sets state to idle and disables motors
            else if (currentstate == states.intaking) {
                setSpeed(0.0);
                currentstate = states.idle;
            }

            // When holding an algae, sets state to outtaking and enables motors in reverse
            else if (currentstate == states.holding) {
                setSpeed(Constants.Algaes.launchSpeed);
                currentstate = states.outaking;
                algaeOuttakeTimer.restart();
            }

        }

        // White is idle
        // Red is holding
        // Green is intaking
        // Blue is outtaking
        setLEDColor(currentstate);

    }

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
