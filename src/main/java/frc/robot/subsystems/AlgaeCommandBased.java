// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AlgaeCancelCommand;
import frc.robot.commands.AlgaeDropCommand;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeLaunchCommand;

public class AlgaeCommandBased extends SubsystemBase {

    final double motorspeedintake = -0.5;
    final double motorspeedouttake = 0.1;
    final double motorspeedidle = 0;
    private DigitalInput linebreak;
    private static AlgaeCommandBased mInstance = null;
    public boolean sensorOutput;
    private LEDs LEDS = LEDs.getInstance();
    private SparkMax motor1;
    private SparkMax motor2;
    private Servo pinMotor;

    // private Servo motorarm;

    private enum states {
        idle,
        intaking,
        holding,
        outaking
    }

    public static final AlgaeCommandBased getInstance() {
        if (mInstance == null) {
            mInstance = new AlgaeCommandBased();
        }

        return mInstance;
    }

    private AlgaeCommandBased() {
        pinMotor = new Servo(Constants.Algaes.pinMotorPort);
        linebreak = new DigitalInput(Constants.Algaes.linebreakPort);
        // limSwInput = new DigitalInput(5);
        motor1 = new SparkMax(Constants.Algaes.motor1Port, MotorType.kBrushless);
        motor2 = new SparkMax(Constants.Algaes.motor2Port, MotorType.kBrushless);
        // motorarm = new Servo(Constants.Algae.AlgaeArmPort);

    }

    // Assigns sparkmax motors to variables
    public void setSpeed(Double speed) {
        motor1.set(speed);
        motor2.set(speed);

    }

    public void dropSpeed() {
        motor1.set(Constants.Algaes.dropSpeed);
        motor2.set(Constants.Algaes.dropSpeed);
    }

    public void intakeSpeed() {
        motor1.set(Constants.Algaes.intakeSpeed);
        motor2.set(Constants.Algaes.intakeSpeed);
    }

    public void launchSpeed() {
        motor1.set(Constants.Algaes.launchSpeed);
        motor2.set(Constants.Algaes.launchSpeed);
    }

    public void idleSpeed() {
        motor1.set(Constants.Algaes.idleSpeed);
        motor2.set(Constants.Algaes.idleSpeed);
    }

    public boolean getLinebreak() {
        return (linebreak.get());
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

    public Command intake() {
        return new AlgaeIntakeCommand(mInstance);
    }

    public Command launch() {
        return new AlgaeLaunchCommand(mInstance);
    }

    public Command drop() {
        return new AlgaeDropCommand(mInstance);
    }

    public Command cancel() {
        return new AlgaeCancelCommand(mInstance);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
