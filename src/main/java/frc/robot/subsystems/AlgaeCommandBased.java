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
    // Singleton Instance
    private static AlgaeCommandBased mInstance = null;

    // Intake Variables
    private SparkMax motor1;
    private SparkMax motor2;
    private DigitalInput linebreak;

    // Pin Variables
    private Servo pinMotor;

    // Singleton Pattern
    public static final AlgaeCommandBased getInstance() {
        if (mInstance == null) {
            mInstance = new AlgaeCommandBased();
        }
        return mInstance;
    }

    private AlgaeCommandBased() {
        // Motors used to control the Algae intake mechanism
        motor1 = new SparkMax(Constants.Algaes.motorID1, MotorType.kBrushless);
        motor2 = new SparkMax(Constants.Algaes.motorID2, MotorType.kBrushless);

        // Sensor used to detect if an Algae is in our intake mechanism
        linebreak = new DigitalInput(Constants.Algaes.linebreakPort);

        // pinMotor servo motor used to lock and extend intake mechanism
        pinMotor = new Servo(Constants.Algaes.pinMotorPort);
    }

    // Set both sparkmax motors to one of the motor speed constants
    // (ex. idle, launch, drop, intake)
    public void setSpeed(Double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }

    // Gets the current value of the linebreak sensor and returns to systems outside
    // of this one
    public boolean getLinebreak() {
        return (linebreak.get());
    }

    // Sets the speed of the servo controlling the pin
    public void setPin(int pinDirection) {
        pinMotor.set(pinDirection);
    }

    // Creating new instance of AlgaeIntakeCommand class
    public Command intake() {
        return new AlgaeIntakeCommand(mInstance);
    }

    // Creating new instance of AlgaeLaunchCommand class
    public Command launch() {
        return new AlgaeLaunchCommand(mInstance);
    }

    // Creating new instance of AlgaeDropCommand class
    public Command drop() {
        return new AlgaeDropCommand(mInstance);
    }

    // Creating new instance of AlgaeCancelCommand class
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
