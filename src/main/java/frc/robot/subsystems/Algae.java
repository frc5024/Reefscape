// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Algae.AlgaeCancelCommand;
import frc.robot.commands.Algae.AlgaeDropCommand;
import frc.robot.commands.Algae.AlgaeIntakeCommand;
import frc.robot.commands.Algae.AlgaeLaunchCommand;

public class Algae extends SubsystemBase {
    // Singleton Instance
    private static Algae mInstance;

    // Motors and linebreak sensor
    private SparkMax algaeMotor1;
    private SparkMax algaeMotor2;
    private DigitalInput algaeLinebreak;

    // Algae constants from Constants
    final int algaeMotor1_ID = Constants.Algaes.algaeMotor1_ID;
    final int algaeMotor2_ID = Constants.Algaes.algaeMotor2_ID;
    final int linebreakChannel = Constants.Algaes.linebreakChannel;

    // Singleton Pattern
    public static final Algae getInstance() {
        if (mInstance == null) {
            mInstance = new Algae();
        }
        return mInstance;
    }

    private Algae() {

        // Motors used to control the Algae intake mechanism
        algaeMotor1 = new SparkMax(algaeMotor1_ID, MotorType.kBrushless);
        algaeMotor2 = new SparkMax(algaeMotor2_ID, MotorType.kBrushless);

        // Sensor used to detect if an Algae is in our intake mechanism
        algaeLinebreak = new DigitalInput(linebreakChannel);

    }

    // Set both sparkmax motors to one of the motor speed constants
    // (ex. idle, launch, drop, intake)
    public void setSpeed(Double speed) {
        algaeMotor1.set(speed);
        algaeMotor2.set(speed);
    }

    // Gets the current value of the linebreak sensor and returns to systems outside
    // of this one
    public boolean getAlgaeLinebreak() {
        return (algaeLinebreak.get());
    }

    // Creating new instance of AlgaeIntakeCommand class
    public Command intake() {
        return new AlgaeIntakeCommand(this);
    }

    // Creating new instance of AlgaeLaunchCommand class
    public Command launch() {
        return new AlgaeLaunchCommand(this);
    }

    // Creating new instance of AlgaeDropCommand class
    public Command drop() {
        return new AlgaeDropCommand(this);
    }

    // Creating new instance of AlgaeCancelCommand class
    public Command cancel() {
        return new AlgaeCancelCommand(this);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
