package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AlgaeCmd;

public class Algae extends SubsystemBase {

    public SparkMax algaeMotor;

    public static Algae mInstance = null;
    public double speed;
    public boolean direction;

    public static Algae getInstance() {
        if (mInstance == null) {
            mInstance = new Algae();
        }
        return mInstance;
    }

    public Algae() {
        algaeMotor = new SparkMax(62, SparkLowLevel.MotorType.kBrushless);
    }

    public void start(double speed) {
        algaeMotor.set(speed);
    }

    public void stop() {
        algaeMotor.set(0);
    }

    // public boolean isExtended() {
    // if (algaeMotor.getEncoder().getPosition() >
    // Constants.AlgaeConstant.algaeOutPos) {
    // return true;
    // } else {
    // return false;
    // }
    // }

    // public boolean isRetracted() {
    // if (algaeMotor.getEncoder().getPosition() <
    // Constants.AlgaeConstant.algaeInPos) {
    // return true;
    // } else {
    // return false;
    // }
    // }

    public Command algaeCommand(boolean toggleExtending) {
        return new AlgaeCmd(this, toggleExtending);
    }
}