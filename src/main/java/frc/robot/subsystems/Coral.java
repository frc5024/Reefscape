package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Coral extends SubsystemBase{

    public coralState state;

    public enum coralState {
        IDLE,
        HOLDING,
        DROPPING,
    }
      
    // motor controller for coral
    private SparkFlex coralMotor;

    //all constants for coral
    int coralMotorChannel = Constants.coralConstants.coralMotorChannel;
    int linebreakTopChannel = Constants.coralConstants.linebreakTopChannel;
    int linebreakBottomChannel = Constants.coralConstants.linebreakBottomChannel;
    double intakeSpeed = Constants.coralConstants.intakeSpeed;
    double outtakeSpeed = Constants.coralConstants.outtakeSpeed;

    ShuffleboardTab tab = Shuffleboard.getTab("CoralMotors");
    GenericEntry intakeMotorSpeedEntry = tab.add("SET intake speed", intakeSpeed).getEntry();
    GenericEntry outtakeMotorSpeedEntry = tab.add("SET outtake speed", outtakeSpeed).getEntry();

    //constructor for coralMotor
    public Coral() {
        coralMotor = new SparkFlex(coralMotorChannel, SparkFlex.MotorType.kBrushless);

        tab.addDouble("intake motor speed", () -> coralMotor.get());
        
    }
    // method for intaking coral, takes in a boolean to determine if the coral should intake

    @Override
    public void periodic() {
        intakeSpeed = intakeMotorSpeedEntry.getDouble(intakeSpeed);
        outtakeSpeed = outtakeMotorSpeedEntry.getDouble(outtakeSpeed);
    }


    public void startIntake(boolean intaking) {
        if(intaking) {
            coralMotor.set(intakeSpeed);
        }
        else {
            coralMotor.set(0);
        }
    }
    //idle state, set motor to 0
    public void setIdle() {
        coralMotor.set(0);
    }

    // method for outtaking coral, takes in a boolean to determine if the coral should outtake
    public void startOuttake(boolean outtaking) {
        //if outtaking true, set outtakeSpeed, if not, set motor to 0
        if(outtaking) {
            coralMotor.set(0.6);
        }
        else {
            coralMotor.set(0);
        }
    }
    
}
