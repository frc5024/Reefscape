package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
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
    int coralChannel = Constants.coralConstants.coralChannel;
    int linebreakChannel = Constants.coralConstants.linebreakChannel;
    double intakeSpeed = Constants.coralConstants.intakeSpeed;
    double outtakeSpeed = Constants.outtakeConstants.outtakeSpeed;

    //constructor for coralMotor
    public Coral() {
        coralMotor = new SparkFlex(coralChannel, SparkLowLevel.MotorType.kBrushless);
        
    }
    // method for intaking coral, takes in a boolean to determine if the coral should intake
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
            coralMotor.set(outtakeSpeed);
        }
        else {
            coralMotor.set(0);
        }
    }
    
}
