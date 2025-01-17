package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
//import com.revrobotics.spark.config.SparkFlexConfig;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Coral extends SubsystemBase{

    public coralState state;

    public enum coralState {
        IDLE,
        HOLDING,
        DROPPING,
    }

    private SparkFlex coralMotor;
    //private DigitalInput linebreak;
    
    int coralChannel = Constants.coralConstants.coralChannel;
    int linebreakChannel = Constants.coralConstants.linebreakChannel;
    double intakeSpeed = Constants.coralConstants.intakeSpeed;
    double outtakeSpeed = Constants.coralConstants.outtakeSpeed;

    public Coral() {
        coralMotor = new SparkFlex(coralChannel, SparkLowLevel.MotorType.kBrushless);
        //linebreak = new DigitalInput(linebreakChannel);
    }

    public void startIntake(boolean intaking) {
        if(intaking) {
            coralMotor.set(intakeSpeed);
        }
        else {
            coralMotor.set(0);
        }
    }

    public void setIdle() {
        coralMotor.set(0);
    }


    public void startOuttake(boolean outtaking) {
        if(outtaking) {
            coralMotor.set(outtakeSpeed);
        }
        else {
            coralMotor.set(0);
        }
    }
    
}
