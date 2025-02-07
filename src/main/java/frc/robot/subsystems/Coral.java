package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private SparkFlex coralMotorReversed;
    private static DigitalInput linebreak;

    //all constants for coral
    int coralMotorChannel = Constants.coralConstants.coralMotorChannel;
    int coralMotorReversedChannel = Constants.coralConstants.coralMotorReversedChannel;
    int linebreakChannel = Constants.coralConstants.linebreakChannel;
    //int linebreakBottomChannel = Constants.coralConstants.linebreakBottomChannel;
    int servoChannel = Constants.coralConstants.servoChannel;
    double intakeSpeed = Constants.coralConstants.intakeSpeed;
    double outtakeSpeed = Constants.coralConstants.outtakeSpeed;
    double outtakeTime = Constants.coralConstants.outtakeTime;

    ShuffleboardTab tab = Shuffleboard.getTab("CoralMotors");
    GenericEntry intakeMotorSpeedEntry = tab.add("SET intake speed", intakeSpeed).getEntry();
    GenericEntry outtakeMotorSpeedEntry = tab.add("SET outtake speed", outtakeSpeed).getEntry();
    GenericEntry outtakeTimeEntry = tab.add("SET outtake time", outtakeTime).getEntry();

    //constructor for coralMotor
    public Coral() {
        linebreak = new DigitalInput(Constants.coralConstants.linebreakChannel);
        
        coralMotor = new SparkFlex(coralMotorChannel, SparkFlex.MotorType.kBrushless);
        tab.addDouble("motor speed", () -> coralMotor.get());

        coralMotorReversed = new SparkFlex(coralMotorReversedChannel, SparkFlex.MotorType.kBrushless);
        tab.addDouble("reversed motor speed", () -> coralMotorReversed.get());
    }

    // method for intaking coral, takes in a boolean to determine if the coral should intake

    @Override
    public void periodic() {
        intakeSpeed = intakeMotorSpeedEntry.getDouble(intakeSpeed);
        outtakeSpeed = outtakeMotorSpeedEntry.getDouble(outtakeSpeed);
        outtakeTime = outtakeTimeEntry.getDouble(outtakeTime);
    }


    
    //idle state, set motor to 0
    public void setIdle() {
        coralMotor.set(0);
        coralMotorReversed.set(0);
    }

    public boolean isLineBroken() {
        return linebreak.get();
    }

    public void startIntake() {
        if(!isLineBroken()){
            coralMotor.set(intakeSpeed);
            coralMotorReversed.set(-intakeSpeed);
        }
    }

    // method for outtaking coral, takes in a boolean to determine if the coral should outtake
    public void startOuttake() {
        if(isLineBroken()){
            //if outtaking true, set outtakeSpeed, if not, set motor to 0
            coralMotor.set(-outtakeSpeed);
            coralMotorReversed.set(outtakeSpeed);
        }
    }
}
