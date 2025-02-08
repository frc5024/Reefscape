package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.coralConstants;
import frc.robot.commands.Coral.CancelIntakeCommand;
import frc.robot.commands.Coral.IntakeCommand;
import frc.robot.commands.Coral.OuttakeCommand;
import frc.robot.commands.Coral.PlopCommand;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Coral extends SubsystemBase{     
    // motor controller for coral
    private SparkFlex coralMotor;
    private SparkFlex coralMotorReversed;
    
    private final SparkBaseConfig coralMotorReversedConfig = new SparkFlexConfig()
        .inverted(true)
        .follow(coralConstants.coralMotorChannel);

    private static DigitalInput linebreak;

    //all constants for coral
    int coralMotorChannel = Constants.coralConstants.coralMotorChannel;
    int coralMotorReversedChannel = Constants.coralConstants.coralMotorReversedChannel;
    int linebreakChannel = Constants.coralConstants.linebreakChannel;
    
    double intakeSpeed = Constants.coralConstants.intakeSpeed;
    double outtakeSpeed = Constants.coralConstants.outtakeSpeed;
    double plopSpeed = Constants.coralConstants.plopSpeed;

    // double servoRotate = Constants.coralConstants.servoRotate;
    // double servoReset = Constants.coralConstants.servoReset;

    //shuffleboard tab for coral
    ShuffleboardTab tab = Shuffleboard.getTab("CoralMotors");
    GenericEntry intakeMotorSpeedEntry = tab.add("SET intake speed", intakeSpeed).getEntry();
    GenericEntry outtakeMotorSpeedEntry = tab.add("SET outtake speed", outtakeSpeed).getEntry();
    GenericEntry plopSpeedEntry = tab.add("SET plop speed", plopSpeed).getEntry();

    //constructor for coralMotor
    public Coral() {
        linebreak = new DigitalInput(Constants.coralConstants.linebreakChannel);
        
        coralMotor = new SparkFlex(coralMotorChannel, SparkFlex.MotorType.kBrushless);
        tab.addDouble("motor speed", () -> coralMotor.get());
        
        coralMotorReversed = new SparkFlex(coralMotorReversedChannel, SparkFlex.MotorType.kBrushless);
        
        tab.addDouble("reversed motor speed", () -> coralMotorReversed.get());

        this.coralMotorReversed.configure(coralMotorReversedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    

    // method for intaking coral, takes in a boolean to determine if the coral should intake

    @Override
    public void periodic() {
        intakeSpeed = intakeMotorSpeedEntry.getDouble(intakeSpeed);
        outtakeSpeed = outtakeMotorSpeedEntry.getDouble(outtakeSpeed);
        plopSpeed = plopSpeedEntry.getDouble(plopSpeed);
    }

    //idle state, set motor to 0
    public void setIdle() {
        coralMotor.set(0);
    }

    public boolean isLineBroken() {
        return linebreak.get();
    }

    public double getEncoder() {
        return coralMotor.getAbsoluteEncoder().getPosition();
        //return coralMotorReversed.getAbsoluteEncoder().getPosition();
    }

    public void set(double speed) {
        coralMotor.set(speed);
    }

    public Command plopCommand(){
        return new PlopCommand(this);
    }

    public Command intakeCommand(){
        return new IntakeCommand(this);
    }

    public Command outtakeCommand(){
        return new OuttakeCommand(this);
    }

    public Command cancelIntakeCommand(){
        return new CancelIntakeCommand(this);
    }

    public Command lowerRampCommand(){
        return new PlopCommand(this);
    }

    //---------------SERVO STUFF --------------
    //rotate servo 90 degrees (lowers the ramp)
    // public void lowerRamp() {
    //     rampServo.set(servoRotate);
    // }
    //rotates servo backwards 90 degrees (raises the ramp back to original position)
    // public void resetRamp() {
    //     rampServo.set(servoReset);
    // }
    // //sets servo to 0 (for initialization)
    // public void setRamp() {
    //     rampServo.set(0);
    // }   
}
