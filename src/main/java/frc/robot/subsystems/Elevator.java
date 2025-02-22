package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees; // TODO: Remove all unused imports
import static edu.wpi.first.units.Units.Inch;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.elevatorConstants;

public class Elevator extends SubsystemBase{
     //created and named the motor controller
    public SparkMax elevatorMotor; //LEAD R  CHANGE THIS
    public SparkMax elevatorMotor2; //FOLLOWER L
    private final SparkBaseConfig elevatorMotorConfig = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake) //sets the motors to break mode
            .inverted(true);
    private final SparkBaseConfig elevatorMotor2Config = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            //.inverted(true)
            .follow(elevatorConstants.motorID1, false);


    //created and named the PID controller
    private ProfiledPIDController PID;
    private ElevatorFeedforward feedForward;
    private TrapezoidProfile.Constraints feedForwardConstraints;

    //made it so "speed" is able to be accessed by the whole class
    private double speed;
    private double voltageValue;
    private boolean enabled;


    //created and named the limit switches
    private static DigitalInput zeroingLimitSwitch;
    private static DigitalInput stoppingLimitSwitch;

    //added shuffleboard tabs to change the different values in the shuffle board app
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    GenericEntry pEntry = tab.add("SET P", elevatorConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", elevatorConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", elevatorConstants.kI).getEntry();
    GenericEntry gEntry = tab.add("SET G", elevatorConstants.G).getEntry();
    GenericEntry vEntry = tab.add("SET V", elevatorConstants.kV).getEntry();
    GenericEntry aEntry = tab.add("SET A", elevatorConstants.kA).getEntry();
    GenericEntry maxSpeedEntry = tab.add("SET Max Speed", (elevatorConstants.elevatorMaxSpeed)).getEntry();
    GenericEntry maxAccelerationEntry = tab.add("SET Max accel", (elevatorConstants.elevatorMaxAccel)).getEntry();
    GenericEntry SETsetPoint = tab.add("SET Dest (DEG)", 0.0).getEntry();
    GenericEntry motor1ManualEntry = tab.add("SET MANUAL SPEED", 0.0).getEntry();


    //constructor
    public Elevator() {
        //assigning the ID and values
        elevatorMotor = new SparkMax(elevatorConstants.motorID1, SparkLowLevel.MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(elevatorConstants.motorID2, SparkLowLevel.MotorType.kBrushless);
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotor2.configure(elevatorMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        zeroingLimitSwitch = new DigitalInput(8);
        stoppingLimitSwitch = new DigitalInput(1);

        //assigning values to the P, I and D
        feedForwardConstraints = new TrapezoidProfile.Constraints(elevatorConstants.elevatorMaxSpeed, elevatorConstants.elevatorMaxAccel);
        PID = new ProfiledPIDController(elevatorConstants.kP, elevatorConstants.kI, elevatorConstants.kD, feedForwardConstraints);
        PID.setTolerance(0.25, 0.25); // TODO: put in constants
        feedForward = new ElevatorFeedforward(0, elevatorConstants.G, elevatorConstants.kV, elevatorConstants.kA); //ks, kg, kv, ka
        elevatorMotor.getEncoder().setPosition(elevatorConstants.zeroPosition);

        //elevatorMotor.setPosition(0);
        
     //callback loop. calls the function everytime it wants a value. Constantly checks the value.
       tab.addDouble("PID Speed Value", () -> speed);
       tab.addBoolean("bottom limitswitch", () -> isBottomLimitSwitchBroken());
       tab.addBoolean("toplimitswitch", () -> isTopLimitSwitchBroken());
       tab.addDouble("voltage", () -> voltageValue);
       tab.addDouble("Actual Velocity", () -> rotationsToInches(elevatorMotor.getEncoder().getVelocity()) / 60);
       tab.addDouble("Estimated Velocity", () -> PID.getSetpoint().velocity);
       tab.addDouble("actual Position", () -> rotationsToInches(elevatorMotor.getEncoder().getPosition()));
       tab.addDouble("estimated Position", () -> PID.getSetpoint().position);

       
       // TODO: log voltage anything else you think you need

       
    }


    @Override
    public void periodic(){

        //getting the PID values and showing them on the shuffle board ("getDouble" constantly checks the value)
        PID.setP(pEntry.getDouble(elevatorConstants.kP));
        PID.setI(iEntry.getDouble(elevatorConstants.kI));
        PID.setD(dEntry.getDouble(elevatorConstants.kD));
        feedForward.setKa(aEntry.getDouble(elevatorConstants.kA));
        feedForward.setKv(vEntry.getDouble(elevatorConstants.kV));
        feedForward.setKg(gEntry.getDouble(elevatorConstants.G));

        if (enabled) {
            feedPIDMotor();
        } else {
            elevatorMotor.set(0);
        }

        speed = elevatorMotor.getAppliedOutput();

        // //if the boolean enabled is true then run this command

        // if (speed >= maxUpSpeedEntry.getDouble(elevatorConstants.elevatorMaxUpSpeed)) {
        //     elevatorMotor.set(maxUpSpeedEntry.getDouble(elevatorConstants.elevatorMaxUpSpeed));
        // } else if (speed <= -maxDownSpeedEntry.getDouble(elevatorConstants.elevatorMaxDownSpeed)) {
        //     elevatorMotor.set(-maxDownSpeedEntry.getDouble(elevatorConstants.elevatorMaxDownSpeed));
        // } else {
        //     elevatorMotor.set(speed);
        // }

        // //safety precaution to prevent the motor from trying to go past the bottom stop
        if (speed < 0 && elevatorMotor.getEncoder().getPosition() <= elevatorConstants.minimumBottomValue) {
            elevatorMotor.set(0); // TODO: verify our minimumBottomValue tolerance is working and reasonable
        }

        checkTopLimitSwitch();
        zeroingEncoder();
    }

    // TODO: rewrite comment to be more accurate
    //creates a command for calculating speed through PID which will be used in the command instead of the periodic
    public void feedPIDMotor() {
        voltageValue = PID.calculate(rotationsToInches(elevatorMotor.getEncoder().getPosition()))
            + feedForward.calculate(PID.getSetpoint().velocity);
        elevatorMotor.setVoltage(voltageValue);
    }

    //gets the position from the SetElevatorSetpointCmd
    public void setGoal(double inches) {
        PID.setConstraints(new TrapezoidProfile.Constraints(
            maxSpeedEntry.getDouble(elevatorConstants.elevatorMaxSpeed),
            maxAccelerationEntry.getDouble(elevatorConstants.elevatorMaxAccel)
        ));

        PID.setGoal(inches);
        resetPID();

    }
    //makes it accesible to the SetElevatorSetpointCmd
    public void resetPID() {
        PID.reset(rotationsToInches(elevatorMotor.getEncoder().getPosition()));
    }

    //setting the manual target speed that can be controlled by the driver to the speed used in calculations and safety checks
    // public void controlMotor(double targetSpeed) {
    //     speed = targetSpeed;
    // }

    //creating a boolean method which returns the condition of both limit switches
    public boolean isBottomLimitSwitchBroken() {
         return !zeroingLimitSwitch.get();
    }

    public boolean isTopLimitSwitchBroken() {
        return !stoppingLimitSwitch.get();
    }


    //encoder value will reset to 0 once the bottom limit switch is triggered
    public void zeroingEncoder () {
         if (isBottomLimitSwitchBroken()) {
             elevatorMotor.getEncoder().setPosition(elevatorConstants.zeroPosition);
         }
    }

    //stop the motor if the the top limit switch is triggered
    public void checkTopLimitSwitch () {
        if (isTopLimitSwitchBroken()) {
            elevatorMotor.set(0);
            //elevatorMotor.get()
        }
    }

    //gets the encoder value for safety precautions in the periodic
    private double encoderValue() {
        return elevatorMotor.getEncoder().getPosition();
    }

    public void zeroEncoderValue() {
        elevatorMotor.getEncoder().setPosition(0.0);
    }

    @Override
    public Command getDefaultCommand() {
        return run(() -> elevatorMotor.set(0)); //***set it to g constant so that it stays put in the future
    }

    // public void motor1Manual() {
    //     elevatorMotor.set(motor1ManualEntry.getDouble(0));
    // }

    // public void motor2Manual() {
    //     elevatorMotor2.set(motor1ManualEntry.getDouble(0));
    // }

    //conversion method
    public static double inchesToRotations(double distanceValue) {
        return distanceValue * (2.007);
    }

    public static double rotationsToInches(double angleValue) {
        return angleValue/(2.007); // TODO: verify this conversion is accurate (does the carriage actually move X inches?)
    }

    public void togglePID(boolean enabled){
        this.enabled = enabled;
    }
}   



