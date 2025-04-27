package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.kauailabs.navx.frc.AHRS;
//import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.Turret.StickRotationCommand;

public class Turret extends SubsystemBase {

    private final SparkMax turretMotor;
    private final PIDController pidController;
    private final AHRS gyro;

    private double outputSpeed;
    private double joystickSpeed;
    private double currentAngle;
    private double targetAngle;

    int turretMotorChannel = Constants.turretConstants.turretMotorChannel;

    public Turret() {

        turretMotor = new SparkMax(turretMotorChannel, SparkLowLevel.MotorType.kBrushless);
        setDefaultCommand(new StickRotationCommand(this));

        pidController = new PIDController(Constants.turretConstants.kP, Constants.turretConstants.kI, Constants.turretConstants.kD);
        pidController.setTolerance(Constants.turretConstants.turretTolerance);
        gyro = new AHRS(SPI.Port.kMXP);
    }

    public void updateJoystick(double joystickSpeed) {
        this.joystickSpeed = joystickSpeed;
    }

    public void turretMath(double joystickSpeed) {
        if(Math.abs(joystickSpeed) > 0.01) {
            outputSpeed = joystickSpeed;
        } else {
            outputSpeed = 0;
        }
        turretMotor.set(outputSpeed);
    }

    public double getTurretAngle(){
        return turretMotor.getEncoder().getPosition();
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle; // Update the desired turret angle
    }

    public boolean isAtTargetAngle() {
        return pidController.atSetpoint();
    }

    public void updateTurretAngle() {
        currentAngle = getTurretAngle();

        double output = pidController.calculate(currentAngle, targetAngle);

        turretMotor.set(output);
    }


    public double getRobotHeading() {
        return gyro.getAngle(); 
    }

    public void setIdle() {
        turretMotor.set(0);
        joystickSpeed = 0;
    }

    @Override
    public void periodic() {
        turretMath(joystickSpeed); 
    }

    public Command stickRotation(){
        return new StickRotationCommand(this);
    }

}
