package frc.robot.modules.coral;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotConstants;

public class CoralModuleIOSim implements CoralModuleIO {
    private final double MOTOR_INTAKE_SPEED = 0.8;
    private final double MOTOR_EJECT_SPEED = -0.6;

    private final DCMotor motor = DCMotor.getKrakenX60Foc(1);
    private final double reduction = (18.0 / 12.0);
    private final double moi = 0.001;

    private final DCMotorSim dcMotorSim;

    private double appliedVoltage = 0.0;
    private boolean has_coral = false;

    /**
     * 
     */
    public CoralModuleIOSim() {
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(this.motor, this.moi, this.reduction),
                this.motor);
    }

    @Override
    public void updateInputs(CoralModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        this.dcMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);

        inputs.data = new CoralModuleIOData(
                true,
                this.dcMotorSim.getAngularPositionRad(),
                this.dcMotorSim.getAngularVelocityRadPerSec(),
                this.appliedVoltage,
                0.0,
                this.dcMotorSim.getCurrentDrawAmps(),
                0.0);
    }

    @Override
    public void eject() {
        this.appliedVoltage = MathUtil.clamp(MOTOR_EJECT_SPEED * 12, -12.0, 12.0);
        this.dcMotorSim.setInputVoltage(this.appliedVoltage);
        this.has_coral = false;
    }

    @Override
    public boolean hasCoral() {
        return this.has_coral;
    }

    @Override
    public void intake() {
        this.appliedVoltage = MathUtil.clamp(MOTOR_INTAKE_SPEED * 12, -12.0, 12.0);
        this.dcMotorSim.setInputVoltage(this.appliedVoltage);
        this.has_coral = true;
    }

    @Override
    public void setHasCoral(boolean has_coral) {
        this.has_coral = has_coral;
    }

    @Override
    public void stop() {
        this.appliedVoltage = 0.0;
        this.dcMotorSim.setInputVoltage(this.appliedVoltage);
    }
}