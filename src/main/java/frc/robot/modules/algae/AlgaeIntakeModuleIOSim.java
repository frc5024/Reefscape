package frc.robot.modules.algae;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotConstants;

/**
 * 
 */
public class AlgaeIntakeModuleIOSim implements AlgaeIntakeModuleIO {
    private final double MOTOR_INTAKE_SPEED = 0.8;
    private final double MOTOR_EJECT_SPEED = -0.6;

    private final DCMotor motor = DCMotor.getKrakenX60Foc(1);
    private final double reduction = (18.0 / 12.0);
    private final double moi = 0.001;

    private final DCMotorSim dcMotorSim;

    private double appliedVoltage = 0.0;

    /**
     * 
     */
    public AlgaeIntakeModuleIOSim() {
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(this.motor, this.moi, this.reduction),
                this.motor);
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.connected = true;
        this.dcMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);
        inputs.appliedVoltage = this.appliedVoltage;
        inputs.supplyCurrentAmps = this.dcMotorSim.getCurrentDrawAmps();
    }

    @Override
    public void eject() {
        this.appliedVoltage = MathUtil.clamp(MOTOR_EJECT_SPEED * 12, -12.0, 12.0);
        this.dcMotorSim.setInputVoltage(this.appliedVoltage);
    }

    @Override
    public void intake() {
        this.appliedVoltage = MathUtil.clamp(MOTOR_INTAKE_SPEED * 12, -12.0, 12.0);
        this.dcMotorSim.setInputVoltage(this.appliedVoltage);
    }

    @Override
    public void stop() {
        this.appliedVoltage = 0.0;
        this.dcMotorSim.setInputVoltage(this.appliedVoltage);
    }
}