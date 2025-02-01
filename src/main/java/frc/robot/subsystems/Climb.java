
package frc.robot.subsystems;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemBase {
    private TalonFX climbMotor;
    ShuffleboardTab tab = Shuffleboard.getTab("Climb");
    GenericEntry climbSpeed = tab.add("climbSpeed", .35).getEntry();
    



    public Climb() {
        climbMotor = new TalonFX(7);
        tab.addDouble("encoder value", () ->climbMotor.getPosition().getValueAsDouble());
        
    }

    public void startMotor(double speed) {
       
        speed = climbSpeed.getDouble(0);
        climbMotor.set(speed);
    }

    public void stopMotor() {
        climbMotor.set(0);
    }

    @Override
    public void periodic(){
        
    }

}
