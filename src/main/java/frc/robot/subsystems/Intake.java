package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class Intake extends SubsystemBase implements Loggable{

    private final CANSparkMax intakeMotor = new CANSparkMax(CANIds.kIntakeCanID, MotorType.kBrushless);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setIdleMode(IdleMode.kCoast);

        intakeMotor.burnFlash();
    }

    /*◇─◇──◇─◇
    ✨Setters✨
    ◇─◇──◇─◇*/

    @Config
    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }

}
