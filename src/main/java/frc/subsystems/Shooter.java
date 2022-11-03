package frc.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter implements Subsystem
{
    private final TalonFX m_conveyor = new TalonFX(6);
    private final TalonSRX m_intake = new TalonSRX(7);
    private final TalonFX m_cannonRight = new TalonFX(4);
    private final TalonFX m_cannonLeft = new TalonFX(5);

    public Shooter() 
    {
        /**
         * Put constructor code here
         */

        m_cannonLeft.setInverted(true); //Right hand rule for motor direction?
    }

    public void SetActive(boolean value)
    {
        if (value)
        {
            //Values set low because I don't know which way they will be spinning
            SetCannonSpeeds(0.1);
            SetConveyerSpeed(0.3);
            SetIntakeSpeed(0.3);
        }
        else
        {
            SetCannonSpeeds(0.0);
            SetConveyerSpeed(0.0);
            SetIntakeSpeed(0.0);
        }
    }

    public void SetCannonSpeeds(double value)
    {
        //One of these is inverted, so no need for a negative value
        m_cannonLeft.set(TalonFXControlMode.PercentOutput, value);
        m_cannonRight.set(TalonFXControlMode.PercentOutput, value);
    }

    public void SetConveyerSpeed(double value)
    {
        m_conveyor.set(TalonFXControlMode.PercentOutput, value);
    }

    public void SetIntakeSpeed(double value) {
        m_intake.set(TalonSRXControlMode.PercentOutput, value);
    }
}
