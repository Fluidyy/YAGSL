package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;


public class Climb extends SubsystemBase{


    private static final int PH_CAN_ID1 = 1;
    private static final int SOLENOID_CHANNEL1 = 1;
    private static final int SOLENOID_CHANNEL2 = 2;
    PneumaticHub m_pH1 = new PneumaticHub(PH_CAN_ID1);
    Solenoid m_solenoid1 = m_pH1.makeSolenoid(SOLENOID_CHANNEL1);
    Solenoid m_solenoid2 = m_pH1.makeSolenoid(SOLENOID_CHANNEL2);
    private CANSparkMax ClimbMotor1;
    private CANSparkMax ClimbMotor2;


    public Climb(){
        ClimbMotor1 = new CANSparkMax(1, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        ClimbMotor2 = new CANSparkMax(2, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    }
    @Override
    public void periodic(){

    }
    public void setSolenoid(){
        m_solenoid1.set(true);
        m_solenoid2.set(true);


    }

    public void runClimbMotor(){
        ClimbMotor1.set(.3);
        ClimbMotor2.set(.3);

    }
    public Command solenoidCommand(){
        return run(

        () -> setSolenoid()
            



        );



    }
    public Command ClimbCmd(){
        return run(

        () -> runClimbMotor()
            



        );



    }    


}
