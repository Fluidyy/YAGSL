package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;


public class pneumatics extends SubsystemBase{


    private static final int PH_CAN_ID1 = 1;
    private static final int SOLENOID_CHANNEL1 = 1;
    private static final int SOLENOID_CHANNEL2 = 2;
    PneumaticHub m_pH1 = new PneumaticHub(PH_CAN_ID1);
    Solenoid m_solenoid1 = m_pH1.makeSolenoid(SOLENOID_CHANNEL1);
    Solenoid m_solenoid2 = m_pH1.makeSolenoid(SOLENOID_CHANNEL2);

    public pneumatics(){}
    @Override
    public void periodic(){

    }
    public void setSolenoid(){
        m_solenoid1.set(true);
        m_solenoid2.set(true);


    }
    public Command solenoidCommand(){
        return run(

        () -> setSolenoid()
            



        );



    }

}
