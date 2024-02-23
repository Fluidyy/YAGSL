package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class intakesub extends SubsystemBase{
    private CANSparkMax intakemotorR = new CANSparkMax(1,MotorType.kBrushless);
    private CANSparkMax intakemotorL = new CANSparkMax(2,MotorType.kBrushless);

    public intakesub(){}
    @Override
    public void periodic(){

    }
    public void setmotor(double speed){
        intakemotorL.set(speed);
        intakemotorR.set(speed);
    }
    public Command intakeCommand(double speed){
        return run(

        () -> setmotor(speed)
            



        );



    }

}
