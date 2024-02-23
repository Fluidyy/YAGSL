package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;


public class kracken extends SubsystemBase{
    private TalonFX motor = new TalonFX(6);
    private PIDController pid = new PIDController(0.01, 0, 0);
    

    public kracken(){}
    @Override
    public void periodic(){

    }
    public void setmotor(double speed){
        motor.set(speed);
    }
    public double encoder(){
         
        SmartDashboard.putNumber("Kracken encoder", motor.getPosition().getValueAsDouble());
        return motor.getPosition().getValueAsDouble();

    }
    public Command pid(double setpoint){
        pid.setSetpoint(setpoint);
        double move = pid.calculate(encoder());
        
        return run(
            () ->  setmotor(move)


        )
        ;

    }

    

    public Command speed(double speed){

        return run(
            

        () -> setmotor(speed)
        );
    }
}
    

