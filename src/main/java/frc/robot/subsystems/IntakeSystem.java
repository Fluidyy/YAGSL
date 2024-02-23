package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;


public class IntakeSystem extends SubsystemBase{
    private CANSparkMax FeederMotor;
    private CANSparkMax IntakeMotor;
    private CANSparkMax IntakePivotMotor;
    private RelativeEncoder IntakeEncoder;
    private PIDController pid = new PIDController(0.01, 0, 0);
    
    

    public IntakeSystem(){
        FeederMotor = new CANSparkMax(1,MotorType.kBrushless);
       IntakeMotor = new CANSparkMax(2,MotorType.kBrushless);
       IntakePivotMotor = new CANSparkMax(3,MotorType.kBrushless);
       IntakeEncoder = IntakeMotor.getEncoder();
       
    }
    @Override
    public void periodic(){

    }
    public void FeederMotor (){
        FeederMotor.set(.3);
    }
    public void IntakeMotor(){
        IntakeMotor.set(.3);
    }
    public void IntakePivotMotor(double speed){
        IntakePivotMotor.set(speed);
    }

    public double IntakeEncoder(){
         
        SmartDashboard.putNumber("Intake encoder", IntakeEncoder.getPosition());
        return IntakeEncoder.getPosition();

    }

    
    public Command Intakepid(double setpoint){
        pid.setSetpoint(setpoint);
        double move = pid.calculate(IntakeEncoder());
        
        return run(
            () ->  IntakePivotMotor(move)


        )
        ;

    }
    public Command intakeMotorsCommand(){
     
        return run(
            () ->  IntakeMotor()


        )
        ;

    }   

    

    public Command feederCommand(){

        return run(
            

        () -> FeederMotor()
        );
    }
}
    

