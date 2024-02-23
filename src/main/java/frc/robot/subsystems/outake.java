package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands; 


public class outake extends SubsystemBase {
    private CANSparkFlex motor2;
    private CANSparkFlex motor1;
 
    

    public outake() {
        motor1 = new CANSparkFlex(12, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        motor2 = new CANSparkFlex(13, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
      
        

        
    }
    
    public void turnOnIndicator() {} 

    public void turnOffIndicator() {  }

    


    public void setSpeed(double speed) {  }


    public void runoutake(){
        
            motor1.set(0.75);
            motor2.set(0.75);

    
    
}
public Command outakeCommand(){

    return run(
        
    
    ()-> runoutake()
    
    );
}


}
