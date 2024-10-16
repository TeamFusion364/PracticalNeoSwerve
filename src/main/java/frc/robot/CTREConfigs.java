package frc.robot;


import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public final class CTREConfigs {
  
    public CANCoderConfiguration swerveCanCoderConfig;
    public CANcoderConfiguration swerveCANcoderConfig;

    public CTREConfigs(){
     
        swerveCanCoderConfig = new CANCoderConfiguration();

       /** Swerve CANCoder Configuration */
       swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
    }
}