
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders.
 */
public class SwerveMod{
    public int moduleNumber;
    private Rotation2d angleOffset;
   // private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;

    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;

    private CANcoder angleEncoder;


    //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveMod(int moduleNumber, SwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
         /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configEncoders();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        configDriveMotor();


       // lastAngle = getState().angle;
    }


    private void configEncoders()
    {     
        // absolute encoder   
       angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig); 
       
        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);

         
        relDriveEncoder.setPositionConversionFactor(Constants.Swerve.driveRevToMeters);
        relDriveEncoder.setVelocityConversionFactor(Constants.Swerve.driveRpmToMetersPerSecond);

        
        relAngleEncoder = mAngleMotor.getEncoder();
        relAngleEncoder.setPositionConversionFactor(Constants.Swerve.DegreesPerTurnRotation);
        // in degrees/sec
        relAngleEncoder.setVelocityConversionFactor(Constants.Swerve.DegreesPerTurnRotation / 60);
    

        resetToAbsolute();
        mDriveMotor.burnFlash();
        mAngleMotor.burnFlash();
        
    }

    private void configAngleMotor()
    {
        mAngleMotor.restoreFactoryDefaults();
        SparkPIDController controller = mAngleMotor.getPIDController();
        controller.setP(Constants.Swerve.angleKP, 0);
        controller.setI(Constants.Swerve.angleKI,0);
        controller.setD(Constants.Swerve.angleKD,0);
        controller.setFF(Constants.Swerve.angleKF,0);
        controller.setOutputRange(-Constants.Swerve.anglePower, Constants.Swerve.anglePower);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleCurrentLimit);
       
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleNuetralMode);
    }

    private void configDriveMotor()
    {        
        mDriveMotor.restoreFactoryDefaults();
        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setP(Constants.Swerve.driveKP,0);
        controller.setI(Constants.Swerve.driveKI,0);
        controller.setD(Constants.Swerve.driveKD,0);
        controller.setFF(Constants.Swerve.driveKF,0);
        controller.setOutputRange(-Constants.Swerve.drivePower, Constants.Swerve.drivePower);
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveNuetralMode); 
    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
         
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

        if(mDriveMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        if(mAngleMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
       
        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }
 
        double velocity = desiredState.speedMetersPerSecond;
        
        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setReference(velocity, ControlType.kVelocity, 0);
        
    }

    private void setAngle(SwerveModuleState desiredState) {
        if(Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) {
            mAngleMotor.stopMotor();
            return;
        }
        Rotation2d angle = desiredState.angle; 
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        SparkPIDController controller = mAngleMotor.getPIDController();
        double degReference = angle.getDegrees();
        
        controller.setReference (degReference, ControlType.kPosition, 0);
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public int getModuleNumber(){
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber){
        this.moduleNumber = moduleNumber;
    }

    void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        relAngleEncoder.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            relDriveEncoder.getVelocity(),
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
            relDriveEncoder.getPosition(), 
            getAngle()
        );
    }

}