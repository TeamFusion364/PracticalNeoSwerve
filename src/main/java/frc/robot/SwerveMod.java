package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.SwerveModuleConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders.
 */
public class SwerveMod{
    private HardwareConfigs hardwareConfigs;

    public int moduleNumber;
    private Rotation2d angleOffset;

    private SparkMax mAngleMotor;
    private SparkMax mDriveMotor;

    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;

    private CANcoder angleEncoder;

    public SwerveMod(int moduleNumber, SwerveModuleConstants moduleConstants)
    {
        this.hardwareConfigs = Robot.hardwareConfigs;
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
         /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "rio");

        /* Angle Motor Config */
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAngleMotor.configure(hardwareConfigs.swerveAngleSparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        /* Drive Motor Config */
        mDriveMotor = new SparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        mDriveMotor.configure(hardwareConfigs.swerveDriveSparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        configEncoders();

    }


    private void configEncoders()
    {     
        // absolute encoder   
       angleEncoder.getConfigurator().apply(hardwareConfigs.swerveCANcoderConfig); 

        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);

        relAngleEncoder = mAngleMotor.getEncoder();
        resetToAbsolute();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

        SmartDashboard.putNumber("Desired angle", desiredState.angle.getDegrees());

    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
       
        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }
 
        double velocity = desiredState.speedMetersPerSecond;
        
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        
    }

    private void setAngle(SwerveModuleState desiredState) {
       if(Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
       {
        mAngleMotor.stopMotor();
        return;
       }
       Rotation2d angle = desiredState.angle;
        
        SparkClosedLoopController controller = mAngleMotor.getClosedLoopController();
        
        controller.setReference(angle.getDegrees(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } 

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public int getModuleNumber(){
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber){
        this.moduleNumber = moduleNumber;
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getDegrees() - angleOffset.getDegrees();
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