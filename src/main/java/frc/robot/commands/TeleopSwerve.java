package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.States;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier dampen;
    private DoubleSupplier speedDial;

    private PIDController rotationController;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier dampen, DoubleSupplier speedDial) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        rotationController = new PIDController(0.01, 0, 0 );
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(3);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.dampen = dampen;
        this.speedDial = speedDial;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);
        double  rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);

        //heading direction state
        switch(States.driveState){
            case d0:

                //heading lock
               rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(0));
                break;
            case d90:

                //heading lock
                rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(90));
                break;
            case d180:

                //heading lock
                rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(180));
                break;
            case d270:

                //heading lock
                rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(270));
                break;


            case standard:
            
                //normal
                rotationVal = rotationVal * SwerveConfig.maxAngularVelocity;
                break;
        }



        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConfig.maxSpeed), 
            rotationVal,
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}