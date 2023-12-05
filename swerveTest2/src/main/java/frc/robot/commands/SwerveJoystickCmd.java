package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenixpro.sim.ChassisReference;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ModulesConstants;
import frc.robot.resources.TecbotController.ButtonType;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveDriveTrain swerveDriveTrain;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    public SwerveJoystickCmd(SwerveDriveTrain swerveDriveTrain,
    Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
    Supplier<Boolean> fieldOrientesFunction){
        this.swerveDriveTrain = swerveDriveTrain;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientesFunction;
        this.xLimiter = new SlewRateLimiter(Constants.ModulesConstants.kMaxSpeedMetersPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.ModulesConstants.kTeleDriveMaxAccelerationUnitsPerSeconds);
        this.turnLimiter = new SlewRateLimiter(ModulesConstants.kTeleDriveMaxAngularAccelerationUnitsPerSeconds);
        addRequirements(swerveDriveTrain);


        SmartDashboard.putNumber("gyro position:", RobotContainer.getSwerveDriveTrain().gyro.getAngle());
    }



    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        xSpeed = Math.abs(xSpeed) > Constants.OIConstants.kDeadband ? xSpeed : 0.0;
        xSpeed = Math.abs(ySpeed) > Constants.OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * ModulesConstants.kMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * ModulesConstants.kMaxSpeedMetersPerSecond;
        turningSpeed = turnLimiter.calculate(turningSpeed) * ModulesConstants.kTeleDriveMaxAngularAccelerationUnitsPerSeconds;

        ChassisSpeeds chassisSpeeds;

        if (fieldOrientedFunction.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveDriveTrain.getRotation());
        }else{
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = ModulesConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 

        swerveDriveTrain.setModulesStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerveDriveTrain.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

