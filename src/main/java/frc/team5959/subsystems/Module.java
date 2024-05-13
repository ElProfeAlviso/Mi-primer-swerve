package frc.team5959.subsystems;
    
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
 
    CANSparkMax driveMotor;
    CANSparkMax turnMotor;

    CANcoder turnEncoder;
    PIDController turnPID;

    public Module(int driveMotorPort, int turnMotorPort,int CANcoderPort, double kP, double kI, double kD){

        driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);

        turnEncoder = new CANcoder(CANcoderPort);
        turnPID = new PIDController(kP, kI, kD);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(),getAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState){

        desiredState = SwerveModuleState.optimize(desiredState, getAngle());

        setSpeed(desiredState);
        setAngle(desiredState);
    }

    public void setSpeed(SwerveModuleState desiredState){
        driveMotor.set(desiredState.speedMetersPerSecond);
    }

    public void setAngle(SwerveModuleState desiredState){
        double PIDvalue = turnPID.calculate(getAngle().getDegrees(), desiredState.angle.getDegrees());
        turnMotor.set(PIDvalue);
    }

    public Rotation2d getAngle(){
        double value = turnEncoder.getAbsolutePosition().getValueAsDouble();
        return Rotation2d.fromDegrees(value);
    }

    public double getDrivePosition(){
        double position;
        
        position = driveMotor.getEncoder().getPosition();

        position *= Constants.Chassis.driveRevsToMeters; //transformar dependiendo de la trnasmision que tenemos
        
        return position;
    }
} 