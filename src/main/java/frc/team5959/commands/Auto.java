package frc.team5959.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5959.subsystems.Chassis;

public class Auto extends Command{
    Chassis chassis;
    double xObjective, yObjective, zObjetive;
    int autoState = 0;

    PIDController autoDrivePID = new PIDController(0, 0, 0);
    PIDController autoRotationPID = new PIDController(0, 0, 0);

    public Auto(Chassis chassis){
        this.chassis = chassis;
    }
    @Override
    public void initialize(){ //inicia autónomo
        yObjective = 1; //un metro
        xObjective = chassis.getPose2d().getX(); //donde está el chassis actualmente
        zObjetive = chassis.getPose2d().getRotation().getDegrees();
    }

    @Override
    public void execute(){
        if (autoState ==0) {

        yObjective = 1; //un metro
        xObjective = chassis.getPose2d().getX(); //donde está el chassis actualmente
        zObjetive = chassis.getPose2d().getRotation().getDegrees();

    } else if (autoState ==1){
        
        yObjective = 1; //un metro
        xObjective = chassis.getPose2d().getX(); //donde está el chassis actualmente
        zObjetive = chassis.getPose2d().getRotation().getDegrees();
    }
    
        double xSpeed = autoDrivePID.calculate(chassis.getPose2d().getX(),xObjective);
        double ySpeed = autoDrivePID.calculate(chassis.getPose2d().getY(),yObjective);
        double zSpeed = autoRotationPID.calculate(chassis.getPose2d().getRotation().getDegrees(), zObjetive);

        if(Math.abs(xSpeed) < 0.1 && Math.abs(ySpeed) < 0.1 && Math.abs(zSpeed) < 0.1){
            autoState++;
        }

        chassis.setFieldOrientedSpeed(xSpeed, ySpeed, zSpeed);
    }

}