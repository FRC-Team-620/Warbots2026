package org.jmhsrobotics.frc2026.util;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;

public class BallTracker {
 private int fuelCount = 0;
    private int maxFuel = 10;
    private Supplier<Pose2d> poseSupplier;



    public BallTracker(Supplier<Pose2d> robotPose, int maxFuel,int startingFuel){
        this.poseSupplier = robotPose;
        this.maxFuel = maxFuel;
        this.fuelCount = startingFuel;
    }

   

    public void addFuel(){
        this.fuelCount += 1;
    }
    
    public void updatelog(){
        var startingPos = new Translation3d(poseSupplier.get().getTranslation()).plus(new Translation3d(0, 0, 0.5));
        // .plus(new Translation3d(0,0,0.2));
        var out = new Translation3d[fuelCount];
        for(int i =0; i <fuelCount;i++){
            out[i]=startingPos.plus(new Translation3d(0,0,0.1*i));
        }
        Logger.recordOutput("fuelPos", out);

    }
}
