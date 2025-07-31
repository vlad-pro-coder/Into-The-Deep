package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.ServoPlus;

@Config
public class PtoAndWheelie {

    public static ServoPlus W1,W2,PTO;

    public static double LiftingRobotWheeliePosW1 = 0, IdleRobotWheeliePosW1 = 0;
    public static double LiftingRobotWheeliePosW2 = 0, IdleRobotWheeliePosW2 = 0;
    public static double EngagePTO = 0, DisengagePTO = 0;

    public static void TiltRobot(){
        W1.setAngle(LiftingRobotWheeliePosW1);
        W2.setAngle(LiftingRobotWheeliePosW2);
    }
    public static void IdleWheeliePos(){
        W1.setAngle(IdleRobotWheeliePosW1);
        W2.setAngle(IdleRobotWheeliePosW2);
    }

    public static void engagePTO(){
        PTO.setAngle(EngagePTO);
    }
    public static void disengagePTO(){
        PTO.setAngle(DisengagePTO);
    }

}
