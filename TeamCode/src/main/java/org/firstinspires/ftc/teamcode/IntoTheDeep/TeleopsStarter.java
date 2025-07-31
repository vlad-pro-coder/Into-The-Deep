package org.firstinspires.ftc.teamcode.IntoTheDeep;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ControllersRelated.SimplyfiedControllers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.MainHandler;

public class TeleopsStarter {
    public static TEAM team = TEAM.RED;

    public static SimplyfiedControllers gm1;
    public static SimplyfiedControllers gm2;
    public static double tSpeed = 1, rot = 0.7;
    public static boolean reverse = true;
    public static double min = 0.4;
    Limelight3A c;

    public TeleopsStarter(HardwareMap hardwareMap,TEAM color){
        gm1 = new SimplyfiedControllers(1);
        gm2 = new SimplyfiedControllers(2);
        team = color;
    }

    public double getPowerSigned(double h, double p){
        double sgn = Math.signum(h);
        h = Math.abs(h);
        for(int i = 1; i < p; i++){
            h *= h;
        }
        return sgn * h;
    }

    public void update(){

        if(Lift.getPosition() > 500){
            tSpeed = 0.6;
        }
        else {
            tSpeed = 1;
        }
        double pow = (min - 1) / (Extendo.MaxExtension) * Extendo.MaxExtension + 1;
        Chassis.drive(
                (reverse ? -1 : 1) * getPowerSigned(gm1.left_stick_x, 3) * tSpeed,
                (reverse ? 1 : -1) * getPowerSigned(gm1.left_stick_y, 3) * tSpeed,
                getPowerSigned(gm1.right_trigger - gm1.left_trigger, 3) * tSpeed * pow * rot
        );
        gm1.update();
        gm2.update();


    }



}
