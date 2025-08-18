package org.firstinspires.ftc.teamcode.IntoTheDeep;


import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.Climb.ClimbActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.ClimbConstants.ClimbUnderWay;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.Climb.updateClimb;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.ClimbConstants.tasks;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ControllersRelated.SimplyfiedControllers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.MainHandler;

public class TeleopsStarter {
    public static TEAM team = TEAM.RED;

    public static Gamepad gm1;
    public static Gamepad gm2;
    public static double tSpeed = 1, rot = 1;
    public static boolean reverse = true;
    public static double min = 0.4;
    MainHandler ActionHandler;

    public TeleopsStarter(HardwareMap hardwareMap,TEAM color,Gamepad gm1,Gamepad gm2){
        TeleopsStarter.gm1 = gm1;
        TeleopsStarter.gm2 = gm2;
        RobotInitializers.InitializeFull(hardwareMap);
        RobotInitializers.InitializeForOperation();
        RobotInitializers.disable();
        ActionHandler = new MainHandler();
        team = color;
        ClimbUnderWay = false;
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
        RobotInitializers.clearCache();

        if(ClimbUnderWay)
        {
            updateClimb();
            return ;
        }

        if(gm1.left_stick_button && gm1.right_stick_button)
        {
            tasks.AddAnotherScheduler(ClimbActions());
            ClimbUnderWay = true;
        }

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

        //Chassis.update();
        Extendo.update();
        Lift.update();
        Localizer.Update();
        Outtake.update();

        ActionHandler.update();

        //gm1.update();
        //gm2.update();


    }



}
