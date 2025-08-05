package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TuneMotionProfile extends LinearOpMode {

    public static double a=0,v=0,d=0;
    public static double pos=0,lastpos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        Outtake.setExtensionPos(0);
        waitForStart();

        while (opModeIsActive()){
            if(lastpos != pos) {
                Outtake.setExtensionPos(pos);
                lastpos = pos;
            }
            Outtake.ExtensionProfile.acceleration = a;
            Outtake.ExtensionProfile.deceleration = d;
            Outtake.ExtensionProfile.maxVelocity = v;

            Outtake.ExtensionProfile.update();
            Outtake.Extension.setAngle(Outtake.ExtensionProfile.getPosition());
            RobotInitializers.Dashtelemetry.addData("pos",Outtake.ExtensionProfile.getPosition());
            RobotInitializers.Dashtelemetry.update();
        }


    }
}
