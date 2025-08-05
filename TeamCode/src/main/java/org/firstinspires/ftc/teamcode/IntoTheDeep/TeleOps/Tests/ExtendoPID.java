package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class ExtendoPID extends LinearOpMode {

    public static double pos = 0,lastpos = 0;
    public static PIDCoefficients coefs= new PIDCoefficients(0,0,0);
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        Extendo.state = Extendo.ExtendoStates.GOTOPOS;
        waitForStart();


        while(opModeIsActive()){
            Extendo.pidController.setPidCoefficients(coefs);
            if(lastpos != pos) {
                Extendo.setExtendoPos(pos);
                lastpos = pos;
            }

            Extendo.update();
            RobotInitializers.Dashtelemetry.addData("extendo pos", Extendo.getPosition());
            RobotInitializers.Dashtelemetry.update();
        }
    }
}
