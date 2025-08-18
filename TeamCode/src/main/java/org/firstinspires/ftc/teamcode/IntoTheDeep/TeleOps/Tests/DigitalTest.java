package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers.ControlHubDigital;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers.ExpansionHubDigital;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.LimitSwitch;

@TeleOp
@Config
public class DigitalTest extends LinearOpMode {

    LimitSwitch lm;
    public static int port = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeHubs(hardwareMap);


        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();
            lm = new LimitSwitch(ExpansionHubDigital,port);
            RobotInitializers.Dashtelemetry.addData("state", lm.getState());
        }
    }
}
