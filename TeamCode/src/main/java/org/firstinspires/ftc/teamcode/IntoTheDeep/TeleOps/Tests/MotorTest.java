package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class MotorTest extends LinearOpMode {
    public static int motor = 0;
    public static double power = 0;
    public static ServoTest.Hubs hub = ServoTest.Hubs.ControlHub;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeHubs(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()){
            if(hub == ServoTest.Hubs.ControlHub){
                RobotInitializers.ControlHubMotors.setMotorPower(motor, power);
            } else {
                RobotInitializers.ExpansionHubMotors.setMotorPower(motor, power);
            }

            for(int i = 0; i < 4; i++){
                telemetry.addData("c" + i, RobotInitializers.ControlHubMotors.getMotorCurrentPosition(i));
                telemetry.addData("e" + i, RobotInitializers.ExpansionHubMotors.getMotorCurrentPosition(i));
            }
            RobotInitializers.clearCache();

            telemetry.update();
        }
    }
}