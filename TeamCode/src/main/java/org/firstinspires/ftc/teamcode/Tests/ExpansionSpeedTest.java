package org.firstinspires.ftc.teamcode.Tests;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.concurrent.TimeUnit;

@TeleOp
public class ExpansionSpeedTest extends LinearOpMode {
    private ServoImplEx servo, servo2;
    private double hz_filter=0, hz=0, alpha=0.00001;
    private Timing.Timer timer;
    private int period = 1200;
    private long prev_t = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(ServoImplEx.class, "right_intake_arm");
        servo2 = hardwareMap.get(ServoImplEx.class, "babis");
        timer = new Timing.Timer(0, TimeUnit.NANOSECONDS);
        timer.start();

        waitForStart();

        while (opModeIsActive()) {
            double vv = (timer.elapsedTime()%2000000000)/1000000000;
            if (vv > 1) {
                servo.setPosition(2-vv);
                servo2.setPosition(2-vv);
            } else {
                servo.setPosition(vv);
                servo2.setPosition(vv);
            }
            hz = 1000000000.0/(timer.elapsedTime()-prev_t);
            hz_filter = alpha * hz + (1-alpha) * hz_filter;
            telemetry.addData("Hz", hz);
            telemetry.addData("Hz filtered", hz_filter);
            telemetry.update();

            prev_t = timer.elapsedTime();
        }
    }
}
