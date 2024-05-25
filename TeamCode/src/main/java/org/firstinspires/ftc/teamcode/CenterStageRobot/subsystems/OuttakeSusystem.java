package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class OuttakeSusystem extends SubsystemBase {

    public enum State {
        INTAKE,
        OUTTAKE,
        EXTREME
    }

    private State state;
    private ServoImplEx firstLinkServo, secondLinkServo; // First: Whole Arm, Second: Only Outtake
    private CRServoImpl wheelServo;

    private final double FIRST_MIN = 0.6, FIRST_MID = 0.22, FIRST_MAX = 0;
    private final double SECOND_MIN = 0.45, SECOND_MID = 0.65, SECOND_MAX = 0.9;
    private final double WHEEL_POWER_OUT = 0.7;
    private final double WHEEL_POWER_IN = 0.9;

    public OuttakeSusystem(HardwareMap hm) {
        firstLinkServo = hm.get(ServoImplEx.class, "outtake_first");
        secondLinkServo = hm.get(ServoImplEx.class, "outtake_second");
        wheelServo = hm.get(CRServoImpl.class, "outtake_wheel");

        go_intake_first();
        go_intake_second();
    }

    public void go_intake_first() {
        state = State.INTAKE;
        firstLinkServo.setPosition(FIRST_MIN);
    }

    public void go_outtake_first() {
        state = State.OUTTAKE;
        firstLinkServo.setPosition(FIRST_MID);
    }

    public void go_intake_second() {
        state = State.INTAKE;
        secondLinkServo.setPosition(SECOND_MIN);
    }

    public void go_outtake_second() {
        state = State.OUTTAKE;
        secondLinkServo.setPosition(SECOND_MID);
    }

    public void go_extreme_first() {
        state = State.EXTREME;
        firstLinkServo.setPosition(FIRST_MAX);
    }

    public void go_extreme_second() {
        state = State.EXTREME;
        secondLinkServo.setPosition(SECOND_MAX);
    }

    public void wheel_grab() {
        wheelServo.setPower(WHEEL_POWER_IN);
    }

    public void wheel_release() {
        wheelServo.setPower(-WHEEL_POWER_OUT);
    }

    public void wheel_stop() {
        wheelServo.setPower(0);
    }

    public State getState() {
        return this.state;
    }
}
