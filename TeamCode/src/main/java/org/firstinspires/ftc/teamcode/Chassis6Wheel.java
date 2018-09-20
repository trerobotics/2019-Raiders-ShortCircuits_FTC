package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by HS East Robotics on 9/13/2018.
 */
public class Chassis6Wheel extends Hardware {

    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;

    public void init (HardwareMap hwMap){
        leftDrive  = hwMap.get(DcMotor.class,"left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
