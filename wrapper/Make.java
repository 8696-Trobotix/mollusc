package org.firstinspires.ftc.teamcode.mollusc.wrapper;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Make {

    private HardwareMap hardwareMap;

    public Make() {
        this.hardwareMap = Mollusc.opMode.hardwareMap;
    }

    public IMU imu(
        String name, 
        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection, 
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection
    ) {
        IMU ret = hardwareMap.get(IMU.class, name);
        RevHubOrientationOnRobot IMUOrientation = new RevHubOrientationOnRobot(
            logoFacingDirection, usbFacingDirection
        );
        imu.initialize(new IMU.Parameters(IMUOrientation));
        ret.resetYaw();
        return ret;
    }

    // Make a default motor with a specific direction. Also brakes by default.
    public DcMotorEx motor(String name, DcMotorEx.Direction direction) {
        DcMotorEx ret = hardwareMap.get(DcMotorEx.class, name);
        ret.setDirection(direction);
        ret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        return ret;
    }

    // Make a default continuous rotation servo with a specific direction.
    public crservo(String name, CRServo.Direction direction) {
        CRServo ret = hardwareMap.get(CRServo.class, name);
        ret.setDirection(direction);
        return ret;
    }

    // Make a default servo.
    public servo(String name, Servo.Direction direction) {
        Servo ret = hardwareMap.get(Servo.class, name);
        ret.setDirection(direction);
        return ret;
    }
}

/*
Copyright 2023 Trobotix 8696

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
