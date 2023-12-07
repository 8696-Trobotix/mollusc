package org.firstinspires.ftc.teamcode.mollusc.utility;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;
import org.firstinspires.ftc.teamcode.mollusc.exception.ParityException;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class Configuration {

    private HashMap<String, String> configData = new HashMap<>();

    public Configuration() {
        this("config.txt")
    }
    public Configuration(String configFileName) {
        Asset asset = new Asset(Mollusc.opMode.hardwareMap.appContext, "mollusc/" + configFileName);

        String[] lines = asset.getLines();

        for (String line : lines) {
            line = line.trim();
            if (line.isEmpty() || line.startsWith("//")) {
                continue;
            }
            String[] parsed = line.split(":");
            String key = parsed[0].trim();
            String value = parsed.length > 1 ? parsed[1].trim() : null;
            configData.put(key, value);
        }
    }

    public HashMap<String, String> getConfigData() {
        return configData;
    }
    public boolean containsKey(String key) {
        return configData.containsKey(key);
    }
    // Returns true if `key` is present but maps to null.
    public boolean noValue(key) {
        return containsKey() ? get(key) == null : false;
    }
    public String get(String key) {
        return configData.get(key);
    }

    private static LinearOpMode useLinearOpMode() throws ParityException {
        if (!(Mollusc.opMode instanceof LinearOpMode)) {
            throw new ParityException("Telemetry-based configuration is not available with iterative OpModes.");
        }
        return Mollusc.opMode;
    }

    public static boolean getBoolean(String caption, String trueLabel, String falseLabel, boolean defaultValue) throws ParityException {
        getBoolean(caption, trueLabel, falseLabel, defaultValue, "(Locked) ");
    }
    public static boolean getBoolean(String caption, String trueLabel, String falseLabel, boolean defaultValue, String lockLabel) throws ParityException {
        LinearOpMode opMode = useLinearOpMode();

        boolean ret = defaultValue;
        Telemetry.Item item = opMode.telemetry.addData(caption, ret ? trueLabel : falseLabel);
        item.setRetained(true);

        while (!opMode.gamepad1.a && !isStopRequested()) {
            item.setValue(ret ? trueLabel : falseLabel);
            opMode.telemetry.update();

            if (Controls.singlePress("_sp1", opMode.gamepad1.dpad_up)) {
                ret = false;
            }
            else if (Controls.singlePress("_sp2", opMode.gamepad1.dpad_down)) {
                ret = true;
            }
        }
        while (opMode.gamepad1.a) {
            opMode.idle();
        }

        item.setCaption(lockLabel + caption);
        opMode.telemetry.update();

        item.setRetained(false);

        return ret;
    }

    public static int getInteger(String caption, int defaultValue, double holdWait) throws ParityException {
        getInteger(caption, defaultValue, holdWait, "(Locked) ", null);
    }
    public static int getInteger(String caption, int defaultValue, double holdWait, String lockLabel, String[] labels) throws ParityException {
        LinearOpMode opMode = useLinearOpMode();

        int ret = defaultValue;
        Telemetry.Item item = opMode.telemetry.addData(caption, labels == null ? ret : labels[ret % labels.length]);
        item.setRetained(true);

        while (!opMode.gamepad1.a && !isStopRequested()) {
            item.setValue(ret);
            opMode.telemetry.update();

            if (Controls.spacedHold("_sh1", opMode.gamepad1.dpad_up, holdWait)) {
                ++ret;
            }
            else if (Controls.spacedHold("_sh2", opMode.gamepad1.dpad_down, holdWait)) {
                --ret;
            }
        }
        while (opMode.gamepad1.a) {
            opMode.idle();
        }

        item.setCaption(lockLabel + caption);
        opMode.telemetry.update();

        item.setRetained(false);

        return ret;
    }

    public static double getDouble(String caption, int defaultValue, double holdWait, double delta, String lockLabel) throws ParityException {
        getDouble(caption, defaultValue, holdWait, delta, "(Locked) ");
    }
    public static double getDouble(String caption, int defaultValue, double holdWait, double delta, String lockLabel) throws ParityException {
        LinearOpMode opMode = useLinearOpMode();

        double ret = defaultValue;
        Telemetry.Item item = opMode.telemetry.addData(caption, ret);
        item.setRetained(true);

        while (!opMode.gamepad1.a && !isStopRequested()) {
            item.setValue(ret);
            opMode.telemetry.update();

            if (Controls.spacedHold("_sh3", opMode.gamepad1.dpad_up, holdWait)) {
                ret += delta;
            }
            else if (Controls.spacedHold("_sh4", opMode.gamepad1.dpad_down, holdWait)) {
                ret -= delta;
            }
        }
        while (opMode.gamepad1.a) {
            opMode.idle();
        }

        item.setCaption(lockLabel + caption);
        opMode.telemetry.update();

        item.setRetained(false);

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
