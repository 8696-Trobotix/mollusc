package org.firstinspires.ftc.teamcode.mollusc.utility;

import org.firstinspires.ftc.teamcode.mollusc.exception.ConfigValueMissingException;
import org.firstinspires.ftc.teamcode.mollusc.exception.AssetRetrievalException;
import org.firstinspires.ftc.teamcode.mollusc.exception.ParityException;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class Configuration {

    private HashMap<String, String> configData = new HashMap<>();

    public Configuration() throws AssetRetrievalException {
        this("config.txt");
    }
    public Configuration(String configFileName) throws AssetRetrievalException {
        Asset asset = new Asset("mollusc/" + configFileName);

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
    public boolean noValue(String key) {
        return containsKey(key) ? configData.get(key) == null : false;
    }
    public String getString(String key) throws ConfigValueMissingException {
        String value = configData.get(key);
        if (value == null) {
            throw new ConfigValueMissingException("Missing configuration value for key: " + key);
        }
        return value;
    }
    public boolean getBoolean(String key) throws ConfigValueMissingException {
        return Boolean.parseBoolean(getString(key));
    }
    public int getInteger(String key) throws ConfigValueMissingException {
        return Integer.parseInt(getString(key));
    }
    public double getDouble(String key) throws ConfigValueMissingException {
        return Double.parseDouble(getString(key));
    }

    private static LinearOpMode useLinearOpMode() throws ParityException {
        if (!(Mollusc.opMode instanceof LinearOpMode)) {
            throw new ParityException("Telemetry-based configuration is not available with iterative OpModes.");
        }
        return (LinearOpMode)Mollusc.opMode;
    }

    public static boolean inputBoolean(String caption, String trueLabel, String falseLabel, boolean defaultValue) throws ParityException {
        return inputBoolean(caption, trueLabel, falseLabel, defaultValue, "(Locked) ");
    }
    public static boolean inputBoolean(String caption, String trueLabel, String falseLabel, boolean defaultValue, String lockLabel) throws ParityException {
        LinearOpMode opMode = useLinearOpMode();

        boolean ret = defaultValue;
        Telemetry.Item item = opMode.telemetry.addData(caption, "");
        item.setRetained(true);

        while (!opMode.gamepad1.a && !opMode.isStopRequested()) {
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

    public static int inputInteger(String caption, int defaultValue, double holdWait, int delta) throws ParityException {
        return inputInteger(caption, defaultValue, holdWait, delta, null, "(Locked) ");
    }
    public static int inputInteger(String caption, int defaultValue, double holdWait, int delta, Object[] labels) throws ParityException {
        return inputInteger(caption, defaultValue, holdWait, delta, labels, "(Locked) ");
    }
    public static int inputInteger(String caption, int defaultValue, double holdWait, int delta, Object[] labels, String lockLabel) throws ParityException {
        LinearOpMode opMode = useLinearOpMode();

        int ret = defaultValue;
        Telemetry.Item item = opMode.telemetry.addData(caption, "");
        item.setRetained(true);

        while (!opMode.gamepad1.a && !opMode.isStopRequested()) {
            item.setValue(labels == null ? ret : labels[ret % labels.length]);
            opMode.telemetry.update();

            if (Controls.spacedHold("_sh1", opMode.gamepad1.dpad_up, holdWait)) {
                ret += delta;
            }
            else if (Controls.spacedHold("_sh2", opMode.gamepad1.dpad_down, holdWait)) {
                ret -= delta;
            }
        }
        while (opMode.gamepad1.a) {
            opMode.idle();
        }

        item.setCaption(lockLabel + caption);
        opMode.telemetry.update();

        item.setRetained(false);

        return labels == null ? ret : ret % labels.length;
    }

    public static double inputDouble(String caption, int defaultValue, double holdWait, double delta) throws ParityException {
        return inputDouble(caption, defaultValue, holdWait, delta, "(Locked) ");
    }
    public static double inputDouble(String caption, int defaultValue, double holdWait, double delta, String lockLabel) throws ParityException {
        LinearOpMode opMode = useLinearOpMode();

        double ret = defaultValue;
        Telemetry.Item item = opMode.telemetry.addData(caption, "");
        item.setRetained(true);

        while (!opMode.gamepad1.a && !opMode.isStopRequested()) {
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
