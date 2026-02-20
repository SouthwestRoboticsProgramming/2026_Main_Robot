package com.swrobotics.robot.subsystems.lights;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// TODO: Look into new WPILib LED strip patterns API
public final class LightsSubsystem extends SubsystemBase {
    private static final double kBrightness = 0.7;

    private final AddressableLED leds;
    private final AddressableLEDBuffer data;

    private final Debouncer batteryLowDebounce;
    private final PrideSequencer prideSequencer;

    private Color commandRequest = null;
    private boolean fullBright = false;

    public LightsSubsystem() {
        leds = new AddressableLED(IOAllocation.RIO.kPWM_LEDs);
        leds.setLength(Constants.kLedStripLength);

        data = new AddressableLEDBuffer(Constants.kLedStripLength);
        leds.setData(data);

        leds.start();

        batteryLowDebounce = new Debouncer(10);
        prideSequencer = new PrideSequencer();
    }

    private void showLowBattery() {
        // Flashing orange lights
        applySolid(Timer.getTimestamp() % 0.4 > 0.2 ? Color.kOrange : Color.kBlack);
    }

    private void showIdle() {
        // Lights off
        applySolid(Color.kBlack);
    }

    @Override
    public void periodic() {
        boolean batteryLow = RobotController.getBatteryVoltage() < Constants.kLowBatteryThreshold;
        boolean isBatteryLow = batteryLowDebounce.calculate(batteryLow);

        String currentPattern;
        if (isBatteryLow) {
            showLowBattery();
            currentPattern = "LowBattery";
        } else if (commandRequest != null) {
            applySolid(commandRequest);
            currentPattern = "CommandRequest";
        } else if (DriverStation.isDisabled()) {
            prideSequencer.apply(this);
            currentPattern = "Pride";
        } else {
            showIdle();
            currentPattern = "Idle";
        }

        Logger.recordOutput("Lights/Pattern", currentPattern);

        commandRequest = null;
        fullBright = false;
    }

    /** Sets all the LEDs to the same color */
    private void applySolid(Color color) {
        Color8Bit corrected = gammaCorrect(color);
        for (int i = 0; i < Constants.kLedStripLength; i++) {
            data.setLED(i, corrected);
        }
        leds.setData(data);
    }

    public static final record Stripe(Color color, float weight) {}

    // Blends between two colors
    private Color interpolate(Color a, Color b, float percent) {
        return new Color(
                MathUtil.lerp(a.red, b.red, percent),
                MathUtil.lerp(a.green, b.green, percent),
                MathUtil.lerp(a.blue, b.blue, percent)
        );
    }

    // Scroll speed is seconds per full pass through the pattern
    public void applyStripes(float scrollSpeed, Stripe... stripes) {
        float scroll;
        Color wrapColor;
        if (scrollSpeed == 0) {
            scroll = 0;
            wrapColor = stripes[stripes.length - 1].color;
        } else {
            scroll = (float) (Timer.getTimestamp() / scrollSpeed) % 1;
            wrapColor = stripes[0].color;
        }

        float totalWeight = 0;
        for (Stripe stripe : stripes)
            totalWeight += stripe.weight;

        float weightPerPixel = totalWeight / Constants.kLedStripLength;

        float position = (1 - scroll) * totalWeight;
        for (int i = 0; i < Constants.kLedStripLength; i++) {
            position += weightPerPixel;
            position %= totalWeight;

            Color color = null;
            Color nextColor = wrapColor;
            float acc = 0;
            for (Stripe stripe : stripes) {
                float end = acc + stripe.weight;
                if (end > position) {
                    if (color != null) {
                        nextColor = stripe.color;
                        break;
                    }
                    color = stripe.color;
                }
                acc = end;
            }

            // Shouldn't happen, but fallback just in case
            if (color == null)
                color = Color.kBlack;

            float diff = acc - position;
            float pixelsToEnd = diff / weightPerPixel;
            if (pixelsToEnd > 1)
                data.setLED(i, gammaCorrect(color));
            else
                data.setLED(i, gammaCorrect(interpolate(color, nextColor, 1 - pixelsToEnd)));
        }

        leds.setData(data);
    }

    // From https://learn.adafruit.com/led-tricks-gamma-correction/the-quick-fix
    private static final int[] GAMMA_CORRECTION = {
            0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
            1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
            2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
            5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
            10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
            17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
            25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
            37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
            51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
            69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
            90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
            115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
            144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
            177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
            215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

    private Color8Bit gammaCorrect(Color desiredColor) {
        // Simulation shows colors correctly, doesn't need gamma correction
        if (RobotBase.isSimulation()) {
            return new Color8Bit(desiredColor);
        }

        double brightness = fullBright ? 1.0 : kBrightness;

        int r = GAMMA_CORRECTION[(int) (desiredColor.red * brightness * 255.0)];
        int g = GAMMA_CORRECTION[(int) (desiredColor.green * brightness * 255.0)];
        int b = GAMMA_CORRECTION[(int) (desiredColor.blue * brightness * 255.0)];
        return new Color8Bit(r, g, b);
    }

    public void disabledInit() {
        prideSequencer.reset();
    }

    /**
     * Called by commands in {@link com.swrobotics.robot.commands.LightCommands}.
     * Should not be called manually.
     *
     * @param color color to set
     */
    public void setCommandRequest(Color color) {
        commandRequest = color;
    }

    public void setFullBright(boolean fullBright) {
        this.fullBright = fullBright;
    }
}
