package frc.lib.leds;

//Presets!
//use functions via LEDPreset.___.___
//i.e. LEDPreset.Rainbow.kRainbow
//or LEDPreset.Solid.kRed
//for the first imput will be the same as the enum and the second will be one of them in the enums
//See Doc for more info
public class LEDPreset {
    public enum Rainbow implements ILEDPreset {
        kRainbow(-0.99),
        kParty(-0.97),
        kOcean(-0.95),
        kLava(-0.93),
        kForest(-0.91),
        kGlitter(-0.89),
        kConfetti(-0.87);

        private final double value;

        Rainbow(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum Shot implements ILEDPreset {
        kRed(-0.85),
        kBlue(-0.83),
        kWhite(-0.81);

        private final double value;

        Shot(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum Sinelon implements ILEDPreset {
        kRainbow(-0.79),
        kParty(-0.77),
        kOcean(-0.75),
        kLava(-0.73),
        kForest(-0.71);

        private final double value;

        Sinelon(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum BeatsPerMinute implements ILEDPreset {
        kRainbow(-0.69),
        kParty(-0.67),
        kOcean(-0.65),
        kLava(-0.63),
        kForest(-0.61);

        private final double value;

        BeatsPerMinute(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum Fire implements ILEDPreset {
        kRainbow(-0.59),
        kParty(-0.57);

        private final double value;

        Fire(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum Twinkles implements ILEDPreset {
        kRainbow(-0.55),
        kParty(-0.53),
        kOcean(-0.51),
        kLava(-0.49),
        kForest(-0.47);

        private final double value;

        Twinkles(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum ColorWaves implements ILEDPreset {
        kRainbow(-0.45),
        kParty(-0.43),
        kOcean(-0.41),
        kLava(-0.39),
        kForest(-0.37);

        private final double value;

        ColorWaves(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum LarsonScanner implements ILEDPreset {
        kRed(-0.35),
        kGray(-0.33);

        private final double value;

        LarsonScanner(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum LightChase implements ILEDPreset {
        kRed(-0.31),
        kBlue(-0.29),
        kGray(-0.27);

        private final double value;

        LightChase(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum Heartbeat implements ILEDPreset {
        kRed(-0.25),
        kBlue(-0.23),
        kWhite(-0.21),
        kGray(-0.19);

        private final double value;

        Heartbeat(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum Breath implements ILEDPreset {
        kRed(-0.17),
        kBlue(-0.15),
        kGray(-0.13);

        private final double value;

        Breath(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum Strobe implements ILEDPreset {
        kRed(-0.11),
        kBlue(-0.09),
        kGold(-0.07),
        kWhite(-0.05);

        private final double value;

        Strobe(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum Color1 implements ILEDPreset {
        kEndToEndBlendToBlack(-0.03),
        kLarsonScanner(-0.01),
        kLightChase(0.01),
        kHeartbeatSlow(0.03),
        kHeartbeatMedium(0.05),
        kHeartbeatFast(0.07),
        kBreathSlow(0.09),
        kBreathFast(0.11),
        kShot(0.13),
        kStrobe(0.15);

        private final double value;

        Color1(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum Color2 implements ILEDPreset {
        kEndToEndBlendToBlack(0.17),
        kLarsonScanner(0.19),
        kLightChase(0.21),
        kHeartbeatSlow(0.23),
        kHeartbeatMedium(0.25),
        kHeartbeatFast(0.27),
        kBreathSlow(0.29),
        kBreathFast(0.31),
        kShot(0.33),
        kStrobe(0.35);

        private final double value;

        Color2(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum Color1And2 implements ILEDPreset {
        kSparkle(0.37),
        kSparkleInverted(0.39),
        kGradient(0.41),
        kBeatsPerMinute(0.43),
        kEndToEndBlend(0.45),
        kEndToEndBlendInverted(0.47),
        kNoBlend(0.49),
        kTwinkles(0.51),
        kColorWaves(0.53),
        kSinelon(0.55);

        private final double value;

        Color1And2(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }

    public enum Solid implements ILEDPreset {
        kHotPink(0.57),
        kDarkRed(0.59),
        kRed(0.61),
        kRedOrange(0.63),
        kOrange(0.65),
        kGold(0.67),
        kYellow(0.69),
        kLawnGreen(0.71),
        kLime(0.73),
        kDarkGreen(0.75),
        kGreen(0.77),
        kBlueGreen(0.79),
        kAqua(0.81),
        kSkyBlue(0.83),
        kDarkBlue(0.85),
        kBlue(0.87),
        kBlueViolet(0.89),
        kViolet(0.91),
        kWhite(0.93),
        kGray(0.95),
        kDarkGray(0.97),
        kBlack(0.99);

        private final double value;

        Solid(double value) {
            this.value = value;
        }

        @Override
        public double value() {
            return value;
        }
    }
}
