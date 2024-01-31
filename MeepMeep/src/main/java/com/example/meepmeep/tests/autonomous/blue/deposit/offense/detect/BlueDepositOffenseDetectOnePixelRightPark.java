package com.example.meepmeep.tests.autonomous.blue.deposit.offense.detect;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueDepositOffenseDetectOnePixelRightPark {
    public int position = 1;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity blueBotOne = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        blueBotOne.runAction(blueBotOne.getDrive().actionBuilder(new Pose2d(-33, 62.5, Math.toRadians(-90)))
                .splineTo(new Vector2d(-32, 34), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-36, 36), Math.toRadians(180))
                .strafeTo(new Vector2d(47.5, 36))
                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(blueBotOne)
                .start();
    }
}