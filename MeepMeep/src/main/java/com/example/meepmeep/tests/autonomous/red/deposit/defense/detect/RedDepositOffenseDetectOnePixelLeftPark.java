package com.example.meepmeep.tests.autonomous.red.deposit.defense.detect;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedDepositOffenseDetectOnePixelLeftPark {
    public int position = 1;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity blueBotOne = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        blueBotOne.runAction(blueBotOne.getDrive().actionBuilder(new Pose2d(-33, -62.5, Math.toRadians(90)))
                .strafeTo(new Vector2d(-46, -40))
                .strafeTo(new Vector2d(-33, -60))
                .waitSeconds(4)
                .strafeTo(new Vector2d(36,-60))
                .strafeToLinearHeading(new Vector2d(42.5, -36), Math.toRadians(180))
                .strafeTo(new Vector2d(42.5,-12.5))
                .strafeTo(new Vector2d(60,-12.5))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(blueBotOne)
                .start();
    }
}