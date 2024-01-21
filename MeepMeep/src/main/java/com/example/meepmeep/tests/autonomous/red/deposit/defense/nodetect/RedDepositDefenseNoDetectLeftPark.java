package com.example.meepmeep.tests.autonomous.red.deposit.defense.nodetect;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedDepositDefenseNoDetectLeftPark {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity blueBotOne = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        blueBotOne.runAction(blueBotOne.getDrive().actionBuilder(new Pose2d(-33, 62.5, Math.toRadians(-90)))
                .lineToY(36)
                .waitSeconds(2.5)
                .turn(Math.toRadians(90))
                .lineToX(10)
                .build());

        // Declare out second bot
        RoadRunnerBotEntity redBotOne = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redBotOne.runAction(redBotOne.getDrive().actionBuilder(new Pose2d(-33, -62.5, Math.toRadians(90)))
                .lineToY(-20)
                .splineTo(new Vector2d(36, 10), Math.toRadians(0))
                .turnTo(Math.toRadians(90))
                .lineToY(36)
                .waitSeconds(15)
                .lineToY(-36)
                .splineTo(new Vector2d(60, -58), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(blueBotOne)
                .addEntity(redBotOne)
                .start();
    }
}