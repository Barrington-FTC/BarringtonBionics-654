package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class SoundEffects {

    private final Context context;

    private MediaPlayer RocketPlayer;
    private MediaPlayer alertPlayer;

    public SoundEffects(HardwareMap hardwareMap) {
        context = hardwareMap.appContext;
    }

    // ---------- LOAD SOUNDS ----------
    public void init() {
        RocketPlayer = MediaPlayer.create(context, R.raw.rocket);

    }

    // ---------- PLAY SOUNDS ----------
    public void playRocket() {
        play(RocketPlayer);
    }

    public void playAlert() {
        play(alertPlayer);
    }

    private void play(MediaPlayer player) {
        if (player == null) return;

        if (player.isPlaying()) {
            player.seekTo(0);
        } else {
            player.start();
        }
    }

    // ---------- CLEANUP ----------
    public void release() {
        releasePlayer(RocketPlayer);
        releasePlayer(alertPlayer);
    }

    private void releasePlayer(MediaPlayer player) {
        if (player != null) {
            player.release();
        }
    }
}