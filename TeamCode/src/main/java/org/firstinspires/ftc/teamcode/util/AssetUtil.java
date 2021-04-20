package org.firstinspires.ftc.teamcode.util;

import android.content.res.AssetFileDescriptor;
import android.content.res.AssetManager;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;

/**
 * TODO(BSFishy): Document this
 */
public class AssetUtil {
    public static String loadVuforiaKey() {
        return getAsset("Vuforia.key");
    }

    /**
     * TODO(BSFishy): Document this
     * @param filename
     * @return
     */
    public static String getAsset(String filename) {
        try {
            AssetManager assetManager = AppUtil.getDefContext().getAssets();
            AssetFileDescriptor afd = assetManager.openFd(filename);

            try (FileInputStream fis = afd.createInputStream()) {
                return getFileContents(fis, "UTF-8");
            }
        } catch (IOException e) {
            throw new RuntimeException("Unable to get file " + filename, e);
        }
    }

    /**
     * TODO(BSFishy): Document this
     * @param fis
     * @param encoding
     * @return
     * @throws IOException
     */
    public static String getFileContents(FileInputStream fis, String encoding) throws IOException {
        try (BufferedReader br = new BufferedReader(new InputStreamReader(fis, encoding))) {
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                sb.append(line).append('\n');
            }

            return sb.toString();
        }
    }
}
