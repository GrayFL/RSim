package org.rsim.console;

import android.app.Activity;
import android.os.Bundle;
import android.content.Intent;
import android.content.ActivityNotFoundException;
import android.content.pm.ShortcutInfo;
import android.content.pm.ShortcutManager;
import android.graphics.drawable.Icon;
import android.net.Uri;
import android.widget.TextView;

/** Launcher entry; all robot control remains in the browser service. */
public final class MainActivity extends Activity {
    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        if (getIntent().getBooleanExtra("pin", false)) {
            TextView status = new TextView(this);
            status.setText("RSim 控制台 · 正在添加桌面入口");
            status.setTextSize(24);
            status.setPadding(32, 48, 32, 32);
            setContentView(status);
            ShortcutManager manager = getSystemService(ShortcutManager.class);
            if (manager.isRequestPinShortcutSupported()) {
                Intent launch = new Intent(this, MainActivity.class).setAction(Intent.ACTION_MAIN);
                ShortcutInfo shortcut = new ShortcutInfo.Builder(this, "rsim-console")
                    .setShortLabel("RSim 控制台")
                    .setIcon(Icon.createWithResource(this, getApplicationInfo().icon))
                    .setIntent(launch).build();
                manager.requestPinShortcut(shortcut, null);
                finish();
            } else status.setText("请从应用列表将 RSim 控制台拖到桌面。");
            return;
        }
        Intent browser = new Intent(Intent.ACTION_VIEW, Uri.parse("http://127.0.0.1:18780/"));
        browser.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
        try { startActivity(new Intent(browser).setPackage("acr.browser.barebones")); }
        catch (ActivityNotFoundException missing) { startActivity(browser); }
        finish();
    }
}
