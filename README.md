# ERO PAD OS v2.25 — PWA & Connection Update

## Package contents
- `index.html` — complete web application and connection-status panel.
- `ERO_PAD_OS_v2_24_COMPLETE_WEB_AND_OLED_UPDATE.ino` — ESP32-S3 firmware source with the exact same HTML embedded. Firmware logic/version remains v2.24.
- `manifest.json` — corrected PWA manifest for the GitHub Pages `/ERO-PAD/` path.
- `sw.js` — offline app-shell Service Worker, registered only on GitHub Pages.
- `icon-192.png`, `icon-512.png` — application icons.
- `README.md` — this guide.

## PWA changes
- Added the manifest link to the HTML.
- Set relative `id`, `start_url`, and `scope` so the app works under the GitHub Pages project path.
- Added Android/iOS web-app metadata and app icons.
- Added a Service Worker that caches the application shell after the first online visit.
- Added an Install ERO PAD button when Chrome emits `beforeinstallprompt`.
- The Service Worker is registered only on HTTPS GitHub Pages; it is not registered on the ESP32 HTTP origin.

## Connection behavior
- GitHub Pages shows `GITHUB PAGES PREVIEW`; it cannot directly control the ESP32 HTTP API because of browser HTTPS/mixed-content restrictions.
- Use the ESP32 hotspot and open `http://192.168.4.1/` for hardware control.
- On the ESP32 origin, the page checks the API and WebSocket and reports connected only when both are live.
- The Service Worker does not cache ESP32 API routes.

## Upload to GitHub
Upload/replace the files in the repository root:
`index.html`, `manifest.json`, `sw.js`, `icon-192.png`, `icon-512.png`, and the `.ino` source. Keep the repository's existing public Pages deployment settings.

## Test after deployment
1. Open `https://iroshanekanayake.github.io/ERO-PAD/` in Chrome.
2. Reload once after the GitHub Pages deployment completes.
3. Open Chrome menu → Install app / Add to Home screen, or use the in-page Install ERO PAD button if it appears.
4. Open the installed app once while online; then test its shell offline.
5. For hardware use, connect to the ESP32 hotspot and open `http://192.168.4.1/`.

## Important limitations
- A browser page cannot read the Android Wi-Fi SSID directly. The board-reachable state is inferred from successful ESP32 API access.
- GitHub Pages is HTTPS and the ESP32 endpoint is HTTP. A GitHub Pages tab cannot directly call the ESP32 HTTP API/WebSocket. Use the ESP32-hosted page for actual hardware control.
- Offline caching covers the application shell only. Live ESP32 functions, WAV transfers, settings saves, and other board API actions require the board connection.
- The updated HTML embedded in the `.ino` is identical to the standalone `index.html`.
- Firmware compilation and hardware testing must be performed with the project's original ESP32-S3 board configuration and libraries before flashing.
