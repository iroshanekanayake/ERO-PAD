# ERO PAD OS v2.24 — Connection Status Update

## Package files
- `ERO_PAD_OS_v2_24_COMPLETE_WEB_AND_OLED_UPDATE.ino` — complete ESP32-S3 firmware with the matching HTML embedded.
- `index.html` — same updated HTML for GitHub Pages/source control.
- `manifest.json`, `icon-192.png`, `icon-512.png` — existing PWA metadata and icons.
- `README.md` — this guide.

## Connection indicator
The app distinguishes:
- GitHub Pages preview: shows that the HTTPS GitHub page is not connected to the ESP32 control API and provides an Open ESP32 Control button.
- ESP32 API: checks `/get-current-kit` every five seconds.
- Live WebSocket: checks the `/ws` connection and reconnects automatically if it drops.
- `ESP32 BOARD CONNECTED` appears only when both the API and WebSocket are connected.

A browser page cannot read the phone's Wi-Fi SSID. `PHONE ↔ BOARD: Board reachable` is based on a successful response from the ESP32, not direct access to Android Wi-Fi details.

## Correct way to use the hardware controller
1. Connect the phone's Wi-Fi to `ERO_PAD_WIFI` (password: `12345678`).
2. If Android says the network has no internet, stay connected to it.
3. Open `http://192.168.4.1/` in Chrome. This loads the HTML from the ESP32 itself, so relative API requests and the WebSocket use the correct local origin.
4. Wait for both API and WebSocket to report connected.

## GitHub Pages limitation
GitHub Pages is HTTPS, while the ESP32 web server is HTTP. Modern browsers block an HTTPS page from calling the ESP32's HTTP API/WebSocket as mixed content. The GitHub Pages copy is therefore a preview/source copy; it cannot directly control the board. Use `http://192.168.4.1/` for hardware control.

## Browser icon
The HTML now includes an ERO PAD favicon. This changes the small site icon shown by Chrome; it is separate from the connection state shown inside the application.

## Preserved features
The existing tabs, Kit Chains, live Piezo Visualizer, Settings, Sound Manager, WAV upload, Files, Backup, calibration and Crosstalk Matrix controls are retained. The existing Crosstalk Level selector remains in the HTML.

## Validation
The HTML embedded in the `.ino` is byte-for-byte identical to `index.html`. JavaScript syntax and ZIP integrity are checked. Compile the firmware with the same ESP32-S3 board configuration and libraries before flashing. Hardware testing is still required.
