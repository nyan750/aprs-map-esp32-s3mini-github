// APRS Map Viewer for ESP32-2432S028 (standard CYD)
//  - Receives APRS stations from APRS-IS over Wi-Fi
//  - Plots them on BMP maps (map1/2/3) stored on the SD card
//  - Touch: switch map / tap a station to centre+zoom / tap again for details
//
// Display: LovyanGFX (ILI9341-class) via LGFX_S3.h - ESP32-S3 Super Mini
// Touch  : XPT2046 on its own SPI bus via the LGFX touch device
//          (T_CLK6/T_DIN5/T_DO4/T_CS2/T_IRQ1); converted point needs
//          sx = 319 - x, y as-is (proven by diag s3 v12/v17).
// Colour : pushImage path is byte-swapped RGB565 on this panel too
//          (diag s3 v12), which is exactly the stored form this code
//          already uses - no packing changes needed.
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <SPI.h>
#include <math.h>
#include <esp_heap_caps.h>
#include <esp_partition.h>
#include <esp_spi_flash.h>
#include <esp_task_wdt.h>
#include "pngle/pngle.h"   // vendored PNG decoder for map generation (MIT)
#include "lgfx/utility/lgfx_miniz.h"   // tinfl inflate, already linked via LovyanGFX
#include <esp_system.h>
#include "LGFX_S3.h"   // ESP32-S3 Super Mini + 2.8" ILI9341-class module
#include "config.h"
#include "maps_data.h"   // embedded 4-bit grayscale maps (no SD card needed)

LGFX tft;
// S3: the touch XPT2046 is an LGFX touch device on its own SPI3 bus
// (see LGFX_S3.h). There is no separate touch library and no bus
// routing to switch - tft.getTouchRaw()/getTouch() do everything.
// The original 028 touchRoute() calls are removed (no-ops here).

// ---------------- settings (phone setup portal, Plane Radar style) ----
// config.h holds only the factory defaults; the live settings are kept
// in NVS and edited from a phone browser. First-time setup (or a BOOT
// reset) uses the captive portal on the "APRSMap-Setup" AP. S3 v24
// (user request): the LAN portal is OFF during normal operation - it
// wastes cycles and answers all day for a page used a few times a
// year. Holding BOOT for 2 s while running starts it on demand
// (http://aprs-map.local / the device IP); BOOT 2 s again, or 5 min
// without a page access, stops it.
static String cfgSsid, cfgPass, cfgCall;
static int cfgPasscode = 0;
static double homeLat = FILTER_LAT, homeLon = FILTER_LON;
static int cfgRangeKm = FILTER_RANGE_KM;
static int cfgScreens = 7;   // v175N: which screens are shown (portal setting)
static WebServer cfgServer(80);
static DNSServer dnsServer;
static bool cfgServerStarted = false;   // handlers registered (once)
static bool cfgServerRunning = false;   // S3 v24: server begin()ed
static bool mdnsRunning = false;        // S3 v24
static bool runtimePortalActive = false;   // S3 v24: BOOT-started LAN portal
static bool portalApMode = false;
static uint32_t portalLastActivityMs = 0;   // last portal page access (v75)

static void loadSettings(bool skipNvs) {
  cfgSsid = WIFI_SSID;
  cfgPass = WIFI_PASSWORD;
  cfgCall = APRS_CALLSIGN;
  cfgPasscode = APRS_PASSCODE;
  homeLat = FILTER_LAT;
  homeLon = FILTER_LON;
  cfgRangeKm = FILTER_RANGE_KM;
  if (skipNvs) return;
  Preferences p;
  p.begin("aprsmap", true);
  cfgSsid = p.getString("ssid", cfgSsid);
  cfgPass = p.getString("pass", cfgPass);
  cfgCall = p.getString("call", cfgCall);
  cfgPasscode = p.getInt("passcode", cfgPasscode);
  homeLat = p.getDouble("lat", homeLat);
  homeLon = p.getDouble("lon", homeLon);
  cfgRangeKm = p.getInt("range", cfgRangeKm);
  cfgScreens = p.getInt("screens", cfgScreens);
  if (cfgScreens < 1 || cfgScreens > 7) cfgScreens = 7;
  p.end();
}

static String htmlEsc(const String& s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') o += "&amp;";
    else if (c == '"') o += "&quot;";
    else if (c == '<') o += "&lt;";
    else o += c;
  }
  return o;
}

static void handleRoot() {
  portalLastActivityMs = millis();
  char latBuf[24], lonBuf[24];
  snprintf(latBuf, sizeof(latBuf), "%.4f", homeLat);
  snprintf(lonBuf, sizeof(lonBuf), "%.4f", homeLon);
  String h = "<!DOCTYPE html><html lang='ja'><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>APRS Map Setup</title><style>body{font-family:sans-serif;margin:16px}"
    "label{display:block;margin-top:12px}input{width:100%;padding:8px;font-size:16px;box-sizing:border-box}"
    "input[type=checkbox]{width:auto;padding:0;margin-right:8px}"
    "button{margin-top:16px;width:100%;padding:12px;font-size:16px}</style></head><body>"
    "<h2>APRS Map 設定</h2><form method='POST' action='/save'>";
  h += "<label>Wi-Fi SSID<input name='ssid' value='" + htmlEsc(cfgSsid) + "'></label>";
  h += "<label>Wi-Fi パスワード<input name='pass' type='password' value='" + htmlEsc(cfgPass) + "'></label>";
  h += "<label>コールサイン<input name='call' value='" + htmlEsc(cfgCall) + "'></label>";
  h += "<label>APRS パスコード (0=自動計算)<input name='passcode' type='number' value='" + String(cfgPasscode) + "'></label>";
  h += "<label>中心の緯度<input name='lat' value='" + String(latBuf) + "'></label>";
  h += "<label>中心の経度<input name='lon' value='" + String(lonBuf) + "'></label>";
  h += "<label>受信範囲 km<input name='range' type='number' value='" + String(cfgRangeKm) + "'></label>";
  // v175N: screen selection (user request) - unselected screens
  // are not shown at all, not even as a tab in the top bar.
  h += "<p style='margin-top:16px;font-weight:bold'>表示する画面</p>";
  h += "<label><input type='checkbox' name='scrall' value='1'" + String(cfgScreens == 7 ? " checked" : "") + ">全部表示</label>";
  h += "<label><input type='checkbox' name='scr0' value='1'" + String(cfgScreens & 1 ? " checked" : "") + ">APRS</label>";
  h += "<label><input type='checkbox' name='scr1' value='1'" + String(cfgScreens & 2 ? " checked" : "") + ">FLIGHT</label>";
  h += "<label><input type='checkbox' name='scr2' value='1'" + String(cfgScreens & 4 ? " checked" : "") + ">AIS</label>";
  h += "<p>「全部表示」を選ぶと3画面すべて表示します。選ばれなかった画面は、上段のタブも含めて表示されません。1つも選ばれていない場合は全部表示になります。</p>";
  h += "<button type='submit'>保存して再起動</button></form>"
       "<p>設定は本体に保存されます。config.h は初期値です。</p>";
  // v105: the map-generation note is larger, and explains the area
  // (v103: one map matched to the centre + receive range, shared by
  // all three tabs), with the user's actual range filled in.
  h += "<p style='font-size:1.3em;line-height:1.5'><a href='/mapgen'>地図を作り直す</a><br>"
       "地図は1枚だけ作り、APRS／FLIGHT／AISの全画面で共用します。"
       "範囲は「中心の緯度経度」と「受信範囲」に合わせます（受信範囲が "
    + String(cfgRangeKm) + " km なら、中心から上下左右 " + String(cfgRangeKm)
    + " km、約 " + String(cfgRangeKm * 2) + " km 四方）。"
       "中心や受信範囲を変えたときは、地図を作り直してください。"
       "生成には約45分かかります。生成中は電源を切らないでください。</p></body></html>";
  cfgServer.send(200, "text/html", h);
}

static void handleSave() {
  portalLastActivityMs = millis();
  String ssid = cfgServer.arg("ssid");
  if (ssid.length() > 0) {
    Preferences p;
    p.begin("aprsmap", false);
    p.putString("ssid", ssid);
    p.putString("pass", cfgServer.arg("pass"));
    String call = cfgServer.arg("call");
    if (call.length() > 0) p.putString("call", call);
    p.putInt("passcode", cfgServer.arg("passcode").toInt());
    double la = cfgServer.arg("lat").toDouble();
    double lo = cfgServer.arg("lon").toDouble();
    if (la >= -90 && la <= 90 && lo >= -180 && lo <= 180 && (la != 0 || lo != 0)) {
      p.putDouble("lat", la);
      p.putDouble("lon", lo);
    }
    int rg = cfgServer.arg("range").toInt();
    if (rg >= 10 && rg <= 500) p.putInt("range", rg);
    // v175N: screen selection (checkboxes arrive only when ticked)
    int scr = 0;
    if (cfgServer.hasArg("scrall")) scr = 7;
    else {
      if (cfgServer.hasArg("scr0")) scr |= 1;
      if (cfgServer.hasArg("scr1")) scr |= 2;
      if (cfgServer.hasArg("scr2")) scr |= 4;
      if (scr == 0) scr = 7;   // none ticked: show everything
    }
    p.putInt("screens", scr);
    p.end();
  }
  cfgServer.send(200, "text/html",
    "<!DOCTYPE html><html lang='ja'><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<body style='font-family:sans-serif;margin:16px'><h2>保存しました</h2>"
    "<p>再起動します。そのままお待ちください。</p></body></html>");
  delay(1000);
  ESP.restart();
}

static void handleMapGen() {
  portalLastActivityMs = millis();
  Preferences p;
  p.begin("aprsmap", false);
  p.putBool("mgforce", true);
  p.end();
  String mp = "<!DOCTYPE html><html lang='ja'><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<body style='font-family:sans-serif;margin:16px'><h2>地図を再生成します</h2>"
    "<p>再起動して、中心の緯度経度と受信範囲（" + String(cfgRangeKm)
    + " km）に合わせた1枚の地図を作り直します（約45分かかります）。"
      "生成中は電源を切らないでください。</p></body></html>";
  cfgServer.send(200, "text/html", mp);
  delay(1000);
  ESP.restart();
}

static void handleNotFound() {
  portalLastActivityMs = millis();
  String loc = portalApMode ? "http://192.168.4.1/" : "/";
  cfgServer.sendHeader("Location", loc);
  cfgServer.send(302, "text/plain", "");
}

static void startConfigServer() {
  if (!cfgServerStarted) {
    cfgServerStarted = true;
    cfgServer.on("/", HTTP_GET, handleRoot);
    cfgServer.on("/save", HTTP_POST, handleSave);
    cfgServer.on("/mapgen", HTTP_GET, handleMapGen);
    cfgServer.onNotFound(handleNotFound);
  }
  if (!cfgServerRunning) { cfgServer.begin(); cfgServerRunning = true; }
  portalLastActivityMs = millis();
}

static void stopConfigServer() {
  if (cfgServerRunning) { cfgServer.stop(); cfgServerRunning = false; }
}

static void beginMdns() {
  if (!mdnsRunning) { MDNS.begin("aprs-map"); mdnsRunning = true; }
}

static void endMdns() {
  if (mdnsRunning) { MDNS.end(); mdnsRunning = false; }
}

static void drawPortalScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::Font2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 12);
  tft.print("SETUP MODE");
  tft.setFont(&fonts::Font0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 48);  tft.print("1. Connect phone Wi-Fi to:");
  tft.setCursor(22, 64);  tft.print("APRSMap-Setup");
  tft.setCursor(10, 84);  tft.print("2. Open in browser:");
  tft.setCursor(22, 100); tft.print("http://192.168.4.1");
  tft.setCursor(10, 120); tft.print("(setup page may open by itself)");
  tft.setCursor(10, 140); tft.print("3. Enter Wi-Fi / callsign, save.");
}

// Captive-portal mode: serve the setup page on our own AP until the
// user saves (which reboots), or the saved Wi-Fi comes back. The
// saved-Wi-Fi retry runs at most every 5 minutes and only after the
// portal has seen no access for 5 minutes, so it can never kick the
// user out mid-typing (v75; was every 45 s regardless of activity).
static void runPortal() {
  portalApMode = true;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("APRSMap-Setup");
  dnsServer.start(53, "*", WiFi.softAPIP());
  startConfigServer();
  drawPortalScreen();
  uint32_t lastRetry = millis();
  portalLastActivityMs = millis();
  while (true) {
    dnsServer.processNextRequest();
    cfgServer.handleClient();
    bool haveCreds = cfgSsid.length() > 0 && cfgSsid != "YOUR_SSID";
    if (haveCreds && millis() - lastRetry > 300000 &&
        millis() - portalLastActivityMs > 300000) {
      lastRetry = millis();
      WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
      uint32_t t0 = millis();
      while (millis() - t0 < 10000 && WiFi.status() != WL_CONNECTED) {
        dnsServer.processNextRequest();
        cfgServer.handleClient();
        delay(20);
      }
      if (WiFi.status() == WL_CONNECTED) {
        dnsServer.stop();
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_STA);
        portalApMode = false;
        stopConfigServer();   // S3 v24: runtime LAN portal stays OFF
        endMdns();
        return;
      }
    }
    delay(2);
  }
}

// Connect with the saved settings; fall back to the setup portal.
static void wifiPhase(bool forcePortal) {
  bool haveCreds = cfgSsid.length() > 0 && cfgSsid != "YOUR_SSID";
  if (!forcePortal && haveCreds) {
    tft.setFont(&fonts::Font0);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 60);
    tft.print("WiFi connecting...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
    uint32_t t0 = millis();
    while (millis() - t0 < 12000 && WiFi.status() != WL_CONNECTED) delay(100);
    if (WiFi.status() == WL_CONNECTED) {
      // S3 v24: no LAN portal during normal operation. BOOT 2 s
      // starts it on demand (runtimePortalLoop, below setup()).
      return;
    }
    WiFi.disconnect();
  }
  runPortal();
}

// BOOT (GPIO0) held AT power-on puts the ESP32 into download mode and
// the firmware never runs - so the reset gesture is: after the splash
// appears, hold BOOT for 2 s (within the first seconds) to wipe the
// saved settings and force the setup portal.
static bool bootResetWindow() {
  pinMode(0, INPUT_PULLUP);
  tft.setFont(&fonts::Font0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 40);
  tft.print("Hold BOOT 2s: reset settings");
  uint32_t winStart = millis();
  uint32_t lowStart = 0;
  while (true) {
    if (digitalRead(0) == LOW) {
      if (!lowStart) lowStart = millis();
      if (millis() - lowStart >= 2000) {
        Preferences p;
        p.begin("aprsmap", false);
        p.clear();
        p.end();
        tft.setCursor(10, 56);
        tft.print("Settings cleared!");
        delay(800);
        return true;
      }
    } else {
      lowStart = 0;
      if (millis() - winStart > 2200) break;   // no press: boot on
    }
    if (millis() - winStart > 8000) break;
    delay(20);
  }
  return false;
}

// ---------------- aircraft (ADS-B via adsb.lol, FlightRadar-style) ----
#define MAX_AC 32
struct Aircraft {
  char flight[12] = "";
  double lat = 0, lon = 0;
  float track = 0;         // deg
  long altFt = -1;         // -1 = on ground / unknown
  int gsKt = -1;
};
static Aircraft aircraft[MAX_AC];
static int acCount = 0;
static char selAcFlight[12] = "";   // aircraft shown in the detail panel
static bool detailIsAc = false;

// Route (airline / origin / destination) per callsign, from adsbdb.
struct RouteInfo {
  bool used = false;
  char flight[12] = "";
  char airline[40] = "";
  char orig[48] = "";   // "HND Tokyo Haneda International Airport"
  char dest[48] = "";
  bool ok = false;
};
static RouteInfo routeCache[12];
static int routeNext = 0;
static uint32_t lastAcFetchMs = 0;
static uint16_t* mapChunk = nullptr;   // 60-row map render chunk (38,400 B, v76)
static uint16_t* mapChunk2 = nullptr;  // S3 v5: second frame half (two-buffer compose)
static int mapSplitRow = 0;            // S3 v5: rows of the frame held by mapChunk
// v104: the ADS-B fetch runs on a Core 0 task so its multi-second
// blocking HTTPS no longer freezes touch / redraw on Core 1 (the
// user-visible ~60 s freeze). chunkMutex guards mapChunk's lifetime:
// the fetch task frees it for TLS heap while drawMap (Core 1) may be
// rendering from it. acMutex guards the aircraft table swap.
static Aircraft acBack[MAX_AC];
static volatile bool acFetchReq = false, acDataReady = false;
static volatile bool acFetchBusy = false;   // v107: a fetch is in flight
static bool colourMapActive();                // v151N: defined at genColorActive
static void fetchAisSnapshotDo();             // S3 v25: defined with the AIS code
// S3 v25: AIS snapshot (Open Waters) state
static volatile bool aisSnapReq = false;          // poll request for the Core 0 task
static uint32_t aisLastSnapOkMs = 0;              // last good snapshot poll
static uint32_t aisLastSnapMs = 0;                // last snapshot poll attempt (loop)
static SemaphoreHandle_t acMutex = nullptr, chunkMutex = nullptr;
static bool mapChunkPermanent = false;   // S3 v3: see setup()
static void freeMapChunk() {
  // S3 v3: when the permanent full-frame buffer is held, the fetch
  // paths must NOT free it for TLS headroom - on the S3 the v2
  // free/realloc cycle degraded the chunk to 60 rows (or failed
  // outright -> line-by-line pushes) once the heap fragmented, and
  // the tap redraw visibly swept top-to-bottom again. TLS keeps its
  // own 56 KB reserve plus the AIS-pause path for headroom.
  if (mapChunkPermanent) return;
  if (chunkMutex) xSemaphoreTake(chunkMutex, portMAX_DELAY);
  if (mapChunk) { free(mapChunk); mapChunk = nullptr; }
  if (mapChunk2) { free(mapChunk2); mapChunk2 = nullptr; }
  if (chunkMutex) xSemaphoreGive(chunkMutex);
}
#define MAP_CHUNK_ROWS 60
// S3 v2: the render chunk is sized dynamically - drawMap first tries
// a FULL frame (240 rows, 153,600 B) so the whole map is composed in
// RAM while the old frame stays on screen, then pushed in ONE
// transfer (~30 ms at 40 MHz) instead of 4 band pushes that visibly
// wiped the screen top-to-bottom on every tap redraw. If the full
// frame does not fit the heap it falls back to 120, then to the
// classic 60 rows. mapChunkRowsCap is the live capacity.
static int mapChunkRowsCap = MAP_CHUNK_ROWS;
static bool acFetchedOnce = false;
#define AC_POLL_MS 30000   // S3 v19: 30 s polls while the FLIGHT screen is shown (user request; was 60 s). Fetches only run on the FLIGHT screen (v16), so the extra TLS churn is bounded to viewing time; the failure backoff ladder scales from this base as before.
static int acFailStreak = 0;              // v161N: failure backoff
static uint32_t acIntervalMs = AC_POLL_MS;

static bool jsonStrField(const String& s, const char* key, int from, int to, char* out, size_t outSz) {
  int k = s.indexOf(key, from);
  if (k < 0 || k >= to) return false;
  int v = k + strlen(key);
  int e = s.indexOf('"', v);
  if (e < 0 || e > to) e = to;
  size_t n = (size_t)(e - v); if (n >= outSz) n = outSz - 1;
  memcpy(out, s.c_str() + v, n); out[n] = 0;
  return true;
}
static bool jsonNumField(const String& s, const char* key, int from, int to, double& v) {
  int k = s.indexOf(key, from);
  if (k < 0 || k >= to) return false;
  v = strtod(s.c_str() + k + strlen(key), nullptr);
  return true;
}

// Pull the aircraft within AC_DIST_NM of the filter centre from the
// community ADS-B feed and replace the table (each reply is complete).
// v130: TLS heap escrow. Two TLS clients now coexist (the persistent
// AIS socket + this ADS-B fetch), and in colour mode the heap layout
// no longer leaves the ~45 KB the adsb.lol handshake needs in
// reachable blocks - the fetch failed with http -1 (largest block
// 40,948) while the AIS TLS connect still succeeded. A 40 KB block
// reserved at boot is freed just for the fetch, so the handshake can
// carve its buffers out of one fresh contiguous hole, then taken
// back. If the re-allocation fails the fetch simply retries next
// minute without the escrow (degraded, not broken).
#define TLS_RESERVE_BYTES 57344   // v162N: 56 KB (was 40 KB)
static uint32_t lastTlsOkMs = 0;   // S3 v17: last successful TLS use (AIS connect / ADS-B 200)
static void* tlsReserve = nullptr;
static size_t tlsReserveBytes = 0;
// v164N: one re-take path for every call site. The v163N lesson:
// the AIS connect frees the 56 KB reserve, handshakes into the
// hole and leaves its session buffers there, so the full-size
// re-take failed and every later fetch ran with reserve 0x0.
// Take the largest size that still fits instead of nothing.
static void tlsReserveRetake() {
  if (tlsReserve) return;
  static const size_t sizes[] = { TLS_RESERVE_BYTES, 49152, 40960, 32768 };
  for (size_t sz : sizes) {
    tlsReserve = heap_caps_malloc(sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (tlsReserve) {
      tlsReserveBytes = sz;
      if (sz < TLS_RESERVE_BYTES)
        Serial.printf("tls reserve: shrunk to %u\n", (unsigned)sz);
      return;
    }
  }
  tlsReserveBytes = 0;
  Serial.println("tls reserve: FAILED");
}
static void fetchAircraftDo();
static void fetchAircraft() {
  if (tlsReserve) { free(tlsReserve); tlsReserve = nullptr; }
  fetchAircraftDo();
  tlsReserveRetake();
}
// S3 v23 (user request): two ADS-B sources. adsb.lol is primary,
// adsb.fi (same per-aircraft fields; its array key is "aircraft"
// instead of "ac") is the fallback - on 2026-11-21 adsb.lol spent
// the day rate-limiting (HTTP 429) and returning 0 aircraft while
// adsb.fi served the same sky normally. The source that worked
// is tried first next time; a source that answers 0 aircraft is
// only believed after the other source has had its say.
static bool acPreferFi = false;

// One source attempt: GET + parse into acBack. Returns the
// aircraft count (>= 0) on an HTTP 200 with a usable body,
// -1 on failure. Does not publish.
static int fetchFromSource(bool fi) {
  WiFiClientSecure client;
  client.setInsecure();   // (this core's WiFiClientSecure has no setBufferSizes)
  HTTPClient http;
  http.setTimeout(12000);
  int distNm = (int)(cfgRangeKm / 1.852);
  if (distNm < 10) distNm = 10;
  if (distNm > 250) distNm = 250;
  char url[128];
  if (fi) snprintf(url, sizeof(url), "https://opendata.adsb.fi/api/v2/lat/%.4f/lon/%.4f/dist/%d",
                   homeLat, homeLon, distNm);
  else snprintf(url, sizeof(url), "https://api.adsb.lol/v2/lat/%.4f/lon/%.4f/dist/%d",
                homeLat, homeLon, distNm);
  if (!http.begin(client, url)) return -1;
  int code = http.GET();
  if (code < 0) {
    // v136: capture the BearSSL error, then retry once; the
    // failures are intermittent.
    char ebuf[80];
    client.lastError(ebuf, sizeof(ebuf));
    Serial.printf("ADS-B http %d ssl: %s (free %u largest %u)\n", code, ebuf,
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    http.end();
    delay(700);
    if (!http.begin(client, url)) return -1;
    code = http.GET();
    if (code < 0) {
      client.lastError(ebuf, sizeof(ebuf));
      Serial.printf("ADS-B retry http %d ssl: %s\n", code, ebuf);
    }
  }
  if (code != 200) {
    Serial.printf("ADS-B http %d (%s, largest heap %u)\n", code, fi ? "adsb.fi" : "adsb.lol",
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    http.end(); return -1;
  }
  lastTlsOkMs = millis();   // S3 v17
  String body = http.getString();
  http.end();
  int acPos = body.indexOf("\"ac\":");
  if (acPos < 0) acPos = body.indexOf("\"aircraft\":");   // adsb.fi schema
  if (acPos < 0) { Serial.printf("ADS-B bad body, len %u\n", (unsigned)body.length()); return -1; }
  int n = 0;
  int pos = acPos;
  while (n < MAX_AC) {
    int h = body.indexOf("\"hex\":\"", pos);
    if (h < 0) break;
    int next = body.indexOf("\"hex\":\"", h + 8);
    int segEnd = next >= 0 ? next : (int)body.length();
    double la, lo, tr, gs;
    char fl[16] = "";
    if (jsonNumField(body, "\"lat\":", h, segEnd, la) &&
        jsonNumField(body, "\"lon\":", h, segEnd, lo)) {
      Aircraft& a = acBack[n];
      jsonStrField(body, "\"flight\":\"", h, segEnd, fl, sizeof(fl));
      // trim the padding spaces the feed puts in callsigns
      int L = strlen(fl);
      while (L > 0 && fl[L - 1] == ' ') fl[--L] = 0;
      strncpy(a.flight, fl[0] ? fl : "----", sizeof(a.flight) - 1);
      a.flight[sizeof(a.flight) - 1] = 0;
      a.lat = la; a.lon = lo;
      a.track = jsonNumField(body, "\"track\":", h, segEnd, tr) ? (float)tr : 0;
      a.gsKt = jsonNumField(body, "\"gs\":", h, segEnd, gs) ? (int)gs : -1;
      a.altFt = -1;
      int ak = body.indexOf("\"alt_baro\":", h);
      if (ak >= 0 && ak < segEnd) {
        const char* p = body.c_str() + ak + 11;
        if (*p != '"') a.altFt = strtol(p, nullptr, 10);   // "ground" stays -1
      }
      n++;
    }
    if (next < 0) break;
    pos = next;
  }
  return n;
}

static bool fetchAircraftInner() {
  if (WiFi.status() != WL_CONNECTED) return false;
  // v137: free the chunk for the handshake again (the v136 log
  // proved the failure is an X509 cert-chain allocation inside
  // mbedTLS - the handshake needs every KB it can get). The v134
  // pin is replaced by the drawMap gate above: while acFetchBusy
  // is set, drawMap leaves the chunk free and renders line-by-line,
  // so nothing steals the headroom mid-handshake.
  freeMapChunk();
  // (The v151N/S3 v12 AIS pause that used to sit here went with
  // the stream in S3 v27 - there is no second TLS session now.)
  // v138 diag: exact heap state the handshake starts from
  Serial.printf("ADS-B fetch start: free %u largest %u dmaFree %u dmaLargest %u reserveBytes %u\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
                (unsigned)tlsReserveBytes);
  bool zeroSeen = false;
  for (int t = 0; t < 2; t++) {
    bool fi = (t == 0) ? acPreferFi : !acPreferFi;
    int n = fetchFromSource(fi);
    if (n > 0) {
      acPreferFi = fi;
      if (acMutex) xSemaphoreTake(acMutex, portMAX_DELAY);
      memcpy(aircraft, acBack, sizeof(acBack));
      acCount = n;
      if (acMutex) xSemaphoreGive(acMutex);
      acDataReady = true;
      Serial.printf("aircraft: %d (%s)\n", n, fi ? "adsb.fi" : "adsb.lol");
      return true;
    }
    if (n == 0) zeroSeen = true;   // answered, but an empty sky is only believed after the other source too
    else if (t == 0) Serial.printf("ADS-B %s failed, trying the other source\n", fi ? "adsb.fi" : "adsb.lol");
  }
  if (zeroSeen) {
    if (acMutex) xSemaphoreTake(acMutex, portMAX_DELAY);
    acCount = 0;
    if (acMutex) xSemaphoreGive(acMutex);
    acDataReady = true;
    Serial.printf("aircraft: 0 (both sources)\n");
    return true;
  }
  return false;
}
static void fetchAircraftDo() {
  bool ok = fetchAircraftInner();
  // v161N: the endpoint is healthy (verified from the host) but
  // the device's connections are reset every cycle (NET_RECV_
  // FAILED) - the signature of per-IP rate limiting after a day
  // of 1-minute polls and test churn. Back off exponentially
  // (2, 4, 8, 15 min cap) and reset on the first success; this
  // also stops the serial error flood the user pointed at.
  if (ok) { acFailStreak = 0; acIntervalMs = AC_POLL_MS; }
  else {
    acFailStreak++;
    uint32_t iv = AC_POLL_MS << (acFailStreak > 4 ? 4 : acFailStreak);
    acIntervalMs = iv > 900000 ? 900000 : iv;
    if (acFailStreak >= 2)
      Serial.printf("ADS-B backoff: next fetch in %lu min\n",
                    (unsigned long)(acIntervalMs / 60000));
  }
}

static void acFetchTask(void* arg) {
  (void)arg;
  for (;;) {
    if (acFetchReq && !tlsReserve) {
      // v170N reserve recovery: the reserve degrades with uptime
      // (57K -> 32K -> FAILED) as retakes lose to fragmentation,
      // and FLIGHT then stays dead until a reboot. Free the map
      // chunk and re-take on the temporarily roomier pool.
      // (S3 v27: the AIS-stream drop that used to precede this
      // went with the stream.)
      freeMapChunk();
      tlsReserveRetake();
      Serial.printf("tls reserve recovery: %s (%u bytes)\n",
                    tlsReserve ? "ok" : "FAILED", (unsigned)tlsReserveBytes);
    }
    if (acFetchReq) { acFetchReq = false; acFetchBusy = true; fetchAircraft(); acFetchBusy = false; }
    if (aisSnapReq) { aisSnapReq = false; acFetchBusy = true; fetchAisSnapshotDo(); acFetchBusy = false; }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static RouteInfo* findRoute(const char* fl) {
  for (int i = 0; i < 12; i++)
    if (routeCache[i].used && !strcmp(routeCache[i].flight, fl)) return &routeCache[i];
  return nullptr;
}

// Fetch the route for one callsign (once, then cached). Blocking ~1 s.
static void ensureRoute(const char* fl) {
  if (findRoute(fl)) return;
  if (acFetchBusy) return;   // v107: never two TLS handshakes at once
  RouteInfo& r = routeCache[routeNext];
  routeNext = (routeNext + 1) % 12;
  memset(&r, 0, sizeof(r));
  r.used = true;
  strncpy(r.flight, fl, sizeof(r.flight) - 1);
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(10000);
  char url[96];
  snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/callsign/%s", fl);
  if (!http.begin(client, url)) return;
  if (http.GET() != 200) { http.end(); return; }
  String body = http.getString();
  http.end();
  int fr = body.indexOf("\"flightroute\"");
  if (fr < 0) return;
  int ap = body.indexOf("\"airline\":{", fr);
  if (ap >= 0) jsonStrField(body, "\"name\":\"", ap, ap + 300, r.airline, sizeof(r.airline));
  char iata[8] = "", nm[44] = "";
  int op = body.indexOf("\"origin\":{", fr);
  int oe = body.indexOf("\"destination\":{", fr);
  if (op >= 0 && oe > op && jsonStrField(body, "\"iata_code\":\"", op, oe, iata, sizeof(iata)) &&
      jsonStrField(body, "\"name\":\"", op, oe, nm, sizeof(nm)))
    snprintf(r.orig, sizeof(r.orig), "%s %s", iata, nm);
  if (oe >= 0 && jsonStrField(body, "\"iata_code\":\"", oe, (int)body.length(), iata, sizeof(iata)) &&
      jsonStrField(body, "\"name\":\"", oe, (int)body.length(), nm, sizeof(nm)))
    snprintf(r.dest, sizeof(r.dest), "%s %s", iata, nm);
  r.ok = r.orig[0] && r.dest[0];
}

// ---------------- ships (AIS) ----------------
// Live vessel positions streamed over a WebSocket (see the AIS engine
// further down). ShipStaticData (name / destination / type) arrives
// only occasionally, so those fields fill in whenever they turn up.
#define MAX_SHIPS 48
struct Ship {
  bool used = false;
  bool hasPos = false;
  uint32_t mmsi = 0;
  char name[24] = "";
  char dest[24] = "";
  double lat = 0, lon = 0;
  float sog = 0;          // knots
  float cog = 361;        // deg, 361 = not available
  int navStatus = -1;
  int type = -1;
  uint32_t lastMs = 0;
};
static Ship ships[MAX_SHIPS];
static uint32_t selShipMmsi = 0;   // ship centred / shown in the detail panel

static void trimSpaces(char* s) {
  int L = strlen(s);
  while (L > 0 && s[L - 1] == ' ') s[--L] = 0;
  int b = 0;
  while (s[b] == ' ') b++;
  if (b) memmove(s, s + b, strlen(s + b) + 1);
}

static Ship* findShip(uint32_t mmsi, bool create) {
  Ship* freeSlot = nullptr;
  Ship* oldest = nullptr;
  uint32_t oldestMs = 0xFFFFFFFF;
  for (int i = 0; i < MAX_SHIPS; i++) {
    if (ships[i].used && ships[i].mmsi == mmsi) return &ships[i];
    if (!ships[i].used && !freeSlot) freeSlot = &ships[i];
    if (ships[i].used && ships[i].lastMs < oldestMs) { oldestMs = ships[i].lastMs; oldest = &ships[i]; }
  }
  if (!create) return nullptr;
  Ship* s = freeSlot ? freeSlot : oldest;
  if (s) { memset(s, 0, sizeof(Ship)); s->cog = 361; s->navStatus = -1; s->type = -1; }
  return s;
}

static int countShips() {
  int n = 0;
  for (int i = 0; i < MAX_SHIPS; i++) if (ships[i].used && ships[i].hasPos) n++;
  return n;
}

static const char* shipTypeText(int t) {
  if (t >= 70 && t <= 79) return "CARGO";
  if (t >= 80 && t <= 89) return "TANKER";
  if (t >= 60 && t <= 69) return "PASSENGER";
  if (t >= 40 && t <= 49) return "HIGH-SPEED CRAFT";
  if (t == 30) return "FISHING";
  if (t == 36) return "SAILING";
  if (t == 37) return "PLEASURE BOAT";
  if (t == 52) return "TUG";
  if (t == 50) return "PILOT BOAT";
  if (t == 51) return "RESCUE";
  if (t == 35) return "MILITARY";
  if (t == 33) return "DREDGING";
  if (t == 55) return "LAW ENFORCEMENT";
  if (t == 31 || t == 32) return "TOWING";
  if (t >= 20 && t <= 29) return "WIG";
  return "";
}

// ---------------- stations ----------------
struct Station {
  bool used = false;
  char call[20] = "";
  double lat = 0, lon = 0;
  char symTable = '/', symCode = ' ';
  int  course = -1;        // deg
  int  speedKt = -1;
  bool hasAlt = false;
  long altFt = 0;
  char comment[80] = "";
  char via[36] = "";
  uint32_t lastMs = 0;
  uint32_t lastUnix = 0;   // real (NTP) time of last packet, 0 = unknown
  bool mobile = false;     // reported speed or actually moved (display policy)
};
static Station stations[MAX_STATIONS];
static volatile bool stationsDirty = true;
static uint32_t packetCount = 0;

// ---------------- station trails (v184N, user request) -------------
// Moving APRS stations draw like AIS ships (triangle along the
// course) in red, with their last 10 minutes of track as a red
// polyline. Points are recorded in upsert() whenever a station has
// actually moved >= ~30 m; a station counts as "moving" while its
// in-window track totals >= ~100 m of travel. Trails live in a
// small pool (most stations never move; DRAM is too tight for a
// per-station array) and are runtime-only - not persisted, they
// rebuild from live packets after a reboot.
#define TRAIL_SLOTS 16
#define TRAIL_PTS 16
#define TRAIL_WINDOW_MS 600000UL   // 10 minutes
struct TrailPt { float lat, lon; uint32_t ms; };
struct Trail {
  int16_t station = -1;   // stations[] index, -1 = free slot
  uint8_t cnt = 0;
  TrailPt pts[TRAIL_PTS];
};
static Trail trails[TRAIL_SLOTS];

static Trail* trailOf(int stationIdx, bool create) {
  Trail* lru = nullptr;
  uint32_t lruMs = 0xFFFFFFFF;
  for (int k = 0; k < TRAIL_SLOTS; k++) {
    if (trails[k].station == stationIdx) return &trails[k];
    if (create) {
      if (trails[k].station < 0) { trails[k].station = (int16_t)stationIdx; trails[k].cnt = 0; return &trails[k]; }
      uint32_t newest = trails[k].cnt ? trails[k].pts[trails[k].cnt - 1].ms : 0;
      if (newest < lruMs) { lruMs = newest; lru = &trails[k]; }
    }
  }
  if (create && lru) { lru->station = (int16_t)stationIdx; lru->cnt = 0; return lru; }
  return nullptr;
}

static void trailDrop(int stationIdx) {
  Trail* t = trailOf(stationIdx, false);
  if (t) { t->station = -1; t->cnt = 0; }
}

static float trailDistM(double lat1, double lon1, double lat2, double lon2) {
  double dy = (lat2 - lat1) * 110540.0;
  double dx = (lon2 - lon1) * 111320.0 * cos((lat1 + lat2) * 0.5 * DEG_TO_RAD);
  return (float)sqrt(dx * dx + dy * dy);
}

// In-window statistics of a station's trail (points are stored in
// chronological order, so the in-window ones form the tail).
static void trailStats(int i, uint32_t now, int& cnt, uint32_t& oldestMs,
                       uint32_t& newestMs, float& pathM) {
  cnt = 0; oldestMs = 0; newestMs = 0; pathM = 0;
  Trail* t = trailOf(i, false);
  if (!t) return;
  double plat = 0, plon = 0;
  for (int k = 0; k < t->cnt; k++) {
    if (now - t->pts[k].ms > TRAIL_WINDOW_MS) continue;
    if (cnt == 0) oldestMs = t->pts[k].ms;
    else pathM += trailDistM(plat, plon, t->pts[k].lat, t->pts[k].lon);
    plat = t->pts[k].lat; plon = t->pts[k].lon;
    newestMs = t->pts[k].ms;
    cnt++;
  }
}

static void trailPush(int i, double lat, double lon, uint32_t ms) {
  Trail* t = trailOf(i, true);
  if (!t) return;
  int drop = 0;   // points that have aged out of the window
  while (drop < t->cnt && ms - t->pts[drop].ms > TRAIL_WINDOW_MS) drop++;
  if (drop) {
    memmove(&t->pts[0], &t->pts[drop], (t->cnt - drop) * sizeof(TrailPt));
    t->cnt -= (uint8_t)drop;
  }
  if (t->cnt > 0) {
    TrailPt& last = t->pts[t->cnt - 1];
    if (trailDistM(last.lat, last.lon, lat, lon) < 30.0f) return;   // hop too small
  }
  if (t->cnt >= TRAIL_PTS) {   // full: drop the oldest point
    memmove(&t->pts[0], &t->pts[1], (TRAIL_PTS - 1) * sizeof(TrailPt));
    t->cnt = TRAIL_PTS - 1;
  }
  t->pts[t->cnt].lat = (float)lat;
  t->pts[t->cnt].lon = (float)lon;
  t->pts[t->cnt].ms = ms;
  t->cnt++;
}

// Bearing (deg) along the newest usable trail segment (>= 15 m),
// scanning backwards; -1 when there is none.
static int trailBearing(int i, uint32_t now) {
  Trail* t = trailOf(i, false);
  if (!t) return -1;
  int newest = -1;
  for (int k = 0; k < t->cnt; k++)
    if (now - t->pts[k].ms <= TRAIL_WINDOW_MS) newest = k;
  for (int k = newest; k >= 1; k--) {
    double lat1 = t->pts[k - 1].lat, lon1 = t->pts[k - 1].lon;
    double lat2 = t->pts[k].lat, lon2 = t->pts[k].lon;
    if (trailDistM(lat1, lon1, lat2, lon2) < 15.0f) continue;
    double dy = (lat2 - lat1) * 110540.0;
    double dx = (lon2 - lon1) * 111320.0 * cos((lat1 + lat2) * 0.5 * DEG_TO_RAD);
    int b = (int)(atan2(dx, dy) * RAD_TO_DEG);
    return (b + 360) % 360;
  }
  return -1;
}

// ---------------- view ----------------
static int    curMap = 0;
static double centerLat = homeLat, centerLon = homeLon;
static int    zoomLevel = 0;               // 0 = whole BMP fits the screen, +1 per x2 step (scale capped at x4)
static int    selected = -1;               // index into stations[]
static bool   detailOpen = false;
// Three screens (user request): APRS stations, Flight (aircraft) and
// AIS (ships), switched with the tabs in the top bar. Each screen
// keeps its own centre/zoom. AIS shares the Flight screen's map.
#define MODE_COUNT 3
static const int MODE_MAP[MODE_COUNT] = {0, 1, 1};   // APRS, Flight, AIS
static int    screenMode = 1;              // 0 = APRS, 1 = Flight, 2 = AIS (S3 v22: boot on FLIGHT, user request)
static double modeCenterLat[MODE_COUNT] = {0, 0, 0}, modeCenterLon[MODE_COUNT] = {0, 0, 0};
static int    modeZoom[MODE_COUNT] = {0, 0, 0};
static bool   listOpen = false;
static int    listPage = 0;
static bool   sdOk = false;

// ---- current view (filled by computeView(), defined after bmpInfo) ----
// The BMP is drawn at vScale (screen px per BMP px). The visible part of
// the BMP starts at source pixel (vRectX0, vRectY0), is vSrcW x vSrcH
// source px in size, and lands on screen at offset (vXOff, vYOff) -
// non-zero when the whole map fits with margins (e.g. 640x320 on 320x240).
static double vScale = 1, vRectX0 = 0, vRectY0 = 0, vSrcW = 320, vSrcH = 240;
static int    vXOff = 0, vYOff = 0;
static double vRectWest = 0, vRectNorth = 0;   // geo at the visible rect's top-left
static double vPxPerLon = 1, vPxPerLat = 1;    // screen px per degree
static void   computeView();
static double viewLatSpan();
static double viewLonSpan();
static bool   geoToPx(double lat, double lon, int& px, int& py);

// ---------------- BMP ----------------
// The map list lives here (not in config.h) since v19, so map changes
// never require editing the user's config.h again. Bounds must match
// the BMP files on the SD card exactly.
#undef MAPS
#define MAP_COUNT 2
static MapDef MAP_LIST[MAP_COUNT] = {   // non-const: bounds are replaced when generated maps are active (v79)
  { "/map1.bmp", "MAP1",  35.90, 35.30, 139.30, 140.20 },  // example bounds (Tokyo area) - replaced by the generated map
  { "/map2.bmp", "MAP2",  35.80, 35.55, 139.55, 139.95 },  // example bounds (Tokyo area)
};

// ---------------- on-device map generation (v80: synced with the M5 build v8.21) --------
// At boot, if no generated maps exist for the configured centre (or a
// regeneration was requested from the portal), the device downloads
// GSI (Geospatial Information Authority of Japan) blank + standard
// tiles for the area around home, classifies coastlines against the
// standard tiles' sea mask, downscales to the usual 640x480 4-bit
// gray + class-bit format, and stores the result in the dedicated
// "mapdata" flash partition. The renderer then reads the generated
// maps through a flash mmap instead of the baked-in maps,
// which remain the fallback. Map bounds mirror make_maps.py's layout
// relative to the centre. PNG decoding: pngle (c) kikuchan, MIT.
// Prototype simplifications vs the PC pipeline: no place-name text
// removal, no sea-component size filter, box min/fraction pooling
// instead of LANCZOS, no coastline thinning.
#define GEN_MAGIC 0x4D473501u            // 'MG5' v1
#define GEN_HDR_BYTES 0x1000
#define GEN_MAP_BYTES (153600 + 38400)   // gray + class per map
#define GEN_SCR_OFF  0x100000            // pass-1 aggregate scratch
#define GEN_SCR_GRAY GEN_SCR_OFF
#define GEN_SCR_SEA  (GEN_SCR_OFF + 307200)
#define GEN_TOTAL_END (GEN_SCR_SEA + 307200)
#define GEN_STAGE_OFF GEN_TOTAL_END   // v6: raw-tile stage up to partition end
// v123: stored SEA plane (one bit per output pixel, majority sea
// fraction from GSI pale tiles), kept OUTSIDE the two map blocks so
// pre-v123 maps still activate (they just fall back to the flood
// fill). Slot i plane at GEN_SEA_OFF + i * GEN_SEA_BYTES.
#define GEN_SEA_OFF (GEN_HDR_BYTES + 2 * GEN_MAP_BYTES)
#define GEN_SEA_BYTES 38400
#define GEN_SEA_MAGIC 0x53454131u        // 'SEA1' 

struct GenHeader {
  uint32_t magic, ver;
  double homeLat, homeLon;
  double b[2][4];                        // N, S, W, E per map
  uint32_t sum[2];                       // byte sum of gray+class
  uint32_t reserved[8];
};

static const esp_partition_t* genPart = nullptr;
static const uint8_t* genMapPtr = nullptr;   // mmap base of genPart
static spi_flash_mmap_handle_t genMmapH = 0;
static bool genMapsActive = false;
static bool genSeaValid = false;                 // v123
static const uint8_t* genSeaPtr[2] = { nullptr, nullptr };   // v123
static uint32_t mgSeaSum = 0;                     // v123: pass-2 sea byte sum
static const uint8_t* mapGrayPtr[2]  = { MAP_GRAY[0],  MAP_GRAY[1] };
static const uint8_t* mapClassPtr[2] = { MAP_CLASS[0], MAP_CLASS[1] };

static double mgGx(double lon, int z) { return (lon + 180.0) / 360.0 * 256.0 * (1 << z); }
static double mgGy(double lat, int z) {
  double r = lat * M_PI / 180.0;
  return (1.0 - log(tan(r) + 1.0 / cos(r)) / M_PI) / 2.0 * 256.0 * (1 << z);
}

static void mgShow(const char* l1, const char* l2, int pct) {
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::Font2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(8, 16);  tft.print("MAP GENERATION");
  tft.setFont(&fonts::Font0);
  tft.setCursor(8, 44);  tft.print("GSI tiles -> onboard maps");
  tft.setCursor(8, 64);  tft.print(l1);
  tft.setCursor(8, 84);  tft.print(l2 ? l2 : "");
  tft.drawRect(8, 108, 304, 14, TFT_WHITE);
  if (pct > 0) tft.fillRect(9, 109, 302 * (pct > 100 ? 100 : pct) / 100, 12, TFT_GREEN);
  tft.setCursor(8, 134); tft.printf("%d %%", pct);
  tft.setCursor(8, 170); tft.print("Do not power off.");
}

// ---- pass 1: tile streaming into per-output-row aggregates ----
static int mgMode = 0;                     // 0 = pale tile (sea bits), 1 = blank (accumulate)
static int mgTileOx = 0, mgTileOy = 0;     // global px of tile top-left
// Generation-only scratch lives in ONE heap block (allocated in
// mgOneMap, freed on exit) - as statics it overflowed DRAM BSS.
static uint8_t* mgWork = nullptr;
static const char* mgFail = "";    // last generation failure step
static pngle_t* mgPng = nullptr;   // persistent decoder (allocated in setup)
static bool mgAllocPng();          // v6: defined near mapGenBoot
static int mgRangeKm();            // v8.13: defined near mgActivate
static bool mgAllocWorkBuffers();
static void mgFreeWorkBuffers();   // v7.1 forward declaration
static bool mgAllocColourBuffers();   // v124 forward declaration
// v5.17: aggregates are two separate 10 KB allocations (mgAggGray/Sea)
static int mgLastHttp = 0;         // last non-200 tile http code
static int mgTilesOk = 0;          // tiles fetched OK in this generation
static uint8_t* mgSeaBits = nullptr;       // [256*32] sea flags of the current tile
static uint16_t* mgColW = nullptr;         // [640]
static uint8_t* mgRowH = nullptr;          // [480]
static uint8_t* mgScrGrayRow = nullptr;    // [640]
static uint8_t* mgScrSeaRow = nullptr;     // [640]
static uint8_t* mgGrayCur = nullptr;       // [640] pass 2 row being emitted
static uint8_t* mgRingInk = nullptr;       // [13*80] packed bits
static uint8_t* mgRingSea = nullptr;       // [13*80] packed bits
static uint8_t* mgRingSea50 = nullptr;     // v123: [13*80] majority-sea bits
static int mgRingRow[13];
static int mgCropX0 = 0, mgCropY0 = 0, mgCropW = 0, mgCropH = 0;
static int mgBandOy0 = 0, mgBandRows = 0;
static int mgStripX0 = 0;   // v6: output column where the current strip starts
static uint8_t* mgAggGray = nullptr;       // [bandRows][640] min luminance
static uint8_t* mgAggSea = nullptr;        // [bandRows][640] sea pixel count
static uint16_t* mgColR = nullptr;         // v124: colour aggregates [160*32]
static uint16_t* mgColG = nullptr;
static uint16_t* mgColB = nullptr;
static uint8_t*  mgColN = nullptr;
static uint32_t  mgImgSum = 0;             // v124: emitted image byte sum
static bool genColorActive = false;        // v124: stored colour image in use
static bool colourMapActive() { return genColorActive; }   // v151N
static const uint8_t* genColorPtr = nullptr;
static uint32_t mgWdtCtr = 0;
static long mgPixels = 0;          // v6.3: pixels drawn by the current decode
static char mgFailDetail[80] = ""; // v6.3: extra line for the FAILED screen
RTC_DATA_ATTR static uint32_t mgCkpt = 0;   // v8.6: survives warm resets:
// (map+1)<<24 | strip<<8 | phase (1=fetch, 2=decode, 3=classify)
static int mgStripNo = 0;

static void mgAccum(int gx, int gy, int r, int g, int b, int lx, int ly) {
  int cx = gx - mgCropX0, cy = gy - mgCropY0;
  if (cx < 0 || cy < 0 || cx >= mgCropW || cy >= mgCropH) return;
  int oy = (int)((double)cy * 480.0 / mgCropH);
  int ox = (int)((double)cx * 640.0 / mgCropW);
  int lo = oy - mgBandOy0;
  if (lo < 0 || lo >= mgBandRows) return;
  int lum = (r * 77 + g * 150 + b * 29) >> 8;
  int sx = ox - mgStripX0;   // v6: aggregates cover one 160-col strip
  if (sx < 0 || sx >= 160) return;
  uint8_t* gm = mgAggGray + lo * 160 + sx;
  if (lum < *gm) *gm = (uint8_t)lum;
  if (mgSeaBits[ly * 32 + (lx >> 3)] & (0x80 >> (lx & 7))) {
    uint8_t* sc = mgAggSea + lo * 160 + sx;
    if (*sc < 255) (*sc)++;
  }
}

// v124: colour accumulation - the same crop->output scatter as
// mgAccum, but averaging the pale tile's true colours per output
// pixel (sums in uint16: at most ~30 source px per output px).
static void mgAccumC(int gx, int gy, int r, int g, int b) {
  int cx = gx - mgCropX0, cy = gy - mgCropY0;
  if (cx < 0 || cy < 0 || cx >= mgCropW || cy >= mgCropH) return;
  int oy = (int)((double)cy * 480.0 / mgCropH);
  int ox = (int)((double)cx * 640.0 / mgCropW);
  int lo = oy - mgBandOy0;
  if (lo < 0 || lo >= mgBandRows) return;
  int sx = ox - mgStripX0;
  if (sx < 0 || sx >= 160) return;
  int i = lo * 160 + sx;
  mgColR[i] = (uint16_t)(mgColR[i] + r);
  mgColG[i] = (uint16_t)(mgColG[i] + g);
  mgColB[i] = (uint16_t)(mgColB[i] + b);
  if (mgColN[i] < 255) mgColN[i]++;
}

static void mgDrawCb(pngle_t* png, uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                     const uint8_t rgba[4]) {
  (void)png;
  mgPixels += (long)w * (long)h;   // v6.3 diagnostics
  for (uint32_t dy = 0; dy < h; dy++)
    for (uint32_t dx = 0; dx < w; dx++) {
      int lx = (int)x + (int)dx, ly = (int)y + (int)dy;
      if (lx > 255 || ly > 255) continue;
      if (mgMode == 0) {
        // v123: the sea tile layer is now GSI "pale" - its water is
        // ONE flat colour (190,210,255), verified on a real tile.
        // The old std-tile heuristic (b-r>25) also fired on std's
        // park/river blues and fed the flood fill years of false
        // coast ink. Shape test with AA tolerance:
        int bb = rgba[2], gg = rgba[1], rr = rgba[0];
        if (bb >= 250 && bb - rr >= 40 && bb - gg >= 25 && rr < 230)
          mgSeaBits[ly * 32 + (lx >> 3)] |= (uint8_t)(0x80 >> (lx & 7));
      } else if (mgMode == 1) {
        mgAccum(mgTileOx + lx, mgTileOy + ly, rgba[0], rgba[1], rgba[2], lx, ly);
      } else if (mgMode == 2) {   // v124: colour accumulate
        mgAccumC(mgTileOx + lx, mgTileOy + ly, rgba[0], rgba[1], rgba[2]);
      }
    }
  if ((++mgWdtCtr & 0x3FF) == 0) esp_task_wdt_reset();
}

static WiFiClientSecure mgTls;     // persistent: per-tile TLS handshakes
static HTTPClient mgHttp;          // exhausted the heap (http -1)
static bool mgTlsInit = false;

static bool __attribute__((noinline)) mgFetchTile(int z, int tx, int ty, const char* layer, int mode) {
  char url[96];
  snprintf(url, sizeof(url), "https://cyberjapandata.gsi.go.jp/xyz/%s/%d/%d/%d.png",
           layer, z, tx, ty);
  if (!mgTlsInit) { mgTls.setInsecure(); mgTlsInit = true; }
  for (int attempt = 0; attempt < 3; attempt++) {
    mgHttp.setReuse(true);
    mgHttp.setTimeout(15000);
    if (!mgHttp.begin(mgTls, url)) { mgLastHttp = -101; return false; }
    mgHttp.addHeader("User-Agent", "MuseAPRSMap/1.0 (personal APRS viewer project)");
    int code = mgHttp.GET();
    if (code != 200) {
      mgLastHttp = code;
      Serial.printf("MAPGEN tile %s z%d %d/%d http %d\n", layer, z, tx, ty, code);
      mgHttp.end();
      mgTls.stop();   // force a fresh handshake on the retry
      delay(400);
      continue;
    }
    mgMode = mode;
    mgTileOx = tx * 256;
    mgTileOy = ty * 256;
    if (mode == 0) memset(mgSeaBits, 0, 8192);
    if (!mgPng) { mgLastHttp = -100; mgHttp.end(); return false; }
    pngle_t* png = mgPng;   // persistent instance (see setup)
    pngle_reset(png);
    WiFiClient* stream = mgHttp.getStreamPtr();
    int total = mgHttp.getSize();   // Content-Length (-1 if unknown)
    int fed = 0;
    uint8_t buf[1024];
    bool ok = true;
    uint32_t t0 = millis();
    while (total < 0 ? mgHttp.connected() : fed < total) {
      size_t avail = stream->available();
      if (avail) {
        if (avail > sizeof(buf)) avail = sizeof(buf);
        int n = stream->readBytes(buf, avail);
        if (n > 0) {
          fed += n;
          if (pngle_feed(png, buf, n) < 0) { ok = false; break; }
        }
      } else {
        delay(1);
      }
      if (millis() - t0 > 30000) { ok = false; break; }
      esp_task_wdt_reset();
    }
    mgHttp.end();
    if (ok) { mgTilesOk++; return true; }
    mgTls.stop();
    delay(400);
  }
  snprintf(mgFailDetail, sizeof(mgFailDetail),
           "z%d tx%d ty%d %s code %d heap %u", z, tx, ty, layer,
           mgLastHttp, (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
  return false;
}

// ---- v6 staging: fetch raw PNGs to flash, decode them later ----
// The streaming design kept the TLS session, the 44 KB PNG decoder
// and the aggregation buffers resident at the same time, and the
// heap could not serve all three (v5.13-v5.19). v6 separates the
// phases per 160-column strip: Phase A fetches tiles into the stage
// area with only the decoder resident; Phase B then decodes from
// flash - no network allocations happen while decoding.
static uint8_t mgTmpBuf[2048];   // v6.1: stage/decode/merge scratch.
// These phases never nest, so one static buffer serves all three -
// as stack arrays they overflowed the 8 KB loopTask stack (on-device
// panic: "stack overflow in task loopTask", first decode of v6).
static uint32_t mgStageUsed = 0;          // padded bytes in the stage
static uint32_t mgStagePrevUsed = 0;      // v6.6: bytes to re-erase pre-strip
static uint8_t mgStageCarry[4];
static int mgStageCarryN = 0;
static uint32_t mgRecOff[10], mgRecLen[10];
static int mgRecTx[10], mgRecCount = 0;
static int mgRecTy[10]; static uint8_t mgRecLayer[10];  // v6.4 diagnostics
static int mgDumpRec = -1;   // v6.4: record to dump on serial after a failure

static bool mgStageWriteRaw(const uint8_t* data, int n) {
  while (n > 0) {
    int k = 4 - mgStageCarryN; if (k > n) k = n;
    memcpy(mgStageCarry + mgStageCarryN, data, k);
    mgStageCarryN += k; data += k; n -= k;
    if (mgStageCarryN == 4) {
      if (esp_partition_write(genPart, GEN_STAGE_OFF + mgStageUsed,
                              mgStageCarry, 4) != ESP_OK)
        return false;
      mgStageUsed += 4;
      mgStageCarryN = 0;
    }
  }
  return true;
}
static bool mgStageEndRecord() {   // flush the tail, 0-padded to 4
  if (mgStageCarryN == 0) return true;
  while (mgStageCarryN < 4) mgStageCarry[mgStageCarryN++] = 0;
  if (esp_partition_write(genPart, GEN_STAGE_OFF + mgStageUsed,
                          mgStageCarry, 4) != ESP_OK)
    return false;
  mgStageUsed += 4;
  mgStageCarryN = 0;
  return true;
}

// Fetch one tile into the stage (record mgRecCount). Same TLS
// plumbing as mgFetchTile, but the body goes to flash untouched.
static bool mgStageTile(int z, int tx, int ty, const char* layer) {
  char url[96];
  snprintf(url, sizeof(url), "https://cyberjapandata.gsi.go.jp/xyz/%s/%d/%d/%d.png",
           layer, z, tx, ty);
  if (!mgTlsInit) { mgTls.setInsecure(); mgTlsInit = true; }
  for (int attempt = 0; attempt < 3; attempt++) {
    mgHttp.setReuse(true);
    mgHttp.setTimeout(15000);
    if (!mgHttp.begin(mgTls, url)) { mgLastHttp = -101; return false; }
    mgHttp.addHeader("User-Agent", "MuseAPRSMap/1.0 (personal APRS viewer project)");
    int code = mgHttp.GET();
    if (code != 200) {
      mgLastHttp = code;
      mgHttp.end();
      mgTls.stop();
      delay(400);
      continue;
    }
    uint32_t recOff = mgStageUsed;
    int recLen = 0;
    WiFiClient* stream = mgHttp.getStreamPtr();
    int total = mgHttp.getSize();
    uint8_t* buf = mgTmpBuf;   // 1024 of the shared scratch (v6.1)
    bool ok = true;
    uint32_t t0 = millis();
    while (total < 0 ? mgHttp.connected() : recLen < total) {
      size_t avail = stream->available();
      if (avail) {
        if (avail > 1024) avail = 1024;
        int n = stream->readBytes(buf, avail);
        if (n > 0) {
          recLen += n;
          if (!mgStageWriteRaw(buf, n)) { ok = false; break; }
        }
      } else {
        delay(1);
      }
      if (millis() - t0 > 30000) { ok = false; break; }
      esp_task_wdt_reset();
    }
    mgHttp.end();   // v8.5: per-tile session again - the v8.4 reuse
    // path is suspected in the mid-fetch reboot; the reserve block
    // (not reuse) is the fragmentation guard.
    if (ok && mgStageEndRecord()) {
      if (mgRecCount < 10) {
        mgRecOff[mgRecCount] = recOff;
        mgRecLen[mgRecCount] = (uint32_t)recLen;
        mgRecTx[mgRecCount] = tx;
        mgRecTy[mgRecCount] = 0;      // filled in by the strip loop (v6.4)
        mgRecLayer[mgRecCount] = (layer[0] == 'b') ? 1 : 0;
        mgRecCount++;
      }
      mgTilesOk++;
      return true;
    }
    mgStageUsed = recOff;   // roll the partial record back
    mgStageCarryN = 0;
    mgHttp.end();
    mgTls.stop();
    delay(400);
  }
  return false;
}

// Decode one staged tile through pngle (mode 0 = std/sea, 1 = blank).
static bool mgDecodeStaged(int rec, int ty, int mode) {
  mgMode = mode;
  mgTileOx = mgRecTx[rec] * 256;
  mgTileOy = ty * 256;
  if (mode == 0) memset(mgSeaBits, 0, 8192);
  if (!genMapPtr) { mgFail = "stage mmap"; return false; }
  pngle_reset(mgPng);
  mgPixels = 0;
  // v6.2: feed the tile in ONE pngle_feed call, straight from the
  // flash mmap (chunked feeding trips a pngle boundary bug).
  int r = pngle_feed(mgPng, genMapPtr + GEN_STAGE_OFF + mgRecOff[rec],
                     mgRecLen[rec]);
  esp_task_wdt_reset();
  if (r >= 0) return true;
  // v6.3 diagnostics: capture what the staged bytes look like via
  // both paths, then retry the decode from esp_partition_read
  // chunks - the retry verdict separates "bad staged bytes / ROM
  // inflate" (retry fails too) from "mmap feed path" (retry works).
  uint8_t mm4[4] = {0}, pr4[4] = {0};
  memcpy(mm4, genMapPtr + GEN_STAGE_OFF + mgRecOff[rec], 4);
  esp_partition_read(genPart, GEN_STAGE_OFF + mgRecOff[rec], pr4, 4);
  long px1 = mgPixels;
  pngle_reset(mgPng);
  mgPixels = 0;
  bool retryOk = true;
  {
    // BSS is too tight for a static 8 KB chunk buffer; borrow heap.
    uint8_t* rbuf = (uint8_t*)heap_caps_malloc(8192, MALLOC_CAP_DMA);
    uint32_t ch = rbuf ? 8192 : 2048;
    if (!rbuf) rbuf = mgTmpBuf;
    uint32_t off = GEN_STAGE_OFF + mgRecOff[rec];
    uint32_t left = mgRecLen[rec];
    while (left > 0) {
      uint32_t n = left > ch ? ch : left;
      if (esp_partition_read(genPart, off, rbuf, n) != ESP_OK) { retryOk = false; break; }
      if (pngle_feed(mgPng, rbuf, n) < 0) { retryOk = false; break; }
      off += n; left -= n;
      esp_task_wdt_reset();
    }
    if (rbuf != mgTmpBuf) free(rbuf);
  }
  snprintf(mgFailDetail, sizeof(mgFailDetail),
           "rec%d %s %d/%d f%d px%ld len%u %02x%02x%02x%02x retry %s",
           rec, mgRecLayer[rec] ? "blank" : "pale",
           mgRecTx[rec], mgRecTy[rec], r, px1, (unsigned)mgRecLen[rec],
           mm4[0], mm4[1], mm4[2], mm4[3], retryOk ? "OK" : "NG");
  return retryOk;
}

// First decode phase of a map: drop the TLS session first (its
// buffers go back to the heap), then allocate the work buffers and
// set up the per-map column/row weight tables.
static bool mgPrepWorkBuffers() {
  if (!mgWork) {
    // v6.5: the TLS connection now stays UP across Phase B. Killing
    // and re-establishing it per strip deterministically corrupted
    // the first tile fetched after each reconnect (staged bytes had
    // a genuine header, a wrong IDAT length and un-inflatable data,
    // identical CRC on every run). The strip-sized work buffers are
    // small enough to coexist with the live session.
    if (!mgAllocWorkBuffers()) { mgFail = "heap (gen buffers)"; return false; }
  }
  mgColW = (uint16_t*)(mgWork + 0);         // 1280
  mgRowH = mgWork + 1280;                   // 480
  mgScrGrayRow = mgWork + 1760;             // 640
  mgScrSeaRow = mgWork + 2400;              // 640
  mgGrayCur = mgWork + 3040;                // 640
  mgRingInk = mgWork + 3680;                // 1040
  mgRingSea = mgWork + 4720;                // 1040
  mgRingSea50 = mgWork + 5760;              // v123: 1040 -> ends 6800
  memset(mgColW, 0, 640 * 2);
  memset(mgRowH, 0, 480);
  for (int cx = 0; cx < mgCropW; cx++) mgColW[(int)(cx * 640.0 / mgCropW)]++;
  for (int cy = 0; cy < mgCropH; cy++) mgRowH[(int)(cy * 480.0 / mgCropH)]++;
  return true;
}

// ---- pass 2 helpers: classified emit from the scratch aggregates ----
static void mgLoadRow(int r) {
  int s = r % 13;
  if (mgRingRow[s] == r) return;
  esp_partition_read(genPart, GEN_SCR_GRAY + r * 640, mgScrGrayRow, 640);
  esp_partition_read(genPart, GEN_SCR_SEA + r * 640, mgScrSeaRow, 640);
  for (int xb = 0; xb < 80; xb++) { mgRingInk[s * 80 + xb] = 0; mgRingSea[s * 80 + xb] = 0; mgRingSea50[s * 80 + xb] = 0; }
  for (int x = 0; x < 640; x++) {
    if (mgScrGrayRow[x] < 245)
      mgRingInk[s * 80 + (x >> 3)] |= (uint8_t)(0x80 >> (x & 7));
    int den = mgColW[x] * mgRowH[r];
    if (den > 0 && (int)mgScrSeaRow[x] * 100 >= 35 * den)
      mgRingSea[s * 80 + (x >> 3)] |= (uint8_t)(0x80 >> (x & 7));
    if (den > 0 && (int)mgScrSeaRow[x] * 100 >= 50 * den)   // v123: majority sea -> stored plane
      mgRingSea50[s * 80 + (x >> 3)] |= (uint8_t)(0x80 >> (x & 7));
  }
  mgRingRow[s] = r;
}
static inline bool mgRingBit(const uint8_t* ring, int r, int x) {
  if (r < 0 || r >= 480 || x < 0 || x >= 640) return false;
  mgLoadRow(r);
  return (ring[(r % 13) * 80 + (x >> 3)] >> (7 - (x & 7))) & 1;
}
static bool mgCoastAt(int r, int x) {
  if (!mgRingBit(mgRingInk, r, x)) return false;
  for (int dy = -4; dy <= 4; dy++)
    for (int dx = -4; dx <= 4; dx++)
      if (mgRingBit(mgRingSea, r + dy, x + dx)) return true;
  return false;
}

// v6.5: walk the staged records' PNG chunk structure. The poisoned
// fetch produced records whose chunk lengths do not add up to the
// record length, so this detects them before decoding.
static bool mgStripRecordsValid() {
  if (!genMapPtr) return false;
  for (int rec = 0; rec < mgRecCount; rec++) {
    const uint8_t* p = genMapPtr + GEN_STAGE_OFF + mgRecOff[rec];
    uint32_t len = mgRecLen[rec];
    if (len < 8 || memcmp(p, "\x89PNG\r\n\x1a\n", 8) != 0) return false;
    uint32_t off = 8;
    bool sawIend = false;
    while (off + 8 <= len) {
      uint32_t clen = ((uint32_t)p[off] << 24) | ((uint32_t)p[off + 1] << 16) |
                      ((uint32_t)p[off + 2] << 8) | (uint32_t)p[off + 3];
      if (clen > len) return false;
      if (memcmp(p + off + 4, "IEND", 4) == 0) { sawIend = true; off += 12 + clen; break; }
      off += 12 + clen;
    }
    if (!sawIend || off != len) return false;
  }
  return true;
}

static bool __attribute__((noinline)) mgOneMap(int mi, const double b[4], int z, uint32_t dataOff, uint32_t* sumOut, bool colour) {
  mgStripNo = 0;   // v8.6
  mgImgSum = 0;    // v124
  double x0 = mgGx(b[2], z), x1 = mgGx(b[3], z);
  double y0 = mgGy(b[0], z), y1 = mgGy(b[1], z);
  mgCropW = (int)lround(x1 - x0);
  mgCropH = (int)lround(y1 - y0);
  mgCropX0 = (int)floor(x0);
  mgCropY0 = (int)floor(y0);
  if (mgCropW < 64 || mgCropH < 64) { mgFail = "crop size"; return false; }
  int tx0 = mgCropX0 >> 8, tx1 = (mgCropX0 + mgCropW - 1) >> 8;
  int ty0 = mgCropY0 >> 8, ty1 = (mgCropY0 + mgCropH - 1) >> 8;
  // (the work buffers, pointer carve and weight tables are set up by
  // mgPrepWorkBuffers() at the first decode phase - v6)
  // v6.6: NOR flash can only clear bits (1->0) when programming;
  // writing over a programmed cell yields old AND new. Every reused
  // region must be ERASED before rewriting - the old zero-write of
  // the sea scratch would have ANDed every later sea value to 0.
  // Both scratches are erased here, once per map, and pass 1 writes
  // each scratch row exactly once (RAM row staging below).
  if (!colour) {   // v124: colour mode uses no scratch planes
    for (uint32_t off = 0; off < 614400; off += 65536) {
      uint32_t n = 614400 - off > 65536 ? 65536 : 614400 - off;
      esp_partition_erase_range(genPart, GEN_SCR_OFF + off, n);
      esp_task_wdt_reset();
    }
  }
  char l2[48];
  int emittedY = 0;   // v125: colour emit partition guard (below)
  for (int ty = ty0; ty <= ty1; ty++) {
    int cyB0 = ty * 256 - mgCropY0; if (cyB0 < 0) cyB0 = 0;
    int cyB1 = ty * 256 + 255 - mgCropY0; if (cyB1 > mgCropH - 1) cyB1 = mgCropH - 1;
    if (cyB1 < cyB0) continue;
    int bandOy0 = (int)(cyB0 * 480.0 / mgCropH);
    int oyB = (int)(cyB1 * 480.0 / mgCropH);
    if (colour) {
      // v125 FIX: adjacent tile rows can claim the SAME boundary
      // output row (the source->output scale is ~0.3, so
      // floor((cyB1+1)*480/cropH) often equals oyB). Line mode
      // never noticed - its scratch merge accumulates across the
      // overlap - but the colour emit OVERWRITES the row, and a
      // flash re-write ANDs old and new bytes: the row corrupts
      // and the header image sum never matches (the user's
      // 'ACTFAIL colour image' loop). Clamp each band to rows not
      // yet emitted; the boundary row keeps the previous tile
      // row's samples (visually identical neighbours).
      if (bandOy0 < emittedY) bandOy0 = emittedY;
      if (oyB < bandOy0) continue;
    }
    int bandRowsFull = oyB - bandOy0 + 1;
    for (int sb = 0; sb < bandRowsFull; sb += 32) {   // v8.21: back to the v8.7 width
      mgBandOy0 = bandOy0 + sb;
      mgBandRows = bandRowsFull - sb; if (mgBandRows > 32) mgBandRows = 32;
      // (v6.9: no RAM row accumulators - the merge writes flash)
      // v6: process the sub-band in 160-column strips. Phase A stages
      // the strip's raw tiles in flash; Phase B decodes them. The two
      // phases never hold their big allocations at the same time.
      for (int sx0 = 0; sx0 < 640; sx0 += 160) {
        mgStripX0 = sx0;
        mgStripNo++; mgCkpt = ((uint32_t)(mi + 1) << 24) | ((uint32_t)(mgStripNo & 0xFFFF) << 8) | 1;   // v8.6
        int inX0 = mgCropX0 + (int)(sx0 * (double)mgCropW / 640);
        int inX1 = mgCropX0 + (int)((sx0 + 160) * (double)mgCropW / 640);
        int txA = inX0 >> 8; if (txA < tx0) txA = tx0;
        int txB = (inX1 - 1) >> 8; if (txB > tx1) txB = tx1;
        // ---- Phase A: fetch this strip's tiles into the stage ----
        // v8.3: up to 3 attempts per strip. v8.2 died at 99% on one
        // late transient fetch failure (http -1, tiles 308); a strip
        // now restages after a pause instead of aborting the run.
        bool ok = false;
        mgFailDetail[0] = 0;   // v8.5: detail belongs to this strip
        for (int attempt = 1; attempt <= 3 && !ok; attempt++) {
          if (attempt > 1) {
            mgHttp.end(); mgTls.stop();
            mgShow("Fetch failed - retrying", "",
                   (mgBandOy0 + mgBandRows) * 85 / 480);
            delay(4000 * attempt);
            esp_task_wdt_reset();
            mgStagePrevUsed = mgStageUsed;   // erase partial stage
          }
          // v6.6: erase what the previous strip/attempt left in the
          // stage - programming over programmed cells ANDs old and
          // new bytes (the deterministic "tile decode" corruption).
          if (mgStagePrevUsed) {
            uint32_t eb = (mgStagePrevUsed + 4095) & ~4095u;
            for (uint32_t o = 0; o < eb; o += 65536) {
              uint32_t n = eb - o > 65536 ? 65536 : eb - o;
              esp_partition_erase_range(genPart, GEN_STAGE_OFF + o, n);
              esp_task_wdt_reset();
            }
            mgStagePrevUsed = 0;
          }
          mgStageUsed = 0; mgStageCarryN = 0; mgRecCount = 0;
          ok = true;
          for (int tx = txA; tx <= txB && ok; tx++) {
            if (mgStageUsed > (genPart->size - GEN_STAGE_OFF) - 131072) {
              mgFail = "stage full"; return false;
            }
            ok = colour ? mgStageTile(z, tx, ty, "pale")
              : (mgStageTile(z, tx, ty, "pale") && mgStageTile(z, tx, ty, "blank"));
          }
          // v6.5: if a staged record's chunk structure is broken,
          // hard-reset the connection and refetch the strip once.
          if (ok && !mgStripRecordsValid()) {
            mgHttp.end();
            mgTls.stop();
            delay(300);
            {   // v6.6: erase the invalid records before restaging
              uint32_t eb = (mgStageUsed + 4095) & ~4095u;
              for (uint32_t o = 0; o < eb; o += 65536) {
                uint32_t n = eb - o > 65536 ? 65536 : eb - o;
                esp_partition_erase_range(genPart, GEN_STAGE_OFF + o, n);
                esp_task_wdt_reset();
              }
            }
            mgStageUsed = 0; mgStageCarryN = 0; mgRecCount = 0;
            for (int tx = txA; tx <= txB && ok; tx++) {
              ok = colour ? mgStageTile(z, tx, ty, "pale")
              : (mgStageTile(z, tx, ty, "pale") && mgStageTile(z, tx, ty, "blank"));
            }
            if (ok) for (int r2 = 0; r2 < mgRecCount; r2++) mgRecTy[r2] = ty;
            if (ok && !mgStripRecordsValid()) {   // v8.3: failed attempt
              ok = false;
              if (!mgFailDetail[0])
                snprintf(mgFailDetail, sizeof(mgFailDetail),
                         "z%d ty%d sx%d bad record heap %u", z, ty, sx0,
                         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            }
          }
        }
        snprintf(l2, sizeof(l2), "rows %d/480 heap %u", mgBandOy0 + mgBandRows,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
        mgShow("Range map (all screens)",
               l2, (mgBandOy0 + mgBandRows) * 85 / 480);
        if (!ok) { mgFail = "tile fetch"; return false; }
        for (int r2 = 0; r2 < mgRecCount; r2++) mgRecTy[r2] = ty;   // v6.4
        mgCkpt = ((uint32_t)(mi + 1) << 24) | ((uint32_t)(mgStripNo & 0xFFFF) << 8) | 2;   // v8.6
        // ---- Phase B: decode the staged tiles into the aggregates ----
        // v8.7: try the work set with the TLS session still up;
        // only if it does not fit, drop the session and retry
        // the allocation once (kept as a fallback).
        if (colour) {   // v124: colour aggregates instead of the work carve
          if (!mgAllocColourBuffers()) {
            mgHttp.end(); mgTls.stop();
            if (!mgAllocColourBuffers()) { mgFail = "heap (colour buffers)"; return false; }
          }
          memset(mgColR, 0, 160 * 32 * 2);
          memset(mgColG, 0, 160 * 32 * 2);
          memset(mgColB, 0, 160 * 32 * 2);
          memset(mgColN, 0, 160 * 32);
        } else {
          if (!mgPrepWorkBuffers()) {
            mgHttp.end(); mgTls.stop();
            if (!mgPrepWorkBuffers()) return false;
          }
          memset(mgAggGray, 0xFF, 160 * mgBandRows);
          memset(mgAggSea, 0, 160 * mgBandRows);
        }
        for (int rec = 0; rec < mgRecCount; rec++) {
          if (!mgDecodeStaged(rec, ty, colour ? 2 : ((rec & 1) ? 1 : 0))) {
            mgFail = "tile decode"; mgDumpRec = rec; return false;
          }
        }
        if (colour) {   // v124: emit this strip-band straight into
          // the stored image (strips/bands are disjoint and the
          // partition was erased, so every cell is written once).
          // Stored byte-swapped, the form pushImage displays
          // correctly on this unit (the v86 colour lesson).
          for (int lo = 0; lo < mgBandRows; lo++) {
            int oy = mgBandOy0 + lo;
            uint16_t rowBuf[160];
            uint16_t lastV = 0xFFFF;   // white, swap-invariant
            for (int sx = 0; sx < 160; sx++) {
              int ai = lo * 160 + sx;
              uint8_t n = mgColN[ai];
              uint16_t v;
              if (n) {
                int r = mgColR[ai] / n, g = mgColG[ai] / n, b2 = mgColB[ai] / n;
                uint16_t std565 = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b2 >> 3));
                v = (uint16_t)((std565 << 8) | (std565 >> 8));
                lastV = v;
              } else v = lastV;   // no sample: carry the left neighbour
              rowBuf[sx] = v;
            }
            esp_partition_write(genPart, dataOff + ((uint32_t)oy * 640 + (uint32_t)sx0) * 2,
                                rowBuf, 320);
            const uint8_t* rb = (const uint8_t*)rowBuf;
            for (int i = 0; i < 320; i++) mgImgSum += rb[i];
          }
          mgStagePrevUsed = mgStageUsed;   // v6.6: re-erase next strip
          mgFreeWorkBuffers();
          esp_task_wdt_reset();
          continue;
        }
        // v6.9: merge this strip's columns into the flash scratch.
        // AND-safety: strips own disjoint column ranges, so each
        // scratch cell is programmed with its final value exactly
        // once per map; other strips rewrite the bytes they read
        // unchanged (AND(v,v) = v). The sea baseline is the erased
        // 0xFF read as 0 - a zero-WRITE init would AND every sea
        // count to 0, which is why v6.6's RAM accumulators existed;
        // they are gone again (their 20 KB broke the heap budget).
        for (int lo = 0; lo < mgBandRows; lo++) {
          int oy = mgBandOy0 + lo;
          uint8_t* rowG = mgTmpBuf;          // shared scratch,
          uint8_t* rowS = mgTmpBuf + 640;    // not stack arrays
          esp_partition_read(genPart, GEN_SCR_GRAY + oy * 640, rowG, 640);
          esp_partition_read(genPart, GEN_SCR_SEA + oy * 640, rowS, 640);
          for (int x = sx0; x < sx0 + 160; x++) {
            int ai = lo * 160 + (x - sx0);
            if (mgAggGray[ai] < rowG[x]) rowG[x] = mgAggGray[ai];
            int base = rowS[x] == 0xFF ? 0 : rowS[x];
            int sv = base + mgAggSea[ai];
            rowS[x] = (uint8_t)(sv > 255 ? 255 : sv);
          }
          esp_partition_write(genPart, GEN_SCR_GRAY + oy * 640, rowG, 640);
          esp_partition_write(genPart, GEN_SCR_SEA + oy * 640, rowS, 640);
        }
        mgStagePrevUsed = mgStageUsed;   // v6.6: re-erase next strip
        mgFreeWorkBuffers();   // v7.1: Phase A gets a clean heap
        esp_task_wdt_reset();
      }
    }
    if (colour && oyB >= emittedY) emittedY = oyB + 1;   // v125
  }

  if (colour) { *sumOut = mgImgSum; mgFreeWorkBuffers(); return true; }   // v124
  // ---- pass 2: classify + pack + store ----
  mgCkpt = ((uint32_t)(mi + 1) << 24) | 3;   // v8.6
  // v8.1: v7.1+ frees the work buffers at each strip end; pass 2
  // uses mgWork's carve (mgGrayCur, rings), so re-acquire it here.
  if (!mgPrepWorkBuffers()) return false;
  for (int i = 0; i < 13; i++) mgRingRow[i] = -1;
  uint32_t sum = 0;
  uint32_t seaSum = 0;   // v123
  for (int r = 0; r < 480; r++) {
    esp_partition_read(genPart, GEN_SCR_GRAY + r * 640, mgGrayCur, 640);
    uint8_t grayRow[320], classRow[80], seaRow[80];
    memset(classRow, 0, sizeof(classRow));
    memset(seaRow, 0, sizeof(seaRow));
    for (int x = 0; x < 640; x++) {
      if (mgRingBit(mgRingSea50, r, x))   // v123: stored sea plane
        seaRow[x >> 3] |= (uint8_t)(0x80 >> (x & 7));
      bool ink = mgRingBit(mgRingInk, r, x);
      bool coast = ink && mgCoastAt(r, x);
      bool grown = false;
      if (!ink) {
        for (int dy = -1; dy <= 1 && !grown; dy++)
          for (int dx = -1; dx <= 1 && !grown; dx++)
            if ((dx || dy) && mgCoastAt(r + dy, x + dx)) grown = true;
      }
      if (coast || grown)
        classRow[x >> 3] |= (uint8_t)(0x80 >> (x & 7));
      int g = mgGrayCur[x];
      if (grown) g = 110;
      // v102: the v85 erase of non-coast ink is REMOVED - with the
      // line-art style back on (borders wanted), it deleted exactly
      // the municipal borders at generation time. And every pixel
      // the ink aggregate classifies as ink is floored to gd <= 200
      // below, so even the faintest municipal border lines survive
      // the contrast stretch + 4-bit quantisation instead of
      // washing out to paper (the on-device v101 report).
      int gd = 255 - (255 - g) * 16 / 10;
      if (gd < 0) gd = 0;
      if (ink && gd > 200) gd = 200;   // v102: ink floor (see above)
      int nib = gd >> 4;
      if (x & 1) grayRow[x >> 1] |= (uint8_t)nib;
      else grayRow[x >> 1] = (uint8_t)(nib << 4);
    }
    esp_partition_write(genPart, dataOff + r * 320, grayRow, 320);
    esp_partition_write(genPart, dataOff + 153600 + r * 80, classRow, 80);
    esp_partition_write(genPart, GEN_SEA_OFF + r * 80, seaRow, 80);   // v123 (slot 0; copied later)
    for (int i = 0; i < 320; i++) sum += grayRow[i];
    for (int i = 0; i < 80; i++) sum += classRow[i];
    for (int i = 0; i < 80; i++) seaSum += seaRow[i];
    if ((r & 31) == 0) {
      mgShow("Classifying coastlines...", "", 85 + r * 15 / 480);
      esp_task_wdt_reset();
    }
  }
  *sumOut = sum;
  mgSeaSum = seaSum;   // v123
  mgFreeWorkBuffers();   // v8.6: was kept allocated into the next
  // map, whose fetches then faced a resident 34 KB work set
  return true;
}

static bool __attribute__((noinline)) mgGenerateInner();
// v162N wrapper: map generation wants the heap for itself.
static bool mgGenerate() {
  if (tlsReserve) { free(tlsReserve); tlsReserve = nullptr; }
  bool ok = mgGenerateInner();
  tlsReserveRetake();
  return ok;
}
static bool __attribute__((noinline)) mgGenerateInner() {
  mgStagePrevUsed = 0;   // v6.6: full erase below covers the first strip
  // v103: ONE map matched to the setup receive range, shared by all
  // three tabs (user request). The frame is the range circle's
  // bounding square around home: half-extents in degrees from the
  // range in km. It is generated once into slot 0 and copied into
  // slot 1 below, so every existing two-slot reader (MODE_MAP, the
  // AIS bounds, the sea mask) works unchanged.
  double R = mgRangeKm();
  double halfLat = R / 111.0;
  double clat = cos(homeLat * M_PI / 180.0); if (clat < 0.2) clat = 0.2;
  double halfLon = R / (111.0 * clat);
  double b[2][4];
  for (int i = 0; i < 2; i++) {
    b[i][0] = homeLat + halfLat;
    b[i][1] = homeLat - halfLat;
    b[i][2] = homeLon - halfLon;
    b[i][3] = homeLon + halfLon;
  }
  // Tile zoom: smallest z whose source width reaches ~1500 px
  // (about 2.3x downscale to 640), clamped to [9, 12].
  int z1 = 9;
  while (z1 < 12 && (b[0][3] - b[0][2]) * 256.0 * (1 << z1) / 360.0 < 1500.0) z1++;
  int ZS[2] = { z1, z1 };
  // v6: the PNG decoder is allocated HERE - the config server is
  // stopped and no TLS session exists yet, the emptiest heap of the
  // boot (the standalone diagnostic reached GSI with ~110 KB free in
  // these conditions). The work buffers come later still, at the
  // first decode phase, after the TLS session is dropped.
  if (!mgAllocPng()) { mgFail = "heap (gen buffers)"; return false; }
  mgShow("Erasing map area...", "", 1);
  // Erase in 64 KB chunks and feed the task watchdog: one blocking
  // erase of ~2 MB takes tens of seconds and trips the WDT (v5.5 bug).
  // v6: the erase covers the tile stage area as well.
  for (uint32_t off = 0; off < genPart->size; off += 0x10000) {
    uint32_t n = genPart->size - off; if (n > 0x10000) n = 0x10000;
    if (esp_partition_erase_range(genPart, off, n) != ESP_OK) { mgFail = "flash erase"; return false; }
    esp_task_wdt_reset();
    mgShow("Erasing map area...", "", 1 + (int)(off * 4 / genPart->size));
  }
  if (!genMapPtr) {   // v6.2: decode reads the stage through the mmap
    if (esp_partition_mmap(genPart, 0, genPart->size, SPI_FLASH_MMAP_DATA,
                           (const void**)&genMapPtr, &genMmapH) != ESP_OK) {
      mgFail = "stage mmap"; return false;
    }
  }
  uint32_t sums[2] = { 0, 0 };
  // v124: generation now produces ONE stored colour image (GSI
  // pale tiles, averaged to 640x480 RGB565). No slot copy - the
  // colour renderer reads the single image for every screen.
  if (!mgOneMap(0, b[0], ZS[0], GEN_HDR_BYTES, &sums[0], true)) return false;
  sums[1] = sums[0];
  GenHeader h = {};
  h.magic = GEN_MAGIC;
  h.ver = 2;   // v124: colour image format
  h.homeLat = homeLat;
  h.homeLon = homeLon;
  memcpy(h.b, b, sizeof(b));
  h.sum[0] = sums[0];
  h.sum[1] = sums[1];
  h.reserved[0] = (uint32_t)mgRangeKm();   // v8.13: range this map was made for
  mgShow("Done", "", 100);
  if (esp_partition_write(genPart, 0, &h, sizeof(h)) != ESP_OK) { mgFail = "header write"; return false; }
  // v8.16: a backup header copy at the end of the header page, plus
  // a read-back verify - a finished map was reported lost across a
  // power cycle, its header reading back as erased (ffffffff/-1).
  if (esp_partition_write(genPart, GEN_HDR_BYTES - sizeof(h), &h, sizeof(h)) != ESP_OK) { mgFail = "header write (backup)"; return false; }
  { GenHeader v1 = {}, v2 = {};
    bool okv = esp_partition_read(genPart, 0, &v1, sizeof(v1)) == ESP_OK && v1.magic == GEN_MAGIC
            && esp_partition_read(genPart, GEN_HDR_BYTES - sizeof(v2), &v2, sizeof(v2)) == ESP_OK && v2.magic == GEN_MAGIC;
    if (!okv) { mgFail = "header verify"; return false; } }
  return true;
}

static char mgActFail[64] = "";   // v8.1: why the last activate failed
static int mgHdrRangeKm = 100;    // v8.13: range stored in the map header
static bool mgHdrFromBackup = false;   // v8.16: activation used the backup header
// v8.13: one clamped view of the configured range, used for the map
// geometry, the header stamp and the boot-time comparison alike.
static int mgRangeKm() {
  int r = cfgRangeKm;
  if (r < 10) r = 10;
  if (r > 500) r = 500;
  return r;
}
static bool mgActivate() {
  if (!genPart) { snprintf(mgActFail, sizeof(mgActFail), "no partition"); return false; }
  if (!genMapPtr) {
    if (esp_partition_mmap(genPart, 0, genPart->size, SPI_FLASH_MMAP_DATA,
                           (const void**)&genMapPtr, &genMmapH) != ESP_OK)
      { snprintf(mgActFail, sizeof(mgActFail), "mmap failed"); return false; }
  }
  // v8.16: the header is read with esp_partition_read (not through
  // the mmap), and a backup copy in the header page takes over if
  // the primary is unreadable.
  static GenHeader hdrS;
  mgHdrFromBackup = false;
  genColorActive = false;
  genColorPtr = nullptr;
  bool hdrOk = esp_partition_read(genPart, 0, &hdrS, sizeof(hdrS)) == ESP_OK
               && hdrS.magic == GEN_MAGIC && (hdrS.ver == 1 || hdrS.ver == 2);
  if (!hdrOk) {
    GenHeader bak;
    if (esp_partition_read(genPart, GEN_HDR_BYTES - sizeof(bak), &bak, sizeof(bak)) == ESP_OK
        && bak.magic == GEN_MAGIC && (bak.ver == 1 || bak.ver == 2)) {
      hdrS = bak;
      mgHdrFromBackup = true;
      hdrOk = true;
      esp_partition_write(genPart, 0, &hdrS, sizeof(hdrS));   // self-heal (works when erased)
      Serial.println("MAPGEN: primary header bad - using backup copy");
    }
  }
  const GenHeader* h = &hdrS;
  if (h->magic != GEN_MAGIC || (h->ver != 1 && h->ver != 2)) {
    snprintf(mgActFail, sizeof(mgActFail), "bad magic/ver %08x/%d",
             (unsigned)h->magic, (int)h->ver);
    Serial.printf("MAPGEN activate: %s\n", mgActFail);
    { uint8_t d0[16], dB[16];
      esp_partition_read(genPart, 0, d0, sizeof(d0));
      esp_partition_read(genPart, GEN_HDR_BYTES - sizeof(GenHeader), dB, sizeof(dB));
      Serial.printf("MAPGEN hdr0 %02x %02x %02x %02x %02x %02x %02x %02x  hdrB %02x %02x %02x %02x %02x %02x %02x %02x\n",
                    d0[0], d0[1], d0[2], d0[3], d0[4], d0[5], d0[6], d0[7],
                    dB[0], dB[1], dB[2], dB[3], dB[4], dB[5], dB[6], dB[7]); }
    return false;
  }
  if (h->ver == 2) {   // v124: stored colour image (GSI pale)
    const uint8_t* img = genMapPtr + GEN_HDR_BYTES;
    uint32_t s2 = 0;
    for (int k = 0; k < 614400; k++) s2 += img[k];
    if (s2 != h->sum[0]) {
      snprintf(mgActFail, sizeof(mgActFail),
               "colour image sum %u vs hdr %u", (unsigned)s2, (unsigned)h->sum[0]);
      Serial.printf("MAPGEN activate: %s\n", mgActFail);
      return false;
    }
    mgActFail[0] = 0;
    genColorPtr = img;
    genColorActive = true;
    genSeaValid = false;
    mgHdrRangeKm = h->reserved[0] ? (int)h->reserved[0] : 100;
    for (int i = 0; i < 2; i++) {
      MAP_LIST[i].north = h->b[0][0];
      MAP_LIST[i].south = h->b[0][1];
      MAP_LIST[i].west = h->b[0][2];
      MAP_LIST[i].east = h->b[0][3];
    }
    genMapsActive = true;
    Serial.println("MAPGEN: colour map active (GSI pale)");
    return true;
  }
  // v8.20: the header centre is NOT compared with the settings any
  // more - changing the centre in the portal no longer triggers a
  // remake. The stored map (with its own bounds from the header) is
  // used as-is until a remake is ordered (BtnC boot hold / portal).
  for (int i = 0; i < 2; i++) {
    const uint8_t* base = genMapPtr + GEN_HDR_BYTES + i * GEN_MAP_BYTES;
    uint32_t s = 0;
    for (int k = 0; k < GEN_MAP_BYTES; k++) s += base[k];
    if (s != h->sum[i]) {
      snprintf(mgActFail, sizeof(mgActFail),
               "map %d sum flash %u vs hdr %u", i, (unsigned)s, (unsigned)h->sum[i]);
      Serial.printf("MAPGEN activate: %s\n", mgActFail);
      return false;
    }
  }
  mgActFail[0] = 0;
  // v123: stored sea plane (old headers carry no magic -> the
  // renderer keeps using the geometric flood fill for those maps).
  genSeaValid = false;
  genSeaPtr[0] = genSeaPtr[1] = nullptr;
  if (h->reserved[1] == GEN_SEA_MAGIC) {
    const uint8_t* sp = genMapPtr + GEN_SEA_OFF;
    uint32_t s2 = 0;
    for (int k = 0; k < GEN_SEA_BYTES; k++) s2 += sp[k];
    if (s2 == h->reserved[2]) {
      genSeaValid = true;
      genSeaPtr[0] = genMapPtr + GEN_SEA_OFF;
      genSeaPtr[1] = genMapPtr + GEN_SEA_OFF + GEN_SEA_BYTES;
      Serial.println("MAPGEN: stored sea plane OK (GSI pale)");
    } else {
      Serial.printf("MAPGEN: sea plane sum mismatch %u vs %u - ignored\n",
                    (unsigned)s2, (unsigned)h->reserved[2]);
    }
  }
  mgHdrRangeKm = h->reserved[0] ? (int)h->reserved[0] : 100;   // v8.13
  for (int i = 0; i < 2; i++) {
    mapGrayPtr[i] = genMapPtr + GEN_HDR_BYTES + i * GEN_MAP_BYTES;
    mapClassPtr[i] = mapGrayPtr[i] + 153600;
    MAP_LIST[i].north = h->b[i][0];
    MAP_LIST[i].south = h->b[i][1];
    MAP_LIST[i].west = h->b[i][2];
    MAP_LIST[i].east = h->b[i][3];
  }
  genMapsActive = true;
  return true;
}

static void handleMapGen();   // portal handler, defined in the portal section

// v5.19: ALL generation buffers are allocated in setup() BEFORE
// Wi-Fi starts (pristine heap). Allocating them inside mgGenerate(),
// after the TLS warm-up, failed deterministically in v5.16-v5.18:
// the warm-up handshake succeeded (http 0), but a 10 KB aggregate
// malloc then found no hole next to the live TLS session and the
// 44 KB decoder (post-cleanup largest block read 42,996 every time).
static bool mgAllocPng() {
  if (mgPng) return true;
  void* pb = heap_caps_calloc(1, PNGLE_T_SIZE, MALLOC_CAP_DMA);
  if (!pb) return false;
  mgPng = (pngle_t*)pb;
  pngle_reset(mgPng);
  pngle_set_draw_callback(mgPng, mgDrawCb);
  return true;
}
// v6: work buffers live only from the first decode phase on, and the
// aggregates are one 160-column strip wide (2,560 B each, not 10 KB).
static bool mgAllocWorkBuffers() {
  // v6.9: back to the buffer set v6.5 proved on-device (19,072 B).
  // The 20 KB of RAM row accumulators (v6.6-v6.8) never fit the
  // heap alongside the live TLS session + pngle, in any arrangement.
  if (!mgSeaBits) mgSeaBits = (uint8_t*)heap_caps_malloc(8192, MALLOC_CAP_DMA);
  if (!mgWork) mgWork = (uint8_t*)heap_caps_malloc(6800, MALLOC_CAP_DMA);   // v123: 5760 + sea50 ring
  if (!mgAggGray) mgAggGray = (uint8_t*)heap_caps_malloc(160 * 32, MALLOC_CAP_DMA);
  if (!mgAggSea) mgAggSea = (uint8_t*)heap_caps_malloc(160 * 32, MALLOC_CAP_DMA);
  return mgAggGray && mgAggSea && mgSeaBits && mgWork;
}
// v7.1: the work buffers live only inside Phase B of each strip.
// Freeing them before every Phase A keeps the heap in the proven
// "pngle only" state for TLS handshakes - with the 32-row
// aggregates resident (24 KB), a retry handshake after a dropped
// connection failed with http -1 (v7.0, tiles 14).
static bool mgAllocColourBuffers() {   // v124
  if (!mgColR) mgColR = (uint16_t*)heap_caps_malloc(160 * 32 * 2, MALLOC_CAP_DMA);
  if (!mgColG) mgColG = (uint16_t*)heap_caps_malloc(160 * 32 * 2, MALLOC_CAP_DMA);
  if (!mgColB) mgColB = (uint16_t*)heap_caps_malloc(160 * 32 * 2, MALLOC_CAP_DMA);
  if (!mgColN) mgColN = (uint8_t*)heap_caps_malloc(160 * 32, MALLOC_CAP_DMA);
  return mgColR && mgColG && mgColB && mgColN;
}
static void mgFreeWorkBuffers() {
  if (mgAggGray) { free(mgAggGray); mgAggGray = nullptr; }
  if (mgAggSea) { free(mgAggSea); mgAggSea = nullptr; }
  if (mgColR) { free(mgColR); mgColR = nullptr; }
  if (mgColG) { free(mgColG); mgColG = nullptr; }
  if (mgColB) { free(mgColB); mgColB = nullptr; }
  if (mgColN) { free(mgColN); mgColN = nullptr; }
  if (mgSeaBits) { free(mgSeaBits); mgSeaBits = nullptr; }
  if (mgWork) { free(mgWork); mgWork = nullptr; }
}

static void mgFreeGenBuffers() {
  if (mgPng) { free(mgPng); mgPng = nullptr; }
  if (mgAggGray) { free(mgAggGray); mgAggGray = nullptr; }
  if (mgAggSea) { free(mgAggSea); mgAggSea = nullptr; }
  if (mgSeaBits) { free(mgSeaBits); mgSeaBits = nullptr; }
  if (mgWork) { free(mgWork); mgWork = nullptr; }
}

extern "C" unsigned long mz_crc32(unsigned long crc,
                                  const unsigned char* ptr, size_t buf_len);
static void mgDumpFailed() {
  if (mgDumpRec < 0 || !genPart) return;
  int rec = mgDumpRec;
  uint32_t off = GEN_STAGE_OFF + mgRecOff[rec], len = mgRecLen[rec];
  uint32_t crc = 0;
  {
    uint32_t o = off, left = len;
    while (left > 0) {
      uint32_t n = left > 2048 ? 2048 : left;
      if (esp_partition_read(genPart, o, mgTmpBuf, n) != ESP_OK) break;
      crc = mz_crc32(crc, mgTmpBuf, n);
      o += n; left -= n;
    }
  }
  Serial.printf("DUMP rec%d %s tx%d ty%d len%u crc %08x\n",
                rec, mgRecLayer[rec] ? "blank" : "pale",
                mgRecTx[rec], mgRecTy[rec], (unsigned)len, (unsigned)crc);
  static const char B64[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  uint32_t dn = len < 4096 ? len : 4096;
  Serial.println("DUMP-DATA-BEGIN");
  for (uint32_t i = 0; i < dn; i += 3) {
    uint8_t b[3] = {0, 0, 0};
    uint32_t got = 0;
    for (uint32_t k = 0; k < 3 && i + k < dn; k++) {
      if (esp_partition_read(genPart, off + i + k, &b[k], 1) == ESP_OK) got++;
    }
    char out[5];
    out[0] = B64[b[0] >> 2];
    out[1] = B64[((b[0] & 3) << 4) | (b[1] >> 4)];
    out[2] = got > 1 ? B64[((b[1] & 15) << 2) | (b[2] >> 6)] : '=';
    out[3] = got > 2 ? B64[b[2] & 63] : '=';
    out[4] = 0;
    Serial.print(out);
    if (((i / 3) + 1) % 19 == 0) Serial.println();
  }
  Serial.println();
  Serial.println("DUMP-DATA-END");
  Serial.flush();
}

static void mgDiagMaybeShow() {
  GenHeader hp = {}, hb = {};
  bool pOk = genPart && esp_partition_read(genPart, 0, &hp, sizeof(hp)) == ESP_OK
             && hp.magic == GEN_MAGIC && (hp.ver == 1 || hp.ver == 2);   // v126: ver2 colour headers are healthy
  bool bOk = genPart && esp_partition_read(genPart, GEN_HDR_BYTES - sizeof(hb), &hb, sizeof(hb)) == ESP_OK
             && hb.magic == GEN_MAGIC && (hb.ver == 1 || hb.ver == 2);   // v126
  char lgf[64] = "", laf[64] = "";
  { Preferences p;
    if (p.begin("aprsmap", true)) {
      String s1 = p.getString("lgfail", ""), s2 = p.getString("lafail", "");
      strncpy(lgf, s1.c_str(), sizeof(lgf) - 1);
      strncpy(laf, s2.c_str(), sizeof(laf) - 1);
      p.end();
    } }
  if (pOk && !lgf[0] && !laf[0]) return;   // healthy: no screen
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  int y = 10;
  char b[80];
  auto line = [&](const char* s) { tft.setCursor(8, y); tft.print(s); y += 15; };
  line("MAP DIAG (v8.17)");
  snprintf(b, sizeof(b), "SET c %.4f,%.4f r %d", homeLat, homeLon, cfgRangeKm); line(b);
  if (pOk) { snprintf(b, sizeof(b), "HDR ok c %.4f,%.4f r %d", hp.homeLat, hp.homeLon,
                      hp.reserved[0] ? (int)hp.reserved[0] : 100); line(b); }
  else { snprintf(b, sizeof(b), "HDR BAD %08x/%d", (unsigned)hp.magic, (int)hp.ver); line(b); }
  line(bOk ? "BAK ok" : "BAK bad");
  line(WiFi.status() == WL_CONNECTED ? "WIFI ok" : "WIFI no");
  if (lgf[0]) { snprintf(b, sizeof(b), "GENFAIL %s", lgf); line(b); }
  if (laf[0]) { snprintf(b, sizeof(b), "ACTFAIL %s", laf); line(b); }
  line("photo this screen, please");
  delay(12000);
  if (pOk) {   // shown once after recovery: clear the stale records
    Preferences p;
    if (p.begin("aprsmap", false)) { p.putString("lgfail", ""); p.putString("lafail", ""); p.end(); }
  }
}

static void __attribute__((noinline)) mapGenBoot() {
  genPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                     ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "mapdata");
  if (!genPart) {
    Serial.println("MAPGEN: no mapdata partition - baked maps only");
    return;
  }
  if (mgCkpt) {   // v8.6: a hard reset interrupted the last run
    static char cl[64];
    snprintf(cl, sizeof(cl), "reset %d at map %d strip %d phase %d",
             (int)esp_reset_reason(), (int)(mgCkpt >> 24),
             (int)((mgCkpt >> 8) & 0xFFFF), (int)(mgCkpt & 0xFF));
    Serial.printf("MAPGEN interrupted: %s\n", cl);
    mgShow("Reset during generation", cl, 0);
    delay(9000);
    mgCkpt = 0;
  }
  mgDiagMaybeShow();   // v8.17
  bool force = false;
  {
    Preferences p;
    if (p.begin("aprsmap", false)) {
      force = p.getBool("mgforce", false);
      if (force) p.putBool("mgforce", false);
      p.end();
    }
  }
  if (!force && mgActivate()) {
    // v8.18: a stored map is accepted as-is. The v8.13 range
    // condition (reject and regenerate when the header's range
    // differs from the setting) is REMOVED - in the field it sent
    // the device into an endless regeneration loop.
    Serial.println("MAPGEN: generated maps active");
    if (mgHdrFromBackup) { mgShow("Map header restored", "backup copy used", 0); delay(3000); }
    return;
  }
  if (!force && mgActFail[0]) {   // v8.2: verdict on the LCD too
    mgShow("Stored map rejected", mgActFail, 0);
    delay(6000);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("MAPGEN: offline - baked maps for now");
    return;
  }
  Serial.println("MAPGEN: generating maps for the configured centre...");
  stopConfigServer();        // v5.16: free every heap byte for the
  endMdns();                 // TLS handshake (see mgGenerate)
  delay(100);
  bool genOk = mgGenerate();
  mgCkpt = 0;   // v8.6: only a hard reset leaves it set
  mgHttp.end();                 // v6.5: session's job is done
  mgTls.stop();
  if (!genOk) mgDumpFailed();   // v6.4: serial dump of the bad record
  mgFreeGenBuffers();   // hand the heap back to normal operation
  // S3 v24: the LAN portal is NOT restarted here - it stays off
  // during normal operation until BOOT is held for 2 s.
  bool actOk = genOk && mgActivate();
  { Preferences p;   // v8.17: persist the outcome for the diag screen
    if (p.begin("aprsmap", false)) {
      p.putString("lgfail", genOk ? "" : String(mgFail));
      p.putString("lafail", (genOk && !actOk) ? String(mgActFail) : String(""));
      p.end();
    } }
  if (actOk) {
    Serial.println("MAPGEN: generation complete");
    mgShow("Map generation complete", "", 100);
    delay(1200);
  } else {
    Serial.println("MAPGEN: generation FAILED - baked maps");
    { static char fl[48], fl2[64];
      if (!mgFail[0]) mgFail = "activate check";
      snprintf(fl, sizeof(fl), "FAILED: %s", mgFail);
      if (mgFailDetail[0]) {
        snprintf(fl2, sizeof(fl2), "%s", mgFailDetail);
      } else {
        snprintf(fl2, sizeof(fl2), "heap %u http %d tiles %d [button]",
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
                 mgLastHttp, mgTilesOk);
      }
      mgShow(fl, fl2, 0);
      // v5.10: keep the diagnosis on screen until the screen is
      // touched (024 port: a touch press instead of the M5 buttons)
      uint32_t ft0 = millis();
      while (millis() - ft0 < 120000) {
        lgfx::touch_point_t tp;
        if (tft.getTouchRaw(&tp, 1)) break;   // S3: any touch sample
        delay(50);
      } }
    delay(2000);
  }
}


struct BmpInfo { bool ok = false; int w = 0, h = 0; uint32_t dataOff = 0; int rowSize = 0; bool topDown = false; };
static BmpInfo bmpInfo[MAP_COUNT];

static void computeView() {
  const MapDef& m = MAP_LIST[curMap];
  const BmpInfo& bi = bmpInfo[curMap];
  int w = bi.ok ? bi.w : 320;
  int h = bi.ok ? bi.h : 240;
  // v116: ONE zoom table for all screens - the map was unified in
  // v103, and the user expects the display (zoom steps and scale
  // labels) to be identical on APRS / FLIGHT / AIS too. Before, the
  // Flight fit view labelled itself "x0.5" while AIS called the
  // same view "x1", and the levels were remembered per screen.
  // v117: TWO zoom steps only (user request): x0.5 (whole map)
  // and x1.0. The 7-step cycle never landed him on x0.5 in
  // practice; a plain toggle between the two views is clearer.
  // v171N: FOUR steps again (user request): x0.5 / x1.0 / x1.5 /
  // x2.0, driven by the [-] [+] buttons (the v117 two-step ZOOM
  // toggle is retired).
  // S3 v9 (= 028 v186N): SIX steps (user request): x2.5 and
  // x3.0 added.
  static const double APRS_SCALES[6] = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
  int li = zoomLevel; if (li < 0) li = 0; if (li > 5) li = 5;
  double s = APRS_SCALES[li];
  vScale = s;
  vSrcW = 320.0 / s; if (vSrcW > w) vSrcW = w;
  vSrcH = 240.0 / s; if (vSrcH > h) vSrcH = h;
  vXOff = (int)lround((320 - vSrcW * s) / 2);
  vYOff = (int)lround((240 - vSrcH * s) / 2);
  double cxPx = (centerLon - m.west) / (m.east - m.west) * w;
  double cyPx = (m.north - centerLat) / (m.north - m.south) * h;
  vRectX0 = cxPx - vSrcW / 2; if (vRectX0 < 0) vRectX0 = 0; if (vRectX0 > w - vSrcW) vRectX0 = w - vSrcW;
  vRectY0 = cyPx - vSrcH / 2; if (vRectY0 < 0) vRectY0 = 0; if (vRectY0 > h - vSrcH) vRectY0 = h - vSrcH;
  vRectWest  = m.west + vRectX0 / w * (m.east - m.west);
  vRectNorth = m.north - vRectY0 / h * (m.north - m.south);
  vPxPerLon = s * w / (m.east - m.west);
  vPxPerLat = s * h / (m.north - m.south);
}

static double viewLatSpan() {
  computeView();
  int h = bmpInfo[curMap].ok ? bmpInfo[curMap].h : 240;
  return vSrcH / h * (MAP_LIST[curMap].north - MAP_LIST[curMap].south);
}
static double viewLonSpan() {
  computeView();
  int w = bmpInfo[curMap].ok ? bmpInfo[curMap].w : 320;
  return vSrcW / w * (MAP_LIST[curMap].east - MAP_LIST[curMap].west);
}

static bool geoToPx(double lat, double lon, int& px, int& py) {
  computeView();
  px = vXOff + (int)lround((lon - vRectWest) * vPxPerLon);
  py = vYOff + (int)lround((vRectNorth - lat) * vPxPerLat);
  return px >= -20 && px < 340 && py >= -20 && py < 260;
}

// Startup/reset zoom: the whole map fitted to the screen (level 0), so
// the whole map is what you see first. Zoom in with [+] / station taps.
static int maxZoomLevel() {
  return 5;   // S3 v9 (= 028 v186N): six steps (x0.5 .. x3.0)
}
static int defaultZoomLevel() {
  return 0;   // v171N: every screen opens at x0.5 (user request)
}

static uint16_t rowPixBuf[320];

// ---- flicker-free composition ---------------------------------------
// A full-frame 16-bit sprite needs 153,600 B, more than this board's
// largest heap block (~110 KB), so it could never be allocated and G
// is the panel itself: every draw below lands on the screen as it
// happens. Flicker is therefore limited by HOW the panel is fed
// (v76): the map goes out in 60-row blocks and the top bar is only
// repainted where its text actually changed. (v77 reverted the v76
// repair batching after it triggered reboots on station taps; v78
// re-batches repairs in a DMA-safe way and skips the redraw of
// unchanged markers - see restoreMapRect and drawStations.)
static LGFX_Sprite spr(&tft);
static LovyanGFX* G = &tft;
static bool sprOk = false;
static char snapAcFlight[MAX_AC][12] = {};
static int vImgRows = 0, vImgCols = 0;
static int tapX = -1, tapY = -1;       // last registered tap (marker)
static uint32_t tapMs = 0;
static bool tapShown = false;
// v112: a press captured while a full map render is running (the
// render blocks loop() for ~500 ms and presses that started and
// ended inside it were lost - "touch takes a long time to respond").
static bool pendingTap = false;
static int pendingTapX = 0, pendingTapY = 0;
static void captureTouchDuringRender();   // defined by touchLoop

// (mapChunk is declared with the aircraft globals - fetchAircraft frees it)
static int chunkY0 = 0, chunkRows = 0;
// S3 v5: two-buffer compose state. When both frame halves exist,
// emitLine only STAGES lines (no pushes during the ~1 s compose);
// drawMap flushes both halves back-to-back at the end, so the panel
// swaps the whole frame within ~30 ms instead of sweeping.
static bool emitTwo = false;
static uint16_t* emitTarget = nullptr;
static void emitLine(int y) {
  if (emitTwo) {
    memcpy(emitTarget + (size_t)chunkRows * 320, rowPixBuf, 320 * 2);
    chunkRows++;
    if (y == 239) {
      G->pushImage(0, 0, 320, mapSplitRow, mapChunk);
      G->pushImage(0, mapSplitRow, 320, 240 - mapSplitRow, mapChunk2);
      chunkRows = 0;
    } else if (y == mapSplitRow - 1) {
      emitTarget = mapChunk2;
      chunkRows = 0;
    }
    return;
  }
  if (!mapChunk) { G->pushImage(0, y, 320, 1, rowPixBuf); return; }
  if (chunkRows == 0) chunkY0 = y;
  memcpy(mapChunk + (size_t)chunkRows * 320, rowPixBuf, 320 * 2);
  chunkRows++;
  if (chunkRows == mapChunkRowsCap || y == 239) {
    G->pushImage(0, chunkY0, 320, chunkRows, mapChunk);
    chunkRows = 0;
  }
}


// Maps are embedded in flash as 4-bit grayscale (maps_data.h) - no SD
// card is used. Downscaling averages the source footprint and upscaling
// uses bilinear interpolation, so lines stay smooth and anti-aliased.
static inline int mapGrayAt(int sx, int sy) {
  const uint8_t* m = mapGrayPtr[curMap];
  uint8_t b = m[sy * MAP_ROW_BYTES + (sx >> 1)];
  return ((sx & 1) ? (b & 0x0F) : (b >> 4)) * 17;   // 0 = line .. 255 = paper
}

// Line class of a source pixel: true = coastline (drawn white),
// false = border (drawn green). Baked into maps_data.h from the GSI
// standard tiles' sea mask.
static inline bool mapCoastAt(int sx, int sy) {
  int i = sy * MAP_W + sx;
  return (mapClassPtr[curMap][i >> 3] >> (7 - (i & 7))) & 1;
}

// v87 map style (user request): no line art; the sea is a flat
// pale-blue fill and the land stays black. The stored maps carry
// NO sea tone to key on: generated maps are classified from GSI
// "blank" tiles, whose sea is paper-white exactly like the land
// (measured on-device by the map diag, 2026-11-17: active GEN map0
// is 99% paper in class 0), and the sea bits gathered during
// generation are discarded after the coast classification. So the
// sea is recovered geometrically: a scanline flood fill from the
// map edges over the paper, walled by everything that is not
// paper (g < 240: line cores, anti-aliasing and the coast-class
// shoulder - the same barrier rule that was verified offline on
// the baked blank-map data: 65% / 43% sea, no leaks). The fill
// runs once per map/source change; the renderer then only tests
// the mask - pale blue inside, black outside.
static const uint16_t SEA_BLUE = (uint16_t)__builtin_bswap16(
    (uint16_t)(((80 & 0xF8) << 8) | ((115 & 0xFC) << 3) | (145 >> 3)));
// v167N: sea darkened to ~60% (135,190,235 -> 80,115,145) - the
// pale sea was what made the whole map read bright (user).
// v90 boot-stage marker (RTC memory survives a panic reboot):
// the splash shows where the previous boot died.

static uint8_t* seaMask = nullptr;          // 640x480 bits
static int seaMaskMap = -1;
static const uint8_t* seaMaskSrc = nullptr;
static int seaMaskFailMap = -1;             // v88: build-failure latch -
static const uint8_t* seaMaskFailSrc = nullptr;  // never retry per frame

// v97: inland line art (municipal / prefectural borders, rivers - any
// ink that is not coastline class) renders as line art on the black
// land. Coast-class ink stays black so the blue sea edge stays crisp.
// v101: the line colour is the pre-v83 border GREEN (0,215,0) - white
// read too faint on this panel (user report). Stored byte-swapped
// like SEA_BLUE, for the 024 pushImage path.
static const uint16_t BORDER_GREEN = 0x4005;   // v167N: (0,170,0) dimmed with the sea, byte-swapped
static inline bool seaMaskBit(int sx, int sy);   // defined just below
static inline bool seaNearAt(int sx, int sy) {
  if (!seaMask) return false;
  for (int dy = -1; dy <= 1; dy++)
    for (int dx = -1; dx <= 1; dx++)
      if (seaMaskBit(sx + dx, sy + dy)) return true;
  return false;
}
static inline bool inkLineAt(int sx, int sy) {   // unthinned line eligibility
  int g = mapGrayAt(sx, sy);
  if (g >= 240) return false;
  // v110: line ink only. The generation contrast stretch pulls the
  // blank tiles' light area tints (residential / park fills, source
  // gray 191-245) down into the 153-239 range; drawing those green
  // made the borders look massively thick (user report). True line
  // cores sit at nibble <=8 (gray <=136); tints fall through to the
  // black land ramp in mapPixelAt.
  if (g >= 153) return false;
  if (!mapCoastAt(sx, sy)) return true;
  // v108: coast-CLASS ink far from the real sea is a border / river
  // line misclassified at generation time - inland water (irrigation
  // irrigation ponds, rivers) puts 'sea' bits all over the plain, so
  // most plain borders came out coast-class and were drawn black.
  // Only ink actually touching the sea mask is a true coastline:
  // keep that black, draw the rest green. Render-side only.
  return !seaNearAt(sx, sy);
}
static inline bool inkWhiteAt(int sx, int sy) {
  if (!inkLineAt(sx, sy)) return false;
  // v121: thinning (user: municipal borders still too thick). A
  // stored line is a dark core plus the generation grown ring -
  // 3-4 px wide. Keep only its upper-left contour: a pixel with
  // line ink both above and left of it is interior and drops out.
  // Host-validated: green pixels halve, lines stay continuous at
  // x1 and at the x0.5 box downscale.
  bool up = sy > 0 && inkLineAt(sx, sy - 1);
  bool lf = sx > 0 && inkLineAt(sx - 1, sy);
  return !(up && lf);
}

static inline bool seaMaskBit(int sx, int sy) {
  if (!seaMask) return false;
  if (sx < 0) sx = 0; if (sx >= MAP_W) sx = MAP_W - 1;
  if (sy < 0) sy = 0; if (sy >= MAP_H) sy = MAP_H - 1;
  int i = sy * MAP_W + sx;
  return (seaMask[i >> 3] >> (7 - (i & 7))) & 1;
}

struct SeaDecision { int16_t x, y, minx, maxx, miny, maxy; bool sea; };
static SeaDecision seaDecisions[512];   // v120: room for interior cells too
static int seaDecisionCount = 0;

static void buildSeaMask(int mi) {
  // v98/v100: a paper region counts as sea only if it touches the map
  // edge AND (almost) all of its ink boundary is coastline class.
  // Plain edge seeding also flooded edge-touching LAND (the
  // neighbouring prefecture cut off by the map frame turned blue -
  // user report); a prefecture cell is bounded largely by border ink
  // and fails the >=90% coast test, while a true sea cell's ink
  // boundary is coastline by construction. Host-validated on the
  // baked maps (sea cells 94-100% coast, land cells <=69%).
  // v100 memory layout: the DMA-capable heap fits only TWO 38,400 B
  // bitplanes at build time (v98's third one failed on device), so
  // phase 1 decides the edge cells into a scratch plane and records
  // the verdicts, then phase 2 replays the accepted cells into the
  // mask. The fill stack is word-accessed only, so it may sit in any
  // heap region (plain INTERNAL caps). Byte-accessed planes use
  // MALLOC_CAP_DMA so they can never land in the byte-load-faulting
  // leftover-IRAM region (the v87-v95 boot panic).
  Serial.printf("sea mask build start map %d (heap %u)\n", mi, (unsigned)ESP.getFreeHeap());
  uint8_t* blocked = (uint8_t*)heap_caps_malloc(38400, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  if (!blocked) { seaMaskFailMap = mi; seaMaskFailSrc = mapGrayPtr[mi]; Serial.println("sea mask: blocked alloc failed"); return; }
  memset(blocked, 0, 38400);
  uint8_t* visited = (uint8_t*)heap_caps_malloc(38400, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  if (!visited) { free(blocked); seaMaskFailMap = mi; seaMaskFailSrc = mapGrayPtr[mi]; Serial.println("sea mask: visited alloc failed"); return; }
  memset(visited, 0, 38400);
  int32_t* seaStack = (int32_t*)heap_caps_malloc(4096 * 4, MALLOC_CAP_INTERNAL);
  if (!seaStack) { free(visited); free(blocked); seaMaskFailMap = mi; seaMaskFailSrc = mapGrayPtr[mi]; Serial.println("sea mask: stack alloc failed"); return; }
  int saved = curMap; curMap = mi;
  for (int y = 0; y < MAP_H; y++) {
    if ((y & 63) == 0) esp_task_wdt_reset();
    for (int x = 0; x < MAP_W; x++) {
      if (mapGrayAt(x, y) < 240) {
        int i = y * MAP_W + x;
        blocked[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
      }
    }
  }
  auto bitAt = [](const uint8_t* p, int x, int y) -> bool {
    int i = y * MAP_W + x;
    return (p[i >> 3] >> (7 - (i & 7))) & 1;
  };
  auto blk = [&](int x, int y) -> bool {   // barrier ink
    if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) return true;
    return bitAt(blocked, x, y);
  };
  uint8_t* fillTarget = visited;
  bool doTally = false;
  long cellN = 0, tallyInk = 0, tallyCoast = 0;
  int cellMinX = 0, cellMaxX = -1, cellMinY = 0, cellMaxY = -1;
  auto setCell = [&](int x, int y) {
    int i = y * MAP_W + x;
    fillTarget[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
    if (x < cellMinX) cellMinX = x;
    if (x > cellMaxX) cellMaxX = x;
    if (y < cellMinY) cellMinY = y;
    if (y > cellMaxY) cellMaxY = y;
    if (!doTally) return;
    cellN++;
    static const int dx4[4] = { 1, -1, 0, 0 }, dy4[4] = { 0, 0, 1, -1 };
    for (int d = 0; d < 4; d++) {
      int nx = x + dx4[d], ny = y + dy4[d];
      if (nx < 0 || nx >= MAP_W || ny < 0 || ny >= MAP_H) continue;
      if (mapGrayAt(nx, ny) < 240) { tallyInk++; if (mapCoastAt(nx, ny)) tallyCoast++; }
    }
  };
  auto filled = [&](int x, int y) -> bool {
    return bitAt(fillTarget, x, y) || blk(x, y);
  };
  // Stepping horizontally/vertically across a corner where the two
  // diagonal neighbours are both barrier would leak through an
  // 8-connected ink chain; the squeeze tests close those corners
  // without dilating the barrier (narrow straits stay open).
  auto squeezeH = [&](int x, int y, int dx) -> bool {
    return (blk(x + dx, y - 1) && blk(x, y + 1)) || (blk(x + dx, y + 1) && blk(x, y - 1));
  };
  auto squeezeV = [&](int x, int y, int ny) -> bool {
    return (blk(x - 1, ny) && blk(x + 1, y)) || (blk(x + 1, ny) && blk(x - 1, y));
  };
  int sp = 0;
  auto push = [&](int sx, int sy) { if (sp < 4096) seaStack[sp++] = (int32_t)((sy << 10) | sx); };
  auto fillFromStack = [&]() {
    while (sp > 0) {
      int32_t v = seaStack[--sp];
      int x = v & 1023, y = v >> 10;
      if (filled(x, y)) continue;
      int lx = x;
      while (lx > 0 && !filled(lx - 1, y) && !squeezeH(lx, y, -1)) lx--;
      int rx = x;
      while (rx < MAP_W - 1 && !filled(rx + 1, y) && !squeezeH(rx, y, +1)) rx++;
      for (int i = lx; i <= rx; i++) setCell(i, y);
      for (int dy = -1; dy <= 1; dy += 2) {
        int ny = y + dy;
        if (ny < 0 || ny >= MAP_H) continue;
        int i = lx;
        while (i <= rx) {
          while (i <= rx && (filled(i, ny) || squeezeV(i, y, ny))) i++;
          if (i > rx) break;
          push(i, ny);
          while (i <= rx && !filled(i, ny)) i++;
        }
      }
    }
  };
  auto fillCell = [&](int sx, int sy, bool tally) {
    doTally = tally;
    cellN = 0; tallyInk = 0; tallyCoast = 0;
    cellMinX = MAP_W; cellMaxX = -1; cellMinY = MAP_H; cellMaxY = -1;
    sp = 0;
    push(sx, sy);
    fillFromStack();
  };
  // Phase 1: decide every edge-touching paper cell.
  seaDecisionCount = 0;
  auto decide = [&](int sx, int sy) {
    fillCell(sx, sy, true);
    if (cellN == 0) return;
    // v108: 80% (was 90%). v111: 72% - the user's own serial log of
    // his generated 50 km map (2026-11-17) showed his stored data
    // fragments the northern sea into small cells scoring 74.7-78.8%
    // (island chains cut the water up; on-device classification runs
    // coarser than the host replication, which saw one 89% cell),
    // while every true-land edge cell scored <=68.2%. 72% sits in
    // the measured gap; baked-map land cells (<=69%) stay land too.
    bool sea = (tallyInk == 0) || (tallyCoast * 25 >= tallyInk * 18);
    if (cellN >= 3 && seaDecisionCount < 512) {
      seaDecisions[seaDecisionCount++] = SeaDecision{
        (int16_t)sx, (int16_t)sy, (int16_t)cellMinX, (int16_t)cellMaxX,
        (int16_t)cellMinY, (int16_t)cellMaxY, sea };
      Serial.printf("sea mask: cell (%d,%d) %ld px, ink %ld coast %ld -> %s\n",
                    sx, sy, cellN, tallyInk, tallyCoast, sea ? "SEA" : "land");
    }
  };
  for (int x = 0; x < MAP_W; x++) {
    if (!blk(x, 0) && !bitAt(visited, x, 0)) decide(x, 0);
    if (!blk(x, MAP_H - 1) && !bitAt(visited, x, MAP_H - 1)) decide(x, MAP_H - 1);
  }
  for (int y = 0; y < MAP_H; y++) {
    if (!blk(0, y) && !bitAt(visited, 0, y)) decide(0, y);
    if (!blk(MAP_W - 1, y) && !bitAt(visited, MAP_W - 1, y)) decide(MAP_W - 1, y);
  }
  free(visited);
  // Phase 2: replay the accepted cells into the real mask.
  if (!seaMask) seaMask = (uint8_t*)heap_caps_malloc(38400, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  if (!seaMask) { free(seaStack); free(blocked); curMap = saved; seaMaskFailMap = mi; seaMaskFailSrc = mapGrayPtr[mi]; Serial.println("sea mask: mask alloc failed"); return; }
  memset(seaMask, 0, 38400);
  fillTarget = seaMask;
  for (int d = 0; d < seaDecisionCount; d++)
    if (seaDecisions[d].sea) fillCell(seaDecisions[d].x, seaDecisions[d].y, false);
  // v122 phase 2.5: interior pockets EMBEDDED in confirmed sea.
  // v120 decided interior cells by tally+size alone and flipped
  // inland ponds / valley pockets to sea (user photo: blue blobs
  // all over the plain). A true island pocket sits within a few
  // px of confirmed sea (only island ink between); an inland pond
  // has no sea anywhere near its bbox. Test: any seaMask pixel
  // within the cell bbox expanded by 5 px.
  {
    uint8_t* done = (uint8_t*)heap_caps_malloc(38400, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!done) {
      Serial.println("sea mask: pocket pass skipped (alloc failed)");
    } else {
      memset(done, 0, 38400);
      for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++)
          if (blk(x, y) || seaMaskBit(x, y)) {
            int i = y * MAP_W + x;
            done[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
          }
      fillTarget = done;
      for (int y = 0; y < MAP_H; y++) {
        if ((y & 63) == 0) esp_task_wdt_reset();
        for (int x = 0; x < MAP_W; x++) {
          if (blk(x, y) || bitAt(done, x, y)) continue;
          fillCell(x, y, true);
          if (cellN < 3 || cellN > 700) continue;
          bool verdict = (tallyInk == 0) || (tallyCoast * 25 >= tallyInk * 18);
          if (!verdict) continue;
          int bx0 = cellMinX > 5 ? cellMinX - 5 : 0, bx1 = cellMaxX < MAP_W - 6 ? cellMaxX + 5 : MAP_W - 1;
          int by0 = cellMinY > 5 ? cellMinY - 5 : 0, by1 = cellMaxY < MAP_H - 6 ? cellMaxY + 5 : MAP_H - 1;
          bool embedded = false;
          for (int yy = by0; yy <= by1 && !embedded; yy++)
            for (int xx = bx0; xx <= bx1; xx++)
              if (seaMaskBit(xx, yy)) { embedded = true; break; }
          Serial.printf("sea mask: pocket (%d,%d) %ld px -> %s\n",
                        x, y, cellN, embedded ? "SEA" : "inland, stays land");
          if (embedded) {
            fillTarget = seaMask;
            fillCell(x, y, false);
            fillTarget = done;
          }
        }
      }
      free(done);
      fillTarget = seaMask;
    }
  }
  // Repair: pockets a stack overflow left inside an accepted cell
  // (bbox-tested so rejected land cells can never leak in).
  auto inBBox = [&](int x, int y, bool sea) -> bool {
    for (int d = 0; d < seaDecisionCount; d++) {
      if (seaDecisions[d].sea != sea) continue;
      if (x >= seaDecisions[d].minx && x <= seaDecisions[d].maxx &&
          y >= seaDecisions[d].miny && y <= seaDecisions[d].maxy) return true;
    }
    return false;
  };
  for (int round = 0; round < 3; round++) {
    bool any = false;
    for (int y = 0; y < MAP_H; y++) {
      if ((y & 63) == 0) esp_task_wdt_reset();
      for (int x = 0; x < MAP_W; x++) {
        if (blk(x, y) || bitAt(seaMask, x, y)) continue;
        if (!((x > 0 && bitAt(seaMask, x - 1, y)) || (x < MAP_W - 1 && bitAt(seaMask, x + 1, y)) ||
              (y > 0 && bitAt(seaMask, x, y - 1)) || (y < MAP_H - 1 && bitAt(seaMask, x, y + 1)))) continue;
        if (!inBBox(x, y, true) || inBBox(x, y, false)) continue;
        fillCell(x, y, false);
        any = true;
      }
    }
    if (!any) break;
  }
  free(seaStack);
  free(blocked);
  curMap = saved;
  seaMaskMap = mi;
  seaMaskSrc = mapGrayPtr[mi];
  uint32_t nSea = 0;
  for (int i = 0; i < 38400; i++) nSea += __builtin_popcount(seaMask[i]);
  Serial.printf("sea mask map %d: %u px (%u%%)\n", mi, (unsigned)nSea, (unsigned)(nSea * 100 / 307200));
}

static void ensureSeaMask() {
  if (genColorActive) return;   // v124: the stored image IS the map
  if (seaMaskMap == curMap && seaMaskSrc == mapGrayPtr[curMap]) return;
  // v112: when both map slots hold identical data (the unified
  // range map is copied into slot 1 at generation time), reuse the
  // other slot's mask instead of rebuilding for seconds - tab
  // switches stay instant.
  if (seaMask && seaMaskMap >= 0 && seaMaskMap != curMap &&
      seaMaskSrc == mapGrayPtr[seaMaskMap] &&
      !memcmp(mapGrayPtr[0], mapGrayPtr[1], 153600) &&
      !memcmp(mapClassPtr[0], mapClassPtr[1], 38400)) {
    seaMaskMap = curMap;
    seaMaskSrc = mapGrayPtr[curMap];
    Serial.println("sea mask: copied from twin slot");
    return;
  }
  if (seaMaskFailMap == curMap && seaMaskFailSrc == mapGrayPtr[curMap]) return;
  // v123: a generated map with a stored sea plane needs no flood
  // fill at all - the mask is copied straight from flash.
  if (genMapsActive && genSeaValid && genSeaPtr[curMap]) {
    if (!seaMask)
      seaMask = (uint8_t*)heap_caps_malloc(38400, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (seaMask) {
      memcpy(seaMask, genSeaPtr[curMap], 38400);
      seaMaskMap = curMap;
      seaMaskSrc = mapGrayPtr[curMap];
      Serial.println("sea mask: stored GSI sea plane");
      return;
    }
  }
  buildSeaMask(curMap);
}

// v124: stored colour image helpers. Stored pixels are
// byte-swapped RGB565 (the form pushImage displays correctly).
static inline void colorUnpack(uint16_t stored, int& r, int& g, int& b) {
  uint16_t v = (uint16_t)((stored << 8) | (stored >> 8));
  r = ((v >> 11) & 31) << 3;
  g = ((v >> 5) & 63) << 2;
  b = (v & 31) << 3;
}
static inline uint16_t colorPack(int r, int g, int b) {
  uint16_t v = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
  return (uint16_t)((v << 8) | (v >> 8));
}
static inline uint16_t colorPxStored(int sx, int sy) {
  uint16_t v;
  memcpy(&v, genColorPtr + ((sy * 640) + sx) * 2, 2);
  return v;
}

// v128: colour sampling in IMAGE coordinates. v124 put this
// logic only in mapPixelAt - but the full-frame renderer drawMap
// has its own inline sea/ink pixel logic and never calls
// mapPixelAt (only restoreMapRect does), so the colour map
// activated yet never appeared. Both paths now share this.
// v168N: the unit is in COLOUR mode (stored ver=2 GSI pale map) -
// the pale tiles are white-based, which is what reads bright.
// Dim the sampled colour at render time; stored data untouched.
static const double MAP_DIM = 0.65;
// v169N: deepen the sea. GSI pale water is blue-dominant
// (b > g > r with a wide b-r gap, ~181,209,237); land whites and
// tints are near-neutral or warm/green, so the test is safe.
// Water pixels take per-channel factors (deeper, more saturated)
// instead of the flat MAP_DIM.
static inline bool isSeaColor(int r, int g, int b) {
  return b > g + 8 && g > r + 8 && b - r > 30;
}
static uint16_t colorAt(int ix, int iy) {
    if (vScale > 1.0) {
      double srcYf = vRectY0 + iy / vScale;
      int sy0 = (int)srcYf; if (sy0 < 0) sy0 = 0; if (sy0 >= MAP_H) sy0 = MAP_H - 1;
      int sy1 = min(sy0 + 1, MAP_H - 1);
      double fy = srcYf - floor(srcYf); if (fy < 0) fy = 0;
      double srcXf = vRectX0 + ix / vScale;
      int sx0 = (int)srcXf; if (sx0 < 0) sx0 = 0; if (sx0 >= MAP_W) sx0 = MAP_W - 1;
      int sx1 = min(sx0 + 1, MAP_W - 1);
      double fx = srcXf - floor(srcXf); if (fx < 0) fx = 0;
      int r00, g00, b00, r10, g10, b10, r01, g01, b01, r11, g11, b11;
      colorUnpack(colorPxStored(sx0, sy0), r00, g00, b00);
      colorUnpack(colorPxStored(sx1, sy0), r10, g10, b10);
      colorUnpack(colorPxStored(sx0, sy1), r01, g01, b01);
      colorUnpack(colorPxStored(sx1, sy1), r11, g11, b11);
      double rr = r00 * (1 - fx) * (1 - fy) + r10 * fx * (1 - fy) + r01 * (1 - fx) * fy + r11 * fx * fy;
      double gg = g00 * (1 - fx) * (1 - fy) + g10 * fx * (1 - fy) + g01 * (1 - fx) * fy + g11 * fx * fy;
      double bb = b00 * (1 - fx) * (1 - fy) + b10 * fx * (1 - fy) + b01 * (1 - fx) * fy + b11 * fx * fy;
      if (isSeaColor((int)rr, (int)gg, (int)bb))
        return colorPack((int)(rr * 0.42 + 0.5), (int)(gg * 0.68 + 0.5), (int)(bb * 0.85 + 0.5));
      return colorPack((int)(rr * MAP_DIM + 0.5), (int)(gg * MAP_DIM + 0.5), (int)(bb * MAP_DIM + 0.5));
    }
    int ys = (int)(vRectY0 + iy / vScale); if (ys < 0) ys = 0; if (ys >= MAP_H) ys = MAP_H - 1;
    int ye = (int)(vRectY0 + (iy + 1) / vScale); if (ye <= ys) ye = ys + 1; if (ye > MAP_H) ye = MAP_H;
    int xs = (int)(vRectX0 + ix / vScale); if (xs < 0) xs = 0; if (xs >= MAP_W) xs = MAP_W - 1;
    int xe = (int)(vRectX0 + (ix + 1) / vScale); if (xe <= xs) xe = xs + 1; if (xe > MAP_W) xe = MAP_W;
    long rs = 0, gs = 0, bs = 0; int cnt = 0;
    for (int sy = ys; sy < ye; sy++)
      for (int sx = xs; sx < xe; sx++) {
        int r, g, b;
        colorUnpack(colorPxStored(sx, sy), r, g, b);
        rs += r; gs += g; bs += b; cnt++;
      }
    if (!cnt) return 0;
    {
      int ar = (int)(rs / cnt), ag = (int)(gs / cnt), ab = (int)(bs / cnt);
      if (isSeaColor(ar, ag, ab))
        return colorPack((int)(ar * 0.42), (int)(ag * 0.68), (int)(ab * 0.85));
      return colorPack((int)(ar * MAP_DIM), (int)(ag * MAP_DIM), (int)(ab * MAP_DIM));
    }
}

// Colour of one screen pixel of the bare map (no stations, no bar).
static uint16_t mapPixelAt(int x, int y) {
  int iy = y - vYOff;
  if (iy < 0 || iy >= vImgRows) return 0;
  int ix = x - vXOff;
  if (ix < 0 || ix >= vImgCols) return 0;
  if (genColorActive) return colorAt(ix, iy);   // v128: shared sampler
  if (vScale > 1.0) {
    double srcYf = vRectY0 + iy / vScale;
    int sy0 = (int)srcYf; if (sy0 < 0) sy0 = 0; if (sy0 >= MAP_H) sy0 = MAP_H - 1;
    int sy1 = min(sy0 + 1, MAP_H - 1);
    double fy = srcYf - floor(srcYf); if (fy < 0) fy = 0;
    double srcXf = vRectX0 + ix / vScale;
    int sx0 = (int)srcXf; if (sx0 < 0) sx0 = 0; if (sx0 >= MAP_W) sx0 = MAP_W - 1;
    int sx1 = min(sx0 + 1, MAP_W - 1);
    double fx = srcXf - floor(srcXf); if (fx < 0) fx = 0;
    double s00 = seaMaskBit(sx0, sy0) ? 1 : 0, s10 = seaMaskBit(sx1, sy0) ? 1 : 0;
    double s01 = seaMaskBit(sx0, sy1) ? 1 : 0, s11 = seaMaskBit(sx1, sy1) ? 1 : 0;
    double sf = s00 * (1 - fx) * (1 - fy) + s10 * fx * (1 - fy)
              + s01 * (1 - fx) * fy + s11 * fx * fy;
    double w00 = inkWhiteAt(sx0, sy0) ? 1 : 0, w10 = inkWhiteAt(sx1, sy0) ? 1 : 0;
    double w01 = inkWhiteAt(sx0, sy1) ? 1 : 0, w11 = inkWhiteAt(sx1, sy1) ? 1 : 0;
    double wf = w00 * (1 - fx) * (1 - fy) + w10 * fx * (1 - fy)
              + w01 * (1 - fx) * fy + w11 * fx * fy;
    return sf >= 0.5 ? SEA_BLUE : (wf >= 0.5 ? BORDER_GREEN : 0);
  }
  int ys = (int)(vRectY0 + iy / vScale); if (ys < 0) ys = 0; if (ys >= MAP_H) ys = MAP_H - 1;
  int ye = (int)(vRectY0 + (iy + 1) / vScale); if (ye <= ys) ye = ys + 1; if (ye > MAP_H) ye = MAP_H;
  int xs = (int)(vRectX0 + ix / vScale); if (xs < 0) xs = 0; if (xs >= MAP_W) xs = MAP_W - 1;
  int xe = (int)(vRectX0 + (ix + 1) / vScale); if (xe <= xs) xe = xs + 1; if (xe > MAP_W) xe = MAP_W;
  int cnt = 0, seaCnt = 0, whiteCnt = 0;
  for (int sy = ys; sy < ye; sy++)
    for (int sx = xs; sx < xe; sx++) {
      cnt++;
      if (seaMaskBit(sx, sy)) seaCnt++;
      else if (inkWhiteAt(sx, sy)) whiteCnt++;
    }
  return (cnt && seaCnt * 2 >= cnt) ? SEA_BLUE
       : (whiteCnt && whiteCnt * 2 >= cnt) ? BORDER_GREEN : 0;   // v110: >=50% (was >=25% - speckle densified at x0.5)
}

static void drawMap() {
  // v137: while a TLS handshake owns the heap (acFetchBusy), do NOT
  // re-allocate the chunk - emitLine has a line-by-line fallback for
  // a null chunk, so the frame still renders, just slower, and the
  // handshake keeps both the escrow hole and the chunk's block.
  if (!mapChunk && !acFetchBusy) {
    // S3 v2: full frame first (see mapChunkRowsCap note above).
    static const int caps[3] = { 240, 120, MAP_CHUNK_ROWS };
    for (int ci = 0; ci < 3 && !mapChunk; ci++) {
      mapChunk = (uint16_t*)heap_caps_malloc(320 * caps[ci] * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
      if (mapChunk) {
        mapChunkRowsCap = caps[ci];
        Serial.printf("render chunk: %d rows (%u B), largest DMA %u\n", caps[ci],
                      (unsigned)(320 * caps[ci] * sizeof(uint16_t)),
                      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
      }
    }
    if (!mapChunk)
      Serial.printf("render chunk: NONE (largest DMA %u) - line-by-line\n",
                    (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
  }
  ensureSeaMask();
  computeView();
  vImgRows = (int)lround(vSrcH * vScale);
  vImgCols = (int)lround(vSrcW * vScale);
  bool smooth = (vScale > 1.0);
  // S3 v5: per-column source ranges for the fast colour path
  // (colorAt recomputed these with 4 software double divisions for
  // EVERY pixel - a large slice of the ~1,090 ms render).
  static int colSrcX0[320], colSrcX1[320];
  bool colorFast = (genColorActive && vScale <= 1.0);
  if (colorFast) {
    for (int ix = 0; ix < 320; ix++) {
      int xs = (int)(vRectX0 + ix / vScale); if (xs < 0) xs = 0; if (xs >= MAP_W) xs = MAP_W - 1;
      int xe = (int)(vRectX0 + (ix + 1) / vScale); if (xe <= xs) xe = xs + 1; if (xe > MAP_W) xe = MAP_W;
      colSrcX0[ix] = xs; colSrcX1[ix] = xe;
    }
  }
  // S3 v5: two-buffer compose arms here (both halves permanent).
  emitTwo = (mapChunk && mapChunk2 && mapSplitRow > 0 && mapSplitRow < 240);
  emitTarget = mapChunk;
  uint32_t t0 = millis();
  // v112: no frame-long startWrite transaction. Each chunk push is
  // a standalone pushImage (the v78 restoreMapRect pattern, which
  // returns only after its transfer completes), so the bus is free
  // between pushes and the touch panel can be polled mid-render.
  chunkRows = 0;
  for (int y = 0; y < 240; y++) {
    if (y && (y & 31) == 0) captureTouchDuringRender();
    int iy = y - vYOff;
    if (iy < 0 || iy >= vImgRows) {                 // letterbox margin
      for (int x = 0; x < 320; x++) rowPixBuf[x] = 0;
      emitLine(y);
      continue;
    }
    if (smooth) {
      if (genColorActive) {   // v128: stored colour image
        for (int x = 0; x < 320; x++) {
          int ix = x - vXOff;
          rowPixBuf[x] = (ix < 0 || ix >= vImgCols) ? 0 : colorAt(ix, iy);
        }
        emitLine(y);
        continue;
      }
      double srcYf = vRectY0 + iy / vScale;
      int sy0 = (int)srcYf; if (sy0 < 0) sy0 = 0; if (sy0 >= MAP_H) sy0 = MAP_H - 1;
      int sy1 = min(sy0 + 1, MAP_H - 1);
      double fy = srcYf - floor(srcYf); if (fy < 0) fy = 0;
      for (int x = 0; x < 320; x++) {
        int ix = x - vXOff;
        if (ix < 0 || ix >= vImgCols) { rowPixBuf[x] = 0; continue; }
        double srcXf = vRectX0 + ix / vScale;
        int sx0 = (int)srcXf; if (sx0 < 0) sx0 = 0; if (sx0 >= MAP_W) sx0 = MAP_W - 1;
        int sx1 = min(sx0 + 1, MAP_W - 1);
        double fx = srcXf - floor(srcXf); if (fx < 0) fx = 0;
        double s00 = seaMaskBit(sx0, sy0) ? 1 : 0, s10 = seaMaskBit(sx1, sy0) ? 1 : 0;
        double s01 = seaMaskBit(sx0, sy1) ? 1 : 0, s11 = seaMaskBit(sx1, sy1) ? 1 : 0;
        double sf = s00 * (1 - fx) * (1 - fy) + s10 * fx * (1 - fy)
                  + s01 * (1 - fx) * fy + s11 * fx * fy;
        double w00 = inkWhiteAt(sx0, sy0) ? 1 : 0, w10 = inkWhiteAt(sx1, sy0) ? 1 : 0;
        double w01 = inkWhiteAt(sx0, sy1) ? 1 : 0, w11 = inkWhiteAt(sx1, sy1) ? 1 : 0;
        double wf = w00 * (1 - fx) * (1 - fy) + w10 * fx * (1 - fy)
                  + w01 * (1 - fx) * fy + w11 * fx * fy;
        rowPixBuf[x] = sf >= 0.5 ? SEA_BLUE : (wf >= 0.5 ? BORDER_GREEN : 0);
      }
      emitLine(y);
      continue;
    }
    if (genColorActive) {   // v128: stored colour image
      // S3 v5 fast path: same footprint averaging as colorAt's
      // vScale<=1 branch, but integer-only with hoisted ranges and
      // direct row reads (was ~1,090 ms per render on-device).
      int ys = (int)(vRectY0 + iy / vScale); if (ys < 0) ys = 0; if (ys >= MAP_H) ys = MAP_H - 1;
      int ye = (int)(vRectY0 + (iy + 1) / vScale); if (ye <= ys) ye = ys + 1; if (ye > MAP_H) ye = MAP_H;
      for (int x = 0; x < 320; x++) {
        int ix = x - vXOff;
        if (ix < 0 || ix >= vImgCols) { rowPixBuf[x] = 0; continue; }
        int xs = colSrcX0[ix], xe = colSrcX1[ix];
        long rs = 0, gs = 0, bs = 0; int cnt = 0;
        for (int sy = ys; sy < ye; sy++) {
          const uint16_t* srow = (const uint16_t*)(genColorPtr + ((size_t)sy * 640 + xs) * 2);
          for (int sx = xs; sx < xe; sx++) {
            uint16_t v = srow[sx - xs];
            uint16_t u = (uint16_t)((v << 8) | (v >> 8));   // stored byte-swapped
            rs += ((u >> 11) & 31) << 3;
            gs += ((u >> 5) & 63) << 2;
            bs += (u & 31) << 3;
            cnt++;
          }
        }
        if (!cnt) { rowPixBuf[x] = 0; continue; }
        int ar = (int)(rs / cnt), ag = (int)(gs / cnt), ab = (int)(bs / cnt);
        rowPixBuf[x] = isSeaColor(ar, ag, ab)
            ? colorPack((ar * 42) / 100, (ag * 68) / 100, (ab * 85) / 100)
            : colorPack((ar * 65) / 100, (ag * 65) / 100, (ab * 65) / 100);
      }
      emitLine(y);
      continue;
    }
    int ys = (int)(vRectY0 + iy / vScale); if (ys < 0) ys = 0; if (ys >= MAP_H) ys = MAP_H - 1;
    int ye = (int)(vRectY0 + (iy + 1) / vScale); if (ye <= ys) ye = ys + 1; if (ye > MAP_H) ye = MAP_H;
    for (int x = 0; x < 320; x++) {
      int ix = x - vXOff;
      if (ix < 0 || ix >= vImgCols) { rowPixBuf[x] = 0; continue; }
      int xs = (int)(vRectX0 + ix / vScale); if (xs < 0) xs = 0; if (xs >= MAP_W) xs = MAP_W - 1;
      int xe = (int)(vRectX0 + (ix + 1) / vScale); if (xe <= xs) xe = xs + 1; if (xe > MAP_W) xe = MAP_W;
      int cnt = 0, seaCnt = 0, whiteCnt = 0;
      for (int sy = ys; sy < ye; sy++)
        for (int sx = xs; sx < xe; sx++) {
          cnt++;
          if (seaMaskBit(sx, sy)) seaCnt++;
          else if (inkWhiteAt(sx, sy)) whiteCnt++;
        }
      rowPixBuf[x] = (cnt && seaCnt * 2 >= cnt) ? SEA_BLUE
                   : (whiteCnt && whiteCnt * 2 >= cnt) ? BORDER_GREEN : 0;   // v110: >=50% (was >=25% - speckle densified at x0.5)
    }
    emitLine(y);
  }
  Serial.printf("map render: %lu ms (flash)\n", (unsigned long)(millis() - t0));
}

// ---------------- drawing ----------------
static const int BAR_H = 22;

static const char* aprsStateText();  // fwd

static char lastBarLeft[32] = "", lastBarMid[48] = "";
// v175N: bar layout - visible tabs pack from the left (a hidden
// screen leaves no tab at all), LIST follows the last tab, the
// status text follows LIST. drawBarF recomputes this every call;
// the touch handler reads the same globals.
static int barTabX[3] = {2, 48, 94};
static const int barTabW[3] = {44, 44, 30};
static int barListX = 126;
static int barStatusX = 168;
static void layoutBar() {
  int x = 2;
  static const int ORDER[3] = {1, 2, 0};   // S3 v22: FLIGHT, AIS, APRS (user request)
  for (int k = 0; k < 3; k++) {
    int i = ORDER[k];
    if ((cfgScreens >> i) & 1) { barTabX[i] = x; x += barTabW[i] + 2; }
    else barTabX[i] = -1;
  }
  // S3 v7 (user correction of v6): the tabs stay packed on the
  // left as before; it is the LIST button that moves beside the
  // [-] [+] buttons at the top-right - [-] is drawn at x=268, so
  // LIST (38 wide) ends 4 px before it. The status text fills
  // the space between the last tab and LIST.
  barListX = 268 - 4 - 38;
  barStatusX = x + 2;
}
static void drawBarF(bool force, bool push) {
  layoutBar();
  computeView();
  char scale[8], mid[48], leftKey[4];
  if (fabs(vScale - lround(vScale)) < 0.01) snprintf(scale, sizeof(scale), "x%d", (int)lround(vScale));
  else snprintf(scale, sizeof(scale), "x%.1f", vScale);
  if (screenMode == 0) snprintf(mid, sizeof(mid), "%s %s", aprsStateText(), scale);
  else if (screenMode == 1) snprintf(mid, sizeof(mid), "AC %d %s", acCount, scale);
  else snprintf(mid, sizeof(mid), "SHIP %d %s", countShips(), scale);
  snprintf(leftKey, sizeof(leftKey), "%c", screenMode == 0 ? 'A' : (screenMode == 1 ? 'F' : 'S'));
  // No visible change -> leave the screen alone (user request: update
  // only when something inside the map area changed).
  if (!force && !strcmp(leftKey, lastBarLeft) && !strcmp(mid, lastBarMid)) return;
  if (!force && !strcmp(leftKey, lastBarLeft)) {
    // v76: same screen, only the status text changed (station count /
    // scale live inside mid) - repaint just that strip instead of
    // blanking and redrawing the whole bar (a visible blink on every
    // 5 s tick and every station update).
    strcpy(lastBarMid, mid);
    int sRight = barListX - 2;   // S3 v7: stop at the LIST button (LIST and [-] [+] live right of it)
    G->fillRect(barStatusX - 2, 0, sRight - (barStatusX - 2), BAR_H, TFT_NAVY);   // status strip only
    G->setFont(&fonts::Font0);
    G->setTextColor(TFT_WHITE, TFT_NAVY);
    G->setCursor(barStatusX, 7);
    G->print(mid);
    if (push && sprOk) spr.pushSprite(0, 0);
    return;
  }
  strcpy(lastBarLeft, leftKey);
  strcpy(lastBarMid, mid);
  G->fillRect(0, 0, 320, BAR_H, TFT_NAVY);
  // screen tabs: the visible screens only (v175N), packed from
  // the left - the active one is filled white
  G->setFont(&fonts::Font0);
  static const char* TABLB[MODE_COUNT] = {"APRS", "FLIGHT", "AIS"};
  for (int i = 0; i < MODE_COUNT; i++) {
    if (barTabX[i] < 0) continue;   // hidden screen: no tab
    int tx = barTabX[i], tw = barTabW[i];
    const char* lb = TABLB[i];
    if (screenMode == i) {
      G->fillRect(tx, 2, tw, 18, TFT_WHITE);
      G->setTextColor(TFT_NAVY, TFT_WHITE);
    } else {
      G->drawRect(tx, 2, tw, 18, TFT_WHITE);
      G->setTextColor(TFT_WHITE, TFT_NAVY);
    }
    G->setCursor(tx + (tw - G->textWidth(lb)) / 2, 7);
    G->print(lb);
  }
  // v171N: [-] [+] are back at the TOP-RIGHT corner (user
  // request, reversing the v119 move-left). The status text sits
  // between the tabs and LIST (S3 v7 layout). The corner's touch reads
  // offset, so the hit boxes (in the touch handler) are wider
  // than the drawn boxes.
  G->setTextColor(TFT_WHITE, TFT_NAVY);
  G->setCursor(barStatusX, 7);
  G->print(mid);
  G->drawRect(barListX, 2, 38, 18, TFT_WHITE);
  G->setCursor(barListX + 6, 7); G->print("LIST");
  G->drawRect(268, 2, 24, 18, TFT_WHITE);
  G->setCursor(277, 7); G->print("-");
  G->drawRect(294, 2, 24, 18, TFT_WHITE);
  G->setCursor(303, 7); G->print("+");
  if (push && sprOk) spr.pushSprite(0, 0);
}
static void drawBar() { drawBarF(true, true); }

// Snapshot of what drawStations() last put on screen, so the loop can
// skip redraws when no visible station actually moved or appeared.
static int16_t snapPx[MAX_STATIONS], snapPy[MAX_STATIONS];
static int16_t snapLabelW[MAX_STATIONS];   // label width as drawn (ghost fix)
static bool snapUsed[MAX_STATIONS] = {};
static int snapSelected = -2;
static char snapCall[MAX_STATIONS][20] = {};   // callsign as drawn (v78)
// v184N: the trail as drawn (erase box + change detection)
static uint8_t snapTrailCntD[MAX_STATIONS] = {};   // in-window points drawn (0 = no trail)
static uint32_t snapTrailOldest[MAX_STATIONS] = {};
static uint32_t snapTrailNewest[MAX_STATIONS] = {};
static bool snapMoving[MAX_STATIONS] = {};         // drawn as a moving (red triangle) station
static int16_t snapTrailBox[MAX_STATIONS][4] = {}; // drawn trail bbox x0,y0,x1,y1

static bool trailSigChanged(int i) {
  int c; uint32_t o, n; float p;
  trailStats(i, millis(), c, o, n, p);
  bool moving = c >= 2 && p >= 100.0f;
  return snapMoving[i] != moving ||
         snapTrailCntD[i] != (moving ? c : 0) ||
         snapTrailOldest[i] != o || snapTrailNewest[i] != n;
}
// v78: set by redraw() each frame - true only when nothing was erased
// or repainted under the markers, in which case the draw* functions
// may skip objects whose on-screen image would not change at all.
static bool skipUnchanged = false;
static bool repairErased = false;   // repairMovedObjects() restored a rect

static void drawHomeMarker() {
  int px, py;
  // home marker: red cross with a black underlay
  if (geoToPx(homeLat, homeLon, px, py) && py >= BAR_H) {
    G->drawLine(px - 6, py, px + 6, py, TFT_BLACK);
    G->drawLine(px, py - 6, px, py + 6, TFT_BLACK);
    G->drawLine(px - 5, py, px + 5, py, TFT_RED);
    G->drawLine(px, py - 5, px, py + 5, TFT_RED);
  }
}

static void drawStations() {
  int px, py;
  uint32_t nowMs = millis();
  for (int i = 0; i < MAX_STATIONS; i++) {
    if (!stations[i].used) { snapUsed[i] = false; snapTrailCntD[i] = 0; snapMoving[i] = false; continue; }
    if (!geoToPx(stations[i].lat, stations[i].lon, px, py)) { snapUsed[i] = false; snapTrailCntD[i] = 0; snapMoving[i] = false; continue; }
    if (py < BAR_H + 4) { snapUsed[i] = false; snapTrailCntD[i] = 0; snapMoving[i] = false; continue; }
    if (px < 0 || px >= 320) { snapUsed[i] = false; snapTrailCntD[i] = 0; snapMoving[i] = false; continue; }   // label would float marker-less
    bool sel = (i == selected);
    // v184N: a station with a live track (>= 2 points, >= 100 m of
    // travel inside the last 10 min) is a moving station: red
    // AIS-style triangle + red trail instead of the orange circle.
    int tCnt; uint32_t tOld, tNew; float tPath;
    trailStats(i, nowMs, tCnt, tOld, tNew, tPath);
    bool moving = tCnt >= 2 && tPath >= 100.0f;
    // v78: this exact marker (position, callsign, selection state)
    // is already on screen and nothing was erased this frame - leave
    // it alone instead of redrawing every label on every update.
    // v184N: the trail must also be exactly as drawn.
    if (skipUnchanged && snapUsed[i] && px == snapPx[i] && py == snapPy[i] &&
        sel == (snapSelected == i) && !strcmp(snapCall[i], stations[i].call) &&
        snapMoving[i] == moving && snapTrailCntD[i] == (moving ? tCnt : 0) &&
        snapTrailOldest[i] == tOld && snapTrailNewest[i] == tNew)
      continue;
    snapUsed[i] = true; snapPx[i] = px; snapPy[i] = py;
    snapLabelW[i] = (int16_t)((int)strlen(stations[i].call) * 6 + 16);
    strncpy(snapCall[i], stations[i].call, sizeof(snapCall[i]) - 1);
    snapCall[i][sizeof(snapCall[i]) - 1] = 0;
    snapMoving[i] = moving;
    snapTrailCntD[i] = moving ? (uint8_t)tCnt : 0;
    snapTrailOldest[i] = tOld; snapTrailNewest[i] = tNew;
    uint16_t dot = sel ? TFT_YELLOW : (moving ? TFT_RED : TFT_ORANGE);
    if (moving) {
      // the last 10 minutes of track as a red polyline under the marker
      int bx0 = 32000, by0 = 32000, bx1 = -32000, by1 = -32000;
      int pvx = 0, pvy = 0; bool havePrev = false;
      Trail* tr = trailOf(i, false);
      for (int k = 0; tr && k < tr->cnt; k++) {
        if (nowMs - tr->pts[k].ms > TRAIL_WINDOW_MS) continue;
        int qx, qy;
        if (!geoToPx(tr->pts[k].lat, tr->pts[k].lon, qx, qy)) { havePrev = false; continue; }
        if (havePrev) G->drawLine(pvx, pvy, qx, qy, TFT_RED);
        pvx = qx; pvy = qy; havePrev = true;
        if (qx < bx0) bx0 = qx; if (qx > bx1) bx1 = qx;
        if (qy < by0) by0 = qy; if (qy > by1) by1 = qy;
      }
      if (havePrev && (pvx != px || pvy != py)) {
        G->drawLine(pvx, pvy, px, py, TFT_RED);
        if (px < bx0) bx0 = px; if (px > bx1) bx1 = px;
        if (py < by0) by0 = py; if (py > by1) by1 = py;
      }
      if (bx0 > bx1) { bx0 = px - 2; bx1 = px + 2; by0 = py - 2; by1 = py + 2; }
      snapTrailBox[i][0] = (int16_t)bx0; snapTrailBox[i][1] = (int16_t)by0;
      snapTrailBox[i][2] = (int16_t)bx1; snapTrailBox[i][3] = (int16_t)by1;
      // AIS-style triangle pointing along the course (same geometry
      // as the underway ships)
      int brg = trailBearing(i, nowMs);
      if (brg < 0) brg = stations[i].course >= 0 ? stations[i].course : 0;
      float r = brg * (float)DEG_TO_RAD;
      float sn = sinf(r), cs = cosf(r);
      int nx = px + (int)(9 * sn),  ny = py - (int)(9 * cs);
      int tx = px - (int)(6 * sn),  ty = py + (int)(6 * cs);
      int lx = tx + (int)(5 * cs),  ly = ty + (int)(5 * sn);
      int rx2 = tx - (int)(5 * cs), ry2 = ty - (int)(5 * sn);
      G->fillTriangle(nx, ny, lx, ly, rx2, ry2, dot);
      G->fillCircle(px, py, 1, TFT_WHITE);
    } else {
      G->fillCircle(px, py, sel ? 6 : 5, dot);
      G->drawCircle(px, py, sel ? 6 : 5, TFT_BLACK);
      G->fillCircle(px, py, 1, TFT_WHITE);
    }
    // v183N: setTextWrap(false) - with LGFX's default wrap ON, a label
    // crossing the RIGHT edge wrapped its tail to the LEFT edge one
    // line down (the user's "left-edge garbage": e.g. JA5MSG-10 at
    // px289 showed its tail "SG-10" at x=0). Clip, never wrap.
    G->setTextWrap(false);
    // label with a full black halo so it reads on any map colour
    G->setFont(&fonts::Font0);
    G->setTextColor(TFT_BLACK);
    G->setCursor(px + 6, py - 5); G->print(stations[i].call);
    G->setCursor(px + 8, py - 5); G->print(stations[i].call);
    G->setCursor(px + 7, py - 6); G->print(stations[i].call);
    G->setCursor(px + 7, py - 4); G->print(stations[i].call);
    G->setCursor(px + 7, py - 5);
    G->setTextColor(dot);
    G->print(stations[i].call);
  }
  snapSelected = selected;
}

// Aircraft snapshot, same idea as the station snapshot.
static int16_t snapAcPx[MAX_AC], snapAcPy[MAX_AC];
static int snapAcCount = -1;
static float snapAcTrack[MAX_AC];              // heading as drawn (v78)
static char snapSelAcFlight[12] = "";          // selection as drawn (v78)
static bool snapDetailIsAc = false;

// ---------------- aircraft trails (S3 v20, user request) ----------
// A short history polyline behind each aircraft on the FLIGHT
// screen, drawn length capped at ~56 px (about 1 cm on this
// panel). One point per fetch, appended when the aircraft has
// moved >= 250 m from the last stored point; 6 points = up to
// ~3 min of history at the 30 s poll. Keyed by callsign (the
// aircraft table is rebuilt every fetch, so indices are not
// stable); slots are LRU-evicted. Same snapshot/repair pattern
// as the APRS station trails (v184N).
#define AC_TRAIL_SLOTS 24
#define AC_TRAIL_PTS 6
struct AcTrail {
  char flight[12] = "";
  uint8_t cnt = 0;
  float lat[AC_TRAIL_PTS];
  float lon[AC_TRAIL_PTS];
  uint32_t lastMs = 0;
};
static AcTrail acTrails[AC_TRAIL_SLOTS];
static int16_t snapAcTrailBox[MAX_AC][4];   // drawn trail bbox
static int snapAcTrailCntD[MAX_AC];         // stored points as drawn
static float snapAcTrailLat[MAX_AC], snapAcTrailLon[MAX_AC];   // newest stored point as drawn

static AcTrail* acTrailOf(const char* flight, bool create) {
  AcTrail* lru = nullptr; uint32_t lruMs = 0xFFFFFFFF;
  for (int k = 0; k < AC_TRAIL_SLOTS; k++) {
    if (acTrails[k].flight[0] && !strcmp(acTrails[k].flight, flight)) return &acTrails[k];
    if (create) {
      if (!acTrails[k].flight[0]) {
        strncpy(acTrails[k].flight, flight, sizeof(acTrails[k].flight) - 1);
        acTrails[k].flight[sizeof(acTrails[k].flight) - 1] = 0;
        acTrails[k].cnt = 0;
        return &acTrails[k];
      }
      if (acTrails[k].lastMs < lruMs) { lruMs = acTrails[k].lastMs; lru = &acTrails[k]; }
    }
  }
  if (create && lru) {
    strncpy(lru->flight, flight, sizeof(lru->flight) - 1);
    lru->flight[sizeof(lru->flight) - 1] = 0;
    lru->cnt = 0;
    return lru;
  }
  return nullptr;
}

static bool acTrailStale(int i) {
  if (!aircraft[i].flight[0]) return snapAcTrailCntD[i] > 0;
  AcTrail* t = acTrailOf(aircraft[i].flight, false);
  int cnt = t ? t->cnt : 0;
  if (cnt != snapAcTrailCntD[i]) return true;
  if (cnt > 0 && (t->lat[cnt - 1] != snapAcTrailLat[i] || t->lon[cnt - 1] != snapAcTrailLon[i])) return true;
  return false;
}

// ---------------- stationary aircraft drop (S3 v21, user request)
// Aircraft that do not move at all (parked, or a stale feed) sit
// on the map forever because every fetch returns them again.
// Track each callsign's position across fetches: within 50 m
// for 3 consecutive fetches (~90 s at the 30 s poll) and the
// aircraft is removed from the table (marker, trail, LIST and
// detail all follow the table). It reappears as soon as a fetch
// shows it somewhere else.
#define AC_STILL_SLOTS 24
struct AcStill {
  char flight[12] = "";
  float lat = 0, lon = 0;
  uint8_t still = 0;
  uint32_t lastMs = 0;
};
static AcStill acStill[AC_STILL_SLOTS];

static void acDropStationary() {
  uint32_t now = millis();
  bool drop[MAX_AC] = {};
  for (int i = 0; i < acCount; i++) {
    if (!aircraft[i].flight[0]) continue;
    AcStill* s = nullptr; AcStill* lru = nullptr; uint32_t lruMs = 0xFFFFFFFF;
    for (int k = 0; k < AC_STILL_SLOTS; k++) {
      if (acStill[k].flight[0] && !strcmp(acStill[k].flight, aircraft[i].flight)) { s = &acStill[k]; break; }
      if (!acStill[k].flight[0]) { if (!lru) lru = &acStill[k]; }
      else if (acStill[k].lastMs < lruMs) { lruMs = acStill[k].lastMs; lru = &acStill[k]; }
    }
    if (!s) {
      s = lru; if (!s) continue;
      strncpy(s->flight, aircraft[i].flight, sizeof(s->flight) - 1);
      s->flight[sizeof(s->flight) - 1] = 0;
      s->lat = (float)aircraft[i].lat; s->lon = (float)aircraft[i].lon;
      s->still = 0; s->lastMs = now;
      continue;
    }
    double dLat = (aircraft[i].lat - s->lat) * 111320.0;
    double dLon = (aircraft[i].lon - s->lon) * 111320.0 * cos(aircraft[i].lat * DEG_TO_RAD);
    if (dLat * dLat + dLon * dLon >= 50.0 * 50.0) {
      s->lat = (float)aircraft[i].lat; s->lon = (float)aircraft[i].lon;
      s->still = 0;
    } else if (s->still < 255) s->still++;
    s->lastMs = now;
    if (s->still >= 3) drop[i] = true;
  }
  int w = 0;
  for (int i = 0; i < acCount; i++) if (!drop[i]) { if (w != i) aircraft[w] = aircraft[i]; w++; }
  if (w != acCount) {
    Serial.printf("ADS-B: dropped %d stationary aircraft\n", acCount - w);
    acCount = w;
  }
}

static void acTrailsUpdate() {
  uint32_t now = millis();
  for (int i = 0; i < acCount; i++) {
    if (!aircraft[i].flight[0]) continue;
    AcTrail* t = acTrailOf(aircraft[i].flight, true);
    if (!t) continue;
    if (t->cnt > 0 && now - t->lastMs > 180000UL) t->cnt = 0;   // long gap: start fresh
    if (t->cnt > 0) {
      double dLat = (aircraft[i].lat - t->lat[t->cnt - 1]) * 111320.0;
      double dLon = (aircraft[i].lon - t->lon[t->cnt - 1]) * 111320.0 * cos(aircraft[i].lat * DEG_TO_RAD);
      if (dLat * dLat + dLon * dLon < 250.0 * 250.0) { t->lastMs = now; continue; }
    }
    if (t->cnt >= AC_TRAIL_PTS) {
      memmove(&t->lat[0], &t->lat[1], sizeof(float) * (AC_TRAIL_PTS - 1));
      memmove(&t->lon[0], &t->lon[1], sizeof(float) * (AC_TRAIL_PTS - 1));
      t->cnt = AC_TRAIL_PTS - 1;
    }
    t->lat[t->cnt] = (float)aircraft[i].lat;
    t->lon[t->cnt] = (float)aircraft[i].lon;
    t->cnt++;
    t->lastMs = now;
  }
}

static void drawAircraft() {
  int px, py;
  for (int i = 0; i < acCount; i++) {
    if (!geoToPx(aircraft[i].lat, aircraft[i].lon, px, py)) { snapAcPx[i] = -1; continue; }
    if (py < BAR_H + 4) { snapAcPx[i] = -1; continue; }
    if (px < 0 || px >= 320) { snapAcPx[i] = -1; continue; }   // same label-spill fix
    bool sel = detailIsAc && !strcmp(aircraft[i].flight, selAcFlight);
    bool selBefore = snapDetailIsAc && !strcmp(aircraft[i].flight, snapSelAcFlight);
    // v78: skip-unchanged (position, flight, TRACK, selection) - the
    // triangle rotates with the track, so track must match too.
    if (skipUnchanged && snapAcPx[i] == px && snapAcPy[i] == py &&
        !strcmp(snapAcFlight[i], aircraft[i].flight) &&
        snapAcTrack[i] == aircraft[i].track && sel == selBefore &&
        !acTrailStale(i))   // S3 v20: trail grew -> redraw
      continue;
    snapAcPx[i] = px; snapAcPy[i] = py;
    strncpy(snapAcFlight[i], aircraft[i].flight, sizeof(snapAcFlight[i]) - 1);
    snapAcFlight[i][sizeof(snapAcFlight[i]) - 1] = 0;
    snapAcTrack[i] = aircraft[i].track;
    uint16_t col = sel ? TFT_YELLOW : TFT_GREEN;   // S3 v13: bright green (user trial; v10-v12 white)
    // S3 v20: history trail behind the aircraft, drawn length
    // capped at ~56 px (about 1 cm); the current position is the
    // final point, stored points are walked newest -> oldest.
    {
      AcTrail* t = aircraft[i].flight[0] ? acTrailOf(aircraft[i].flight, false) : nullptr;
      int tcnt = t ? t->cnt : 0;
      snapAcTrailCntD[i] = tcnt;
      if (tcnt > 0) { snapAcTrailLat[i] = t->lat[tcnt - 1]; snapAcTrailLon[i] = t->lon[tcnt - 1]; }
      int bx0 = px, by0 = py, bx1 = px, by1 = py;
      if (tcnt > 0) {
        int cx = px, cy = py; float budget = 56.0f;
        for (int k = tcnt - 1; k >= 0 && budget > 0; k--) {
          int qx, qy;
          if (!geoToPx(t->lat[k], t->lon[k], qx, qy)) break;
          float dx = (float)(qx - cx), dy = (float)(qy - cy);
          float seg = sqrtf(dx * dx + dy * dy);
          if (seg < 1.0f) continue;
          int ex = qx, ey = qy;
          if (seg > budget) { ex = cx + (int)(dx * budget / seg); ey = cy + (int)(dy * budget / seg); }
          G->drawLine(cx, cy, ex, ey, col);
          if (ex < bx0) bx0 = ex; if (ex > bx1) bx1 = ex;
          if (ey < by0) by0 = ey; if (ey > by1) by1 = ey;
          budget -= seg; cx = qx; cy = qy;
        }
      }
      snapAcTrailBox[i][0] = bx0; snapAcTrailBox[i][1] = by0;
      snapAcTrailBox[i][2] = bx1; snapAcTrailBox[i][3] = by1;
    }
    // small triangle pointing along the track, cyan like a radar plot
    float r = aircraft[i].track * (float)DEG_TO_RAD;
    float sn = sinf(r), cs = cosf(r);
    int nx = px + (int)(9 * sn),  ny = py - (int)(9 * cs);         // nose
    int bx = px - (int)(6 * sn),  by = py + (int)(6 * cs);         // tail centre
    int lx = bx + (int)(5 * cs),  ly = by + (int)(5 * sn);
    int rx = bx - (int)(5 * cs),  ry = by - (int)(5 * sn);
    G->fillTriangle(nx, ny, lx, ly, rx, ry, col);
    G->fillCircle(px, py, 1, TFT_WHITE);
    G->setTextWrap(false);   // v183N: clip at the right edge, never wrap to the left
    G->setFont(&fonts::Font0);
    G->setTextColor(TFT_BLACK);
    G->setCursor(px + 7, py - 5); G->print(aircraft[i].flight);
    G->setCursor(px + 9, py - 5); G->print(aircraft[i].flight);
    G->setCursor(px + 8, py - 6); G->print(aircraft[i].flight);
    G->setCursor(px + 8, py - 4); G->print(aircraft[i].flight);
    G->setCursor(px + 8, py - 5);
    G->setTextColor(col);
    G->print(aircraft[i].flight);
  }
  snapAcCount = acCount;
  strncpy(snapSelAcFlight, selAcFlight, sizeof(snapSelAcFlight) - 1);
  snapSelAcFlight[sizeof(snapSelAcFlight) - 1] = 0;
  snapDetailIsAc = detailIsAc;
}

// Ship snapshot, same idea as the aircraft snapshot.
static int16_t snapShipPx[MAX_SHIPS], snapShipPy[MAX_SHIPS];
static uint32_t snapShipMmsi[MAX_SHIPS] = {};
static int snapShipCount = -1;
static float snapShipCog[MAX_SHIPS];           // course as drawn (v78)
static bool snapShipUnder[MAX_SHIPS] = {};     // underway shape as drawn (v78)
static char snapShipName[MAX_SHIPS][24] = {};  // label as drawn (v78)
static uint32_t snapSelShipMmsi = 0;           // selection as drawn (v78)

static void drawShips() {
  int px, py;
  for (int i = 0; i < MAX_SHIPS; i++) {
    if (!ships[i].used || !ships[i].hasPos) { snapShipPx[i] = -1; continue; }
    if (!geoToPx(ships[i].lat, ships[i].lon, px, py)) { snapShipPx[i] = -1; continue; }
    if (py < BAR_H + 4) { snapShipPx[i] = -1; continue; }
    if (px < 0 || px >= 320) { snapShipPx[i] = -1; continue; }   // same label-spill fix
    char label[24];
    if (ships[i].name[0]) strncpy(label, ships[i].name, sizeof(label) - 1);
    else snprintf(label, sizeof(label), "%lu", (unsigned long)ships[i].mmsi);
    label[sizeof(label) - 1] = 0;
    bool sel = ships[i].mmsi == selShipMmsi && selShipMmsi != 0;
    bool underway = ships[i].sog >= 0.5f && ships[i].cog < 360.0f;
    // v78: skip-unchanged (position, MMSI, course, shape, label,
    // selection) - anchored ships otherwise got fully redrawn on
    // every AIS update even though they never move.
    if (skipUnchanged && snapShipPx[i] == px && snapShipPy[i] == py &&
        snapShipMmsi[i] == ships[i].mmsi && snapShipCog[i] == ships[i].cog &&
        snapShipUnder[i] == underway && !strcmp(snapShipName[i], label) &&
        sel == (snapShipMmsi[i] == snapSelShipMmsi && snapSelShipMmsi != 0))
      continue;
    snapShipPx[i] = px; snapShipPy[i] = py;
    snapShipMmsi[i] = ships[i].mmsi;
    snapShipCog[i] = ships[i].cog;
    snapShipUnder[i] = underway;
    strncpy(snapShipName[i], label, sizeof(snapShipName[i]) - 1);
    snapShipName[i][sizeof(snapShipName[i]) - 1] = 0;
    uint16_t col = sel ? TFT_YELLOW : TFT_CYAN;   // S3 v10: cyan (user trial; was APRS orange)
    if (underway) {
      // triangle pointing along the course over ground
      float r = ships[i].cog * (float)DEG_TO_RAD;
      float sn = sinf(r), cs = cosf(r);
      int nx = px + (int)(9 * sn),  ny = py - (int)(9 * cs);
      int bx = px - (int)(6 * sn),  by = py + (int)(6 * cs);
      int lx = bx + (int)(5 * cs),  ly = by + (int)(5 * sn);
      int rx = bx - (int)(5 * cs),  ry = by - (int)(5 * sn);
      G->fillTriangle(nx, ny, lx, ly, rx, ry, col);
    } else {
      // anchored / moored / drifting: a small square
      G->fillRect(px - 4, py - 4, 9, 9, col);
      G->drawRect(px - 4, py - 4, 9, 9, TFT_BLACK);
    }
    G->fillCircle(px, py, 1, TFT_WHITE);
    G->setTextWrap(false);   // v183N: clip at the right edge, never wrap to the left
    G->setFont(&fonts::Font0);
    G->setTextColor(TFT_BLACK);
    G->setCursor(px + 7, py - 5); G->print(label);
    G->setCursor(px + 9, py - 5); G->print(label);
    G->setCursor(px + 8, py - 6); G->print(label);
    G->setCursor(px + 8, py - 4); G->print(label);
    G->setCursor(px + 8, py - 5);
    G->setTextColor(col);
    G->print(label);
  }
  snapShipCount = countShips();
  snapSelShipMmsi = selShipMmsi;
}

// ◀ ▶ ▲ ▼ hints at the screen edges while zoomed in (the pan tap zones)
static void drawPanHints() {
  if (zoomLevel <= 0) return;   // v117: pan hints at x1.0 (the zoomed step)
  G->fillTriangle(8, 120, 2, 112, 2, 128, TFT_BLACK);   // v174N: black (user: silver was hard to see on the pale map)
  G->fillTriangle(311, 120, 317, 112, 317, 128, TFT_BLACK);
  G->fillTriangle(160, BAR_H + 3, 152, BAR_H + 13, 168, BAR_H + 13, TFT_BLACK);
  G->fillTriangle(160, 237, 152, 227, 168, 227, TFT_BLACK);
}

// v184N: a station whose trail changed since it was drawn (new
// point, or points expiring out of the 10-minute window) needs a
// repaint on the same once-a-minute gate as the mobile markers.
static bool trailRepaintNeeded() {
  int px, py;
  for (int i = 0; i < MAX_STATIONS; i++) {
    if (!stations[i].used || !snapUsed[i]) continue;
    if (!geoToPx(stations[i].lat, stations[i].lon, px, py) || py < BAR_H + 4) continue;
    if (trailSigChanged(i)) return true;
  }
  return false;
}

// APRS display policy (user request): fixed stations are stored and
// the screen only repaints when the visible set grows/shrinks; mobile
// stations repaint once a minute (checked in loop()).
static bool mobileOnScreenMoved() {
  int px, py;
  for (int i = 0; i < MAX_STATIONS; i++) {
    if (!stations[i].used || !stations[i].mobile) continue;
    if (!geoToPx(stations[i].lat, stations[i].lon, px, py) || py < BAR_H + 4) continue;
    if (snapUsed[i] && (px != snapPx[i] || py != snapPy[i])) return true;
  }
  return false;
}

static bool stationsOnScreenChanged() {
  int px, py;
  if (screenMode == 0) {
    if (selected != snapSelected) return true;
    for (int i = 0; i < MAX_STATIONS; i++) {
      bool vis = false;
      if (stations[i].used && geoToPx(stations[i].lat, stations[i].lon, px, py) && py >= BAR_H + 4) vis = true;
      if (vis != snapUsed[i]) return true;
    }
    return false;
  }
  if (screenMode == 2) {
    if (countShips() != snapShipCount) return true;
    for (int i = 0; i < MAX_SHIPS; i++) {
      if (!ships[i].used || !ships[i].hasPos) { if (snapShipPx[i] >= 0) return true; continue; }
      bool vis = geoToPx(ships[i].lat, ships[i].lon, px, py) && py >= BAR_H + 4;
      if (!vis) { if (snapShipPx[i] >= 0) return true; continue; }
      if (snapShipPx[i] != px || snapShipPy[i] != py) return true;
    }
    return false;
  }
  if (acCount != snapAcCount) return true;
  for (int i = 0; i < acCount; i++) {
    bool vis = geoToPx(aircraft[i].lat, aircraft[i].lon, px, py) && py >= BAR_H + 4;
    if (!vis) { if (snapAcPx[i] >= 0) return true; continue; }
    if (snapAcPx[i] != px || snapAcPy[i] != py) return true;
  }
  return false;
}

static double haversineKm(double la1, double lo1, double la2, double lo2) {
  double dLa = (la2 - la1) * DEG_TO_RAD, dLo = (lo2 - lo1) * DEG_TO_RAD;
  double a = sin(dLa / 2) * sin(dLa / 2) + cos(la1 * DEG_TO_RAD) * cos(la2 * DEG_TO_RAD) * sin(dLo / 2) * sin(dLo / 2);
  return 6371.0 * 2 * atan2(sqrt(a), sqrt(1 - a));
}
static double bearingDeg(double la1, double lo1, double la2, double lo2) {
  double y = sin((lo2 - lo1) * DEG_TO_RAD) * cos(la2 * DEG_TO_RAD);
  double x = cos(la1 * DEG_TO_RAD) * sin(la2 * DEG_TO_RAD) - sin(la1 * DEG_TO_RAD) * cos(la2 * DEG_TO_RAD) * cos((lo2 - lo1) * DEG_TO_RAD);
  double b = atan2(y, x) * RAD_TO_DEG;
  return b < 0 ? b + 360 : b;
}

static void drawAcDetail() {
  int idx = -1;
  for (int i = 0; i < acCount; i++)
    if (!strcmp(aircraft[i].flight, selAcFlight)) { idx = i; break; }
  if (idx < 0) { detailOpen = false; detailIsAc = false; return; }
  const Aircraft& a = aircraft[idx];
  const RouteInfo* r = findRoute(a.flight);
  G->fillRoundRect(8, 30, 304, 190, 6, 0x1082);   // dark slate
  G->drawRoundRect(8, 30, 304, 190, 6, TFT_WHITE);
  G->setTextColor(TFT_RED, 0x1082);
  G->setFont(&fonts::Font2);
  G->setCursor(288, 34); G->print("X");
  G->setTextColor(TFT_GREEN, 0x1082);   // S3 v13: match the marker colour
  G->setCursor(16, 34);
  G->print(a.flight);
  G->setFont(&fonts::Font0);
  G->setTextColor(TFT_WHITE, 0x1082);
  int y = 58;
  char buf[96];
  if (r && r->airline[0]) { G->setCursor(16, y); G->print(r->airline); y += 14; }
  if (r && r->ok) {
    snprintf(buf, sizeof(buf), "FROM %s", r->orig);
    G->setCursor(16, y); G->print(buf); y += 12;
    snprintf(buf, sizeof(buf), "TO   %s", r->dest);
    G->setCursor(16, y); G->print(buf); y += 14;
  } else {
    G->setCursor(16, y); G->print("ROUTE UNKNOWN"); y += 14;
  }
  if (a.altFt >= 0)
    snprintf(buf, sizeof(buf), "ALT %ld ft (%ld m)", a.altFt, (long)(a.altFt * 0.3048));
  else
    snprintf(buf, sizeof(buf), "ALT ---- (on ground?)");
  G->setCursor(16, y); G->print(buf); y += 12;
  if (a.gsKt >= 0) {
    snprintf(buf, sizeof(buf), "GS %d kt (%d km/h)  HDG %03d", a.gsKt, (int)(a.gsKt * 1.852), (int)a.track);
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  double d = haversineKm(homeLat, homeLon, a.lat, a.lon);
  double brg = bearingDeg(homeLat, homeLon, a.lat, a.lon);
  snprintf(buf, sizeof(buf), "DIST %.1f km  BRG %.0f deg", d, brg);
  G->setCursor(16, y); G->print(buf);
}

static void drawShipDetail() {
  Ship* s = selShipMmsi ? findShip(selShipMmsi, false) : nullptr;
  if (!s || !s->hasPos) { detailOpen = false; return; }
  G->fillRoundRect(8, 30, 304, 190, 6, 0x1082);   // dark slate
  G->drawRoundRect(8, 30, 304, 190, 6, TFT_WHITE);
  G->setTextColor(TFT_RED, 0x1082);
  G->setFont(&fonts::Font2);
  G->setCursor(288, 34); G->print("X");
  G->setTextColor(TFT_CYAN, 0x1082);   // S3 v10: match the marker colour
  G->setCursor(16, 34);
  char title[28];
  if (s->name[0]) strncpy(title, s->name, sizeof(title) - 1);
  else snprintf(title, sizeof(title), "%lu", (unsigned long)s->mmsi);
  title[sizeof(title) - 1] = 0;
  G->print(title);
  G->setFont(&fonts::Font0);
  G->setTextColor(TFT_WHITE, 0x1082);
  int y = 58;
  char buf[96];
  snprintf(buf, sizeof(buf), "MMSI %lu", (unsigned long)s->mmsi);
  G->setCursor(16, y); G->print(buf); y += 12;
  const char* tt = shipTypeText(s->type);
  if (tt[0]) {
    snprintf(buf, sizeof(buf), "TYPE %s", tt);
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  snprintf(buf, sizeof(buf), "LAT %.4f  LON %.4f", s->lat, s->lon);
  G->setCursor(16, y); G->print(buf); y += 12;
  double d = haversineKm(homeLat, homeLon, s->lat, s->lon);
  double brg = bearingDeg(homeLat, homeLon, s->lat, s->lon);
  snprintf(buf, sizeof(buf), "DIST %.1f km  BRG %.0f deg", d, brg);
  G->setCursor(16, y); G->print(buf); y += 12;
  if (s->sog >= 0.5f && s->cog < 360.0f)
    snprintf(buf, sizeof(buf), "SOG %.1f kt (%d km/h)  COG %03d", s->sog, (int)(s->sog * 1.852), (int)s->cog);
  else
    snprintf(buf, sizeof(buf), "SOG %.1f kt (anchored / slow)", s->sog);
  G->setCursor(16, y); G->print(buf); y += 12;
  if (s->dest[0]) {
    snprintf(buf, sizeof(buf), "DEST %.30s", s->dest);
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  snprintf(buf, sizeof(buf), "LAST %lu min ago", (unsigned long)((millis() - s->lastMs) / 60000));
  G->setCursor(16, y); G->print(buf);
}

static void drawDetail() {
  if (screenMode == 2) { drawShipDetail(); return; }
  if (detailIsAc) { drawAcDetail(); return; }
  if (selected < 0 || !stations[selected].used) { detailOpen = false; return; }
  const Station& s = stations[selected];
  G->fillRoundRect(8, 30, 304, 190, 6, 0x1082);   // dark slate
  G->drawRoundRect(8, 30, 304, 190, 6, TFT_WHITE);
  G->setTextColor(TFT_RED, 0x1082);
  G->setFont(&fonts::Font2);
  G->setCursor(288, 34); G->print("X");
  G->setTextColor(TFT_YELLOW, 0x1082);
  G->setCursor(16, 34);
  G->print(s.call);
  G->setFont(&fonts::Font0);
  G->setTextColor(TFT_WHITE, 0x1082);
  int y = 58;
  char buf[96];
  snprintf(buf, sizeof(buf), "LAT %.4f  LON %.4f", s.lat, s.lon);
  G->setCursor(16, y); G->print(buf); y += 12;
  double d = haversineKm(homeLat, homeLon, s.lat, s.lon);
  double brg = bearingDeg(homeLat, homeLon, s.lat, s.lon);
  snprintf(buf, sizeof(buf), "DIST %.1f km  BRG %.0f deg", d, brg);
  G->setCursor(16, y); G->print(buf); y += 12;
  if (s.course >= 0 && s.speedKt >= 0) {
    snprintf(buf, sizeof(buf), "COURSE %03d  SPEED %d kt (%d km/h)", s.course, s.speedKt, (int)(s.speedKt * 1.852));
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  if (s.hasAlt) {
    snprintf(buf, sizeof(buf), "ALT %ld ft (%ld m)", s.altFt, (long)(s.altFt * 0.3048));
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  uint32_t ageMin = (s.lastUnix && time(nullptr) > 1700000000)
      ? ((uint32_t)time(nullptr) - s.lastUnix) / 60
      : (millis() - s.lastMs) / 60000;
  snprintf(buf, sizeof(buf), "LAST %lu min ago   SYM %c%c", (unsigned long)ageMin, s.symTable, s.symCode);
  G->setCursor(16, y); G->print(buf); y += 12;
  if (s.via[0]) {
    snprintf(buf, sizeof(buf), "VIA %.40s", s.via);
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  y += 4;
  G->setTextColor(TFT_CYAN, 0x1082);
  // comment, wrapped at ~48 chars, up to 5 lines
  const char* c = s.comment;
  while (*c && y < 210) {
    char line[52];
    int n = 0;
    while (c[n] && n < 48 && c[n] != '\n') n++;
    memcpy(line, c, n); line[n] = 0;
    G->setCursor(16, y); G->print(line);
    y += 12;
    c += n; if (*c == '\n') c++;
  }
}

static bool fullRedraw = true;
static bool mapDirty = true;              // view changed -> re-render the map from SD
static uint16_t* restoreChunk = nullptr;   // v78: dedicated 320x24 staging buffer (15,360 B)
static void restoreMapRect(int x0, int y0, int x1, int y1) {
  if (x0 < 0) x0 = 0; if (y0 < BAR_H) y0 = BAR_H;
  if (x1 > 319) x1 = 319; if (y1 > 239) y1 = 239;
  if (x1 < x0 || y1 < y0) return;
  int w = x1 - x0 + 1;
  // v78: batched again, but NOT the way v76 did it. v76 staged into
  // the SHARED mapChunk and pushed several blocks under one held
  // startWrite transaction - the buffer could be restaged while a
  // DMA transfer from it was still in flight, and the device began
  // rebooting on station taps (fixed by the v77 revert). Here each
  // chunk is pushed with a standalone pushImage (its own transaction,
  // which returns only after the transfer completes, so the buffer
  // is never reused early), and the staging buffer is dedicated to
  // this function instead of being shared with drawMap. Marker erase
  // is 1 push, the detail-panel area ~8 pushes, the full map area
  // ~10 - instead of 23 / 190 / 218 line-by-line transfers.
  if (!restoreChunk)
    // v109: INTERNAL | DMA (DRAM only). With plain INTERNAL this
    // 15,360 B staging buffer could land in the leftover-IRAM heap
    // region, whose halfword stores fault - the user's Guru
    // Meditation (LoadStoreError in restoreMapRect, EXCVADDR
    // 0x40095944) - the same root cause as the v87 sea-mask panic.
    restoreChunk = (uint16_t*)heap_caps_malloc(320 * 24 * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  if (!restoreChunk) {   // allocation failed: v75 line-by-line fallback
    for (int y = y0; y <= y1; y++) {
      for (int x = x0; x <= x1; x++) rowPixBuf[x - x0] = mapPixelAt(x, y);
      G->pushImage(x0, y, w, 1, rowPixBuf);
    }
    return;
  }
  int rowsPerPush = (320 * 24) / w;
  if (rowsPerPush < 1) rowsPerPush = 1;
  for (int y = y0; y <= y1;) {
    int n = min(rowsPerPush, y1 - y + 1);
    for (int r = 0; r < n; r++)
      for (int x = x0; x <= x1; x++)
        restoreChunk[r * w + (x - x0)] = mapPixelAt(x, y + r);
    G->pushImage(x0, y, w, n, restoreChunk);
    y += n;
  }
}

// When only markers moved, repaint just the bare map under their old
// spots (all markers are redrawn right after), not the whole map.
static void repairMovedObjects() {
  int px, py;
  for (int i = 0; i < MAX_STATIONS; i++) {
    if (!snapUsed[i]) continue;
    bool same = stations[i].used && geoToPx(stations[i].lat, stations[i].lon, px, py) &&
                py >= BAR_H + 4 && px == snapPx[i] && py == snapPy[i] &&
                i != selected && i != snapSelected &&
                !strcmp(snapCall[i], stations[i].call);   // v78: slot reused by another call
    // v184N: a drawn trail whose points changed (new point, expiry,
    // station gone) must be repainted even when the marker is same.
    bool trailStale = snapTrailCntD[i] >= 2 &&
        (!stations[i].used || trailSigChanged(i));
    if (same && !trailStale) continue;
    // Erase with the width that was DRAWN: if this slot has since been
    // reused by a station with a shorter callsign, using the current
    // call left the tail of the previous label on screen (user report).
    int w = snapLabelW[i];
    repairErased = true;
    if (!same)
      restoreMapRect(snapPx[i] - 9, snapPy[i] - 11, snapPx[i] + w, snapPy[i] + 11);
    if (snapTrailCntD[i] >= 2)
      restoreMapRect(snapTrailBox[i][0] - 2, snapTrailBox[i][1] - 2,
                     snapTrailBox[i][2] + 2, snapTrailBox[i][3] + 2);
  }
  for (int i = 0; i < MAX_AC; i++) {
    if (snapAcPx[i] < 0) continue;
    bool same = i < acCount && !strcmp(aircraft[i].flight, snapAcFlight[i]) &&
                geoToPx(aircraft[i].lat, aircraft[i].lon, px, py) &&
                py >= BAR_H + 4 && px == snapAcPx[i] && py == snapAcPy[i] &&
                snapAcTrack[i] == aircraft[i].track;   // v78: rotated triangle is not "same"
    // S3 v20: a trail that changed must be repainted even when
    // the marker itself is unchanged.
    bool trailStale = snapAcTrailCntD[i] > 0 &&
        (!(i < acCount) || acTrailStale(i));
    if (same && !trailStale) continue;
    int w = (int)strlen(snapAcFlight[i]) * 6 + 16;
    repairErased = true;
    if (!same)
      restoreMapRect(snapAcPx[i] - 13, snapAcPy[i] - 13, snapAcPx[i] + w, snapAcPy[i] + 13);
    if (snapAcTrailCntD[i] > 0)
      restoreMapRect(snapAcTrailBox[i][0] - 2, snapAcTrailBox[i][1] - 2,
                     snapAcTrailBox[i][2] + 2, snapAcTrailBox[i][3] + 2);
  }
  for (int i = 0; i < MAX_SHIPS; i++) {
    if (snapShipPx[i] < 0) continue;
    char labelNow[24];
    if (ships[i].name[0]) strncpy(labelNow, ships[i].name, sizeof(labelNow) - 1);
    else snprintf(labelNow, sizeof(labelNow), "%lu", (unsigned long)ships[i].mmsi);
    labelNow[sizeof(labelNow) - 1] = 0;
    bool underwayNow = ships[i].sog >= 0.5f && ships[i].cog < 360.0f;
    bool same = ships[i].used && ships[i].hasPos && ships[i].mmsi == snapShipMmsi[i] &&
                geoToPx(ships[i].lat, ships[i].lon, px, py) &&
                py >= BAR_H + 4 && px == snapShipPx[i] && py == snapShipPy[i] &&
                ships[i].cog == snapShipCog[i] && underwayNow == snapShipUnder[i] &&
                !strcmp(labelNow, snapShipName[i]);   // v78: course/shape/label all as drawn
    if (same) continue;
    char oldLabel[24];
    // label width as DRAWN (v78: from the snapshot, not the current
    // slot occupant - the ship may have been replaced in its slot)
    if (snapShipName[i][0]) strncpy(oldLabel, snapShipName[i], sizeof(oldLabel) - 1);
    else snprintf(oldLabel, sizeof(oldLabel), "%lu", (unsigned long)snapShipMmsi[i]);
    oldLabel[sizeof(oldLabel) - 1] = 0;
    int w = (int)strlen(oldLabel) * 6 + 16;
    repairErased = true;
    restoreMapRect(snapShipPx[i] - 13, snapShipPy[i] - 13, snapShipPx[i] + w, snapShipPy[i] + 13);
  }
}

static bool markerRepair = false;
// ---- visible-object list (LIST button, both screens) ----------------
#define LIST_ROWS 11
#define LIST_Y0 (BAR_H + 26)
#define LIST_ROWH 15
static int listIdx[MAX_STATIONS + MAX_AC];
static double listDist[MAX_STATIONS + MAX_AC];
static int listCount = 0;

static void buildList() {
  listCount = 0;
  int px, py;
  if (screenMode == 0) {
    for (int i = 0; i < MAX_STATIONS; i++) {
      if (!stations[i].used) continue;
      if (!geoToPx(stations[i].lat, stations[i].lon, px, py) || py < BAR_H) continue;
      listIdx[listCount] = i;
      listDist[listCount] = haversineKm(homeLat, homeLon, stations[i].lat, stations[i].lon);
      listCount++;
    }
  } else if (screenMode == 1) {
    for (int i = 0; i < acCount; i++) {
      if (!geoToPx(aircraft[i].lat, aircraft[i].lon, px, py) || py < BAR_H) continue;
      listIdx[listCount] = i;
      listDist[listCount] = haversineKm(homeLat, homeLon, aircraft[i].lat, aircraft[i].lon);
      listCount++;
    }
  } else {
    for (int i = 0; i < MAX_SHIPS; i++) {
      if (!ships[i].used || !ships[i].hasPos) continue;
      if (!geoToPx(ships[i].lat, ships[i].lon, px, py) || py < BAR_H) continue;
      listIdx[listCount] = i;
      listDist[listCount] = haversineKm(homeLat, homeLon, ships[i].lat, ships[i].lon);
      listCount++;
    }
  }
  for (int i = 1; i < listCount; i++) {   // nearest first
    int id = listIdx[i]; double dd = listDist[i]; int j = i - 1;
    while (j >= 0 && listDist[j] > dd) { listIdx[j + 1] = listIdx[j]; listDist[j + 1] = listDist[j]; j--; }
    listIdx[j + 1] = id; listDist[j + 1] = dd;
  }
}

static void drawList() {
  buildList();
  int pages = max(1, (listCount + LIST_ROWS - 1) / LIST_ROWS);
  if (listPage >= pages) listPage = pages - 1;
  if (listPage < 0) listPage = 0;
  G->fillRect(0, BAR_H, 320, 240 - BAR_H, 0x1082);
  G->setFont(&fonts::Font2);
  G->setTextColor(TFT_YELLOW, 0x1082);
  G->setCursor(10, BAR_H + 3);
  G->print(screenMode == 0 ? "APRS LIST" : (screenMode == 1 ? "FLIGHT LIST" : "SHIP LIST"));
  G->setTextColor(TFT_RED, 0x1082);
  G->setCursor(294, BAR_H + 3);
  G->print("X");
  G->setFont(&fonts::Font0);
  if (listCount == 0) {
    G->setTextColor(TFT_WHITE, 0x1082);
    const char* msg = screenMode == 0 ? "NO STATIONS VISIBLE"
                    : (screenMode == 1 ? "NO AIRCRAFT VISIBLE" : "NO SHIPS VISIBLE");
    G->setCursor((320 - G->textWidth(msg)) / 2, 120);
    G->print(msg);
  }
  int start = listPage * LIST_ROWS;
  for (int r = 0; r < LIST_ROWS && start + r < listCount; r++) {
    int id = listIdx[start + r];
    int y = LIST_Y0 + r * LIST_ROWH;
    char right[40];
    G->setTextColor(TFT_WHITE, 0x1082);
    G->setCursor(10, y + 3);
    if (screenMode == 0) {
      G->print(stations[id].call);
      snprintf(right, sizeof(right), "%.1f km", listDist[start + r]);
    } else if (screenMode == 1) {
      G->print(aircraft[id].flight);
      if (aircraft[id].altFt >= 0)
        snprintf(right, sizeof(right), "%.0f km  %d ft", listDist[start + r], aircraft[id].altFt);
      else
        snprintf(right, sizeof(right), "%.0f km", listDist[start + r]);
    } else {
      if (ships[id].name[0]) G->print(ships[id].name);
      else {
        char mb[16];
        snprintf(mb, sizeof(mb), "%lu", (unsigned long)ships[id].mmsi);
        G->print(mb);
      }
      snprintf(right, sizeof(right), "%.1f km  %.0f kn", listDist[start + r], ships[id].sog);
    }
    G->setCursor(310 - G->textWidth(right), y + 3);
    G->print(right);
    G->drawLine(8, y + LIST_ROWH - 2, 312, y + LIST_ROWH - 2, TFT_NAVY);
  }
  // paging footer
  G->setTextColor(TFT_WHITE, 0x1082);
  if (listPage > 0) { G->setCursor(10, 226); G->print("< PREV"); }
  if (listPage < pages - 1) { G->setCursor(268, 226); G->print("NEXT >"); }
  char pc[12];
  snprintf(pc, sizeof(pc), "%d/%d", listPage + 1, pages);
  G->setCursor((320 - G->textWidth(pc)) / 2, 226);
  G->print(pc);
}

// v78: the bare map was repainted under every marker (full map draw
// or a panel/list restore), so all "already on screen" snapshots are
// void and the draw* functions must draw everything again.
static void invalidateSnapshots() {
  for (int i = 0; i < MAX_STATIONS; i++) { snapUsed[i] = false; snapTrailCntD[i] = 0; snapMoving[i] = false; }
  snapSelected = -2;
  for (int i = 0; i < MAX_AC; i++) snapAcPx[i] = -1;
  snapAcCount = -1;
  for (int i = 0; i < MAX_SHIPS; i++) snapShipPx[i] = -1;
  snapShipCount = -1;
}

static void redraw() {
  static bool prevDetail = false, prevList = false;
  computeView();
  vImgRows = (int)lround(vSrcH * vScale);
  vImgCols = (int)lround(vSrcW * vScale);
  ensureSeaMask();
  static double lastViewLat = 0, lastViewLon = 0;
  static int lastViewZoom = -1, lastViewMap = -1;
  bool viewChanged = centerLat != lastViewLat || centerLon != lastViewLon
                  || zoomLevel != lastViewZoom || curMap != lastViewMap;
  bool mapRedrawn = false;
  bool barRepair = false;   // S3 v8 (= 028 v187N): tap ring overlapped the bar
  repairErased = false;
  if (mapDirty || viewChanged) {
    if (chunkMutex) xSemaphoreTake(chunkMutex, portMAX_DELAY);
    drawMap(); mapDirty = false;
    if (chunkMutex) xSemaphoreGive(chunkMutex);
    mapRedrawn = true;
    lastViewLat = centerLat; lastViewLon = centerLon;
    lastViewZoom = zoomLevel; lastViewMap = curMap;
    invalidateSnapshots();
  } else {
    // panel or list closed -> repaint the map area under it (the
    // list close had no repaint path before v76 and left the slate
    // background on screen).
    if (prevList && !listOpen) {
      restoreMapRect(0, BAR_H, 319, 239);   // the list covered everything
      invalidateSnapshots();
    } else if (prevDetail && !detailOpen) {
      restoreMapRect(7, 29, 312, 220);      // v78: only the panel's own rect
      invalidateSnapshots();
    } else repairMovedObjects();
    if (markerRepair) {
      restoreMapRect(tapX - 12, tapY - 12, tapX + 12, tapY + 12);
      markerRepair = false;
      repairErased = true;   // the tap ring may have clipped a marker
      // S3 v8 (= 028 v187N): restoreMapRect only repaints the map
      // area (y >= BAR_H), so a ring drawn over the bar itself -
      // a tap on [+] / [-] / a tab / LIST - was never erased and
      // stayed as a yellow ghost on the button. Force a full bar
      // repaint below when the ring touched the bar.
      if (tapY < BAR_H + 12) barRepair = true;
    }
    // v173N: left-edge cleanup (user report: label fragments
    // sometimes remain at the left edge, on APRS and on AIS).
    // Every erase rect checks out on paper, so as a guarantee:
    // whenever any marker erase ran (or the detail panel closed),
    // repaint the left strip from the map too - no stale fragment
    // can survive there. Current markers redraw right after this.
    if (repairErased || (prevDetail && !detailOpen))
      restoreMapRect(0, BAR_H, 47, 239);
  }
  // v78: unchanged markers may be skipped only when nothing was
  // erased or repainted under them this frame.
  skipUnchanged = !repairErased && !mapRedrawn;
  prevDetail = detailOpen;
  prevList = listOpen;
  // v76: a full map redraw overwrites the bar area too, so the bar
  // must be repainted; on a marker-only update the bar is untouched
  // on screen, so leave it to drawBarF's own change check instead of
  // forcing a full-bar blink on every update.
  if (mapRedrawn || barRepair) drawBarF(true, false);
  else drawBarF(false, false);
  drawHomeMarker();
  if (screenMode == 0) drawStations();
  else if (screenMode == 1) drawAircraft();
  else drawShips();
  drawPanHints();
  if (detailOpen) drawDetail();
  if (listOpen) drawList();
  if (tapShown) {
    G->drawCircle(tapX, tapY, 9, TFT_YELLOW);
    G->drawCircle(tapX, tapY, 10, TFT_YELLOW);
  }
  if (sprOk) spr.pushSprite(0, 0);
  fullRedraw = false;
  stationsDirty = false;
}

// ---------------- APRS parsing ----------------
static bool isDig(char c) { return c >= '0' && c <= '9'; }

// Parse an uncompressed position starting at p ("DDMM.mmN/DDDMM.mmWc...")
static bool parseUncomp(const char* p, Station& st, const char*& rest) {
  if (!isDig(p[0]) || !isDig(p[1]) || !isDig(p[2]) || !isDig(p[3]) || p[4] != '.' ||
      !isDig(p[5]) || !isDig(p[6]) || (p[7] != 'N' && p[7] != 'S')) return false;
  double lat = (p[0] - '0') * 10 + (p[1] - '0')
             + ((p[2] - '0') * 10 + (p[3] - '0') + ((p[5] - '0') * 10 + (p[6] - '0')) / 100.0) / 60.0;
  if (p[7] == 'S') lat = -lat;
  st.symTable = p[8];
  const char* q = p + 9;
  if (!isDig(q[0]) || !isDig(q[1]) || !isDig(q[2]) || !isDig(q[3]) || !isDig(q[4]) || q[5] != '.' ||
      !isDig(q[6]) || !isDig(q[7]) || (q[8] != 'E' && q[8] != 'W')) return false;
  double lon = (q[0] - '0') * 100 + (q[1] - '0') * 10 + (q[2] - '0')
             + ((q[3] - '0') * 10 + (q[4] - '0') + ((q[6] - '0') * 10 + (q[7] - '0')) / 100.0) / 60.0;
  if (q[8] == 'W') lon = -lon;
  st.symCode = q[9];
  st.lat = lat; st.lon = lon;
  rest = q + 10;
  return true;
}

// Parse a compressed position starting at p (table + 4 + 4 base91 + code ...)
static bool parseComp(const char* p, Station& st, const char*& rest) {
  for (int i = 1; i <= 8; i++) if (p[i] < '!' || p[i] > '{') return false;
  st.symTable = p[0];
  long yv = 0, xv = 0;
  for (int i = 0; i < 4; i++) yv = yv * 91 + (p[1 + i] - 33);
  for (int i = 0; i < 4; i++) xv = xv * 91 + (p[5 + i] - 33);
  st.lat = 90.0 - yv / 380926.0;
  st.lon = -180.0 + xv / 190463.0;
  st.symCode = p[9];
  rest = p + 10;
  // optional csT (course/speed or altitude): 3 base91-range bytes
  if (p[10] >= '!' && p[10] <= '{' && p[11] >= '!' && p[11] <= '{' && p[12] >= '!' && p[12] <= '{') {
    int cs1 = p[10] - 33, cs2 = p[11] - 33, tt = p[12] - 33;
    if (tt & 0x10) {
      st.hasAlt = true;
      st.altFt = (long)pow(1.002, cs1 * 91 + cs2);
    } else {
      st.course = cs1 * 4;
      st.speedKt = (int)(pow(1.08, cs2) - 1);
      if (st.course > 360 || st.speedKt > 799) { st.course = -1; st.speedKt = -1; }
    }
    rest = p + 13;
  }
  return true;
}

static bool parsePositionAt(const char* p, Station& st, const char*& rest) {
  if (isDig(p[0])) return parseUncomp(p, st, rest);
  return parseComp(p, st, rest);
}

// Mic-E digit from a destination-callsign character.
// flag: 0 = '0'-'9', 1 = 'A'-'J' (message bits, chars 1-3 only),
//       2 = 'P'-'Y' (the hemisphere/offset marks in chars 4-6)
static int micDigit(char c, int& flag) {
  flag = 0;
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'J') { flag = 1; return c - 'A'; }
  if (c >= 'P' && c <= 'Y') { flag = 2; return c - 'P'; }
  return 0; // K, L, Z = space
}

static bool parseMicE(const char* dst, const uint8_t* info, int infoLen, Station& st) {
  if (strlen(dst) < 6 || infoLen < 8) return false;
  int d[6], f[6];
  for (int i = 0; i < 6; i++) d[i] = micDigit(dst[i], f[i]);
  double lat = d[0] * 10 + d[1] + ((d[2] * 10 + d[3]) + (d[4] * 10 + d[5]) / 100.0) / 60.0;
  // Hemisphere/offset flags follow Dire Wolf's aprs_mic_e exactly:
  // char4: '0'-'9'/'L' = South, 'P'-'Z' = North (v12 had this backwards;
  // verified against JO4HJX-9>S4SXR4 - char4 'X', really in Mie, N).
  if (f[3] != 2) lat = -lat;
  bool lonOff = (f[4] == 2);                   // char5: 'P'-'Y' = +100 deg
  bool west = (f[5] == 2);                     // char6: 'P'-'Y' = West
  // Longitude bytes use wrapped ranges (also Dire Wolf's rules):
  int ch = info[0], lonDeg;
  if (lonOff && ch >= 118 && ch <= 127) lonDeg = ch - 118;
  else if (!lonOff && ch >= 38 && ch <= 127) lonDeg = ch - 38 + 10;
  else if (lonOff && ch >= 108 && ch <= 117) lonDeg = ch - 108 + 100;
  else if (lonOff && ch >= 38 && ch <= 107) lonDeg = ch - 38 + 110;
  else return false;
  ch = info[1];
  int lonMin;
  if (ch >= 88 && ch <= 97) lonMin = ch - 88;
  else if (ch >= 38 && ch <= 87) lonMin = ch - 38 + 10;
  else return false;
  ch = info[2];
  if (ch < 28 || ch > 127) return false;
  double lon = lonDeg + (lonMin + (ch - 28) / 100.0) / 60.0;
  if (west) lon = -lon;
  int sp = (info[3] - 28) * 10 + (info[4] - 28) / 10;
  int crs = ((info[4] - 28) % 10) * 100 + (info[5] - 28);
  if (crs >= 400) crs -= 400;
  if (sp >= 800) sp -= 800;
  st.lat = lat; st.lon = lon;
  st.symCode = (char)info[6];
  st.symTable = (char)info[7];
  if (sp < 200) st.speedKt = sp;
  if (crs < 360) st.course = crs;
  // comment follows the symbol table char
  if (infoLen > 8) {
    int n = infoLen - 8; if (n > 78) n = 78;
    memcpy(st.comment, info + 8, n); st.comment[n] = 0;
  }
  return true;
}

static void finishStation(Station& st, const char* rest) {
  // course/speed prefix "DDD/SSS" at the start of the comment
  const char* c = rest;
  if (isDig(c[0]) && isDig(c[1]) && isDig(c[2]) && c[3] == '/' && isDig(c[4]) && isDig(c[5]) && isDig(c[6])) {
    st.course = (c[0] - '0') * 100 + (c[1] - '0') * 10 + (c[2] - '0');
    st.speedKt = (c[4] - '0') * 100 + (c[5] - '0') * 10 + (c[6] - '0');
    c += 7;
    if (*c == ' ') c++;
  }
  strncpy(st.comment, c, sizeof(st.comment) - 1);
  st.comment[sizeof(st.comment) - 1] = 0;
  // altitude "/A=001234" anywhere in the comment
  const char* a = strstr(st.comment, "/A=");
  if (a) {
    char* end = nullptr;
    long v = strtol(a + 3, &end, 10);
    if (end != a + 3) { st.hasAlt = true; st.altFt = v; }
  }
}

static bool stPersistDirty = false;

static int findSlot(const char* call) {
  int slot = -1, oldest = 0;
  uint32_t oldestMs = 0xFFFFFFFF;
  for (int i = 0; i < MAX_STATIONS; i++) {
    if (stations[i].used && strcmp(stations[i].call, call) == 0) { slot = i; break; }
    if (!stations[i].used && slot < 0) slot = i;
    if (stations[i].used && stations[i].lastMs < oldestMs) { oldestMs = stations[i].lastMs; oldest = i; }
  }
  if (slot < 0) slot = oldest;
  return slot;
}

static void upsert(Station& st) {
  int slot = findSlot(st.call);
  bool mob = stations[slot].mobile;
  if (stations[slot].used &&
      (fabs(stations[slot].lat - st.lat) > 0.00005 ||
       fabs(stations[slot].lon - st.lon) > 0.00005)) mob = true;
  if (st.speedKt > 0) mob = true;
  st.mobile = mob;
  st.lastMs = millis();
  time_t t = time(nullptr);
  st.lastUnix = t > 1700000000 ? (uint32_t)t : 0;
  // v184N: record the track for the trail display. A slot taken
  // over by a different callsign starts with an empty trail; when a
  // station has moved, its previous position seeds the first point.
  if (stations[slot].used && strcmp(stations[slot].call, st.call) != 0)
    trailDrop(slot);
  if (stations[slot].used && !strcmp(stations[slot].call, st.call)) {
    Trail* tr = trailOf(slot, false);
    if ((!tr || tr->cnt == 0) &&
        trailDistM(stations[slot].lat, stations[slot].lon, st.lat, st.lon) >= 30.0f)
      trailPush(slot, stations[slot].lat, stations[slot].lon, stations[slot].lastMs);
    trailPush(slot, st.lat, st.lon, millis());
  }
  st.used = true;
  stations[slot] = st;
  stationsDirty = true;
  stPersistDirty = true;
  Serial.printf("STN %s lat %.4f lon %.4f\n", st.call, st.lat, st.lon);
}

// ---- station persistence: show the last-heard stations immediately
// after boot instead of waiting for each one to beacon again. The
// table is kept in NVS with real (NTP) timestamps; APRS-IS then keeps
// it live as before. (aprs.fi's API cannot replace this: it only
// answers per-callsign lookups, with no area search.)
struct PersistStation {          // v41: slimmer record (104 B)
  char call[16];
  float lat, lon;
  int16_t course, speedKt;
  int32_t altFt;
  uint8_t hasAlt;
  char comment[40];
  char via[24];
  uint32_t lastUnix;
};
struct PersistStationV1 {        // v35-v40 layout (144 B), for migration
  char call[20];
  float lat, lon;
  int16_t course, speedKt;
  int32_t altFt;
  uint8_t hasAlt;
  char comment[64];
  char via[36];
  uint32_t lastUnix;
};
#define MAX_PERSIST 56
static uint32_t lastPersistMs = 0;
static bool ntpStarted = false, stRestored = false;

static void saveStations() {
  time_t t = time(nullptr);
  if (t < 1700000000) return;
  static PersistStation buf[MAX_STATIONS];
  int n = 0;
  uint32_t nowM = millis();
  for (int i = 0; i < MAX_STATIONS; i++) {
    if (!stations[i].used) continue;
    PersistStation& p = buf[n++];
    memset(&p, 0, sizeof(p));
    strncpy(p.call, stations[i].call, sizeof(p.call) - 1);
    p.lat = (float)stations[i].lat;
    p.lon = (float)stations[i].lon;
    p.course = (int16_t)stations[i].course;
    p.speedKt = (int16_t)stations[i].speedKt;
    p.altFt = (int32_t)stations[i].altFt;
    p.hasAlt = stations[i].hasAlt ? 1 : 0;
    strncpy(p.comment, stations[i].comment, sizeof(p.comment) - 1);
    strncpy(p.via, stations[i].via, sizeof(p.via) - 1);
    p.lastUnix = stations[i].lastUnix ? stations[i].lastUnix
                                      : (uint32_t)t - (nowM - stations[i].lastMs) / 1000;
  }
  if (n > MAX_PERSIST) {   // keep the freshest ones (blob stays ~6 KB)
    for (int i = 0; i < MAX_PERSIST; i++) {
      int best = i;
      for (int j = i + 1; j < n; j++) if (buf[j].lastUnix > buf[best].lastUnix) best = j;
      if (best != i) { PersistStation tmp = buf[i]; buf[i] = buf[best]; buf[best] = tmp; }
    }
    n = MAX_PERSIST;
  }
  // v73: save in 8-station chunks (832 B each) instead of one 5824 B blob.
  // A single big blob needs one contiguous free run in the 20 KB NVS
  // partition and kept failing with "persist: short write 0/5824" once
  // the table filled up; each chunk only needs ~832 B free.
  Preferences pr;
  pr.begin("aprsst", false);
  const int CH = 8, NCH = (MAX_PERSIST + CH - 1) / CH;   // 7 chunks
  int nch = (n + CH - 1) / CH;
  bool ok = true;
  for (int c = 0; c < NCH; c++) {
    char key[4]; snprintf(key, sizeof(key), "t%d", c);
    if (c >= nch) { pr.remove(key); continue; }   // drop stale chunks
    int cn = min(CH, n - c * CH);
    size_t want = cn * sizeof(PersistStation);
    size_t w = pr.putBytes(key, buf + c * CH, want);
    if (w != want) {   // retry once after removing just this chunk
      Serial.printf("persist: chunk %d short write %u/%u - retry\n", c, (unsigned)w, (unsigned)want);
      pr.remove(key);
      w = pr.putBytes(key, buf + c * CH, want);
      Serial.printf("persist: chunk %d retry %u/%u\n", c, (unsigned)w, (unsigned)want);
    }
    if (w != want) ok = false;
  }
  if (ok) pr.remove("tbl");   // old single blob no longer needed
  pr.putInt("count", ok ? n : 0);
  if (ok && n) Serial.printf("persist: saved %d stations in %d chunks\n", n, nch);
  pr.end();
}

static void restoreStations() {
  time_t nowT = time(nullptr);
  if (nowT < 1700000000) return;
  Preferences pr;
  pr.begin("aprsst", true);
  int n = pr.getInt("count", 0);
  if (n <= 0 || n > MAX_STATIONS) { Serial.printf("persist: no data (count %d)\n", n); pr.end(); return; }
  static PersistStation buf[MAX_STATIONS];
  // v73: chunked layout t0..t6 (8 stations each) first; fall back to the
  // old single "tbl" blob (incl. v35-v40 records) when no chunks exist.
  if (pr.getBytesLength("t0") > 0) {
    int gotN = 0;
    const int CH = 8;
    for (int c = 0; c * CH < n; c++) {
      char key[4]; snprintf(key, sizeof(key), "t%d", c);
      int cn = min(CH, n - c * CH);
      size_t got = pr.getBytes(key, buf + c * CH, cn * sizeof(PersistStation));
      if (got != (size_t)(cn * sizeof(PersistStation))) {
        Serial.printf("persist: chunk %d short read %u/%u\n", c, (unsigned)got,
                      (unsigned)(cn * sizeof(PersistStation)));
        break;
      }
      gotN += cn;
    }
    n = gotN;
    if (!n) { Serial.println("persist: no chunk data"); pr.end(); return; }
  } else {
    size_t got = pr.getBytes("tbl", buf, n * sizeof(PersistStation));
    if (got != (size_t)(n * sizeof(PersistStation))) {
      // maybe a v35-v40 blob (bigger records): convert it
      // v79: buf1 was a 13.8 KB function static; BSS no longer has
      // room for it next to the map-generation statics, and this
      // one-time migration path can borrow the heap instead.
      PersistStationV1* buf1 =
          (PersistStationV1*)heap_caps_malloc(n * sizeof(PersistStationV1), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);   // v109: DRAM only (byte-accessed)
      size_t got1 = buf1 ? pr.getBytes("tbl", buf1, n * sizeof(PersistStationV1)) : 0;
      if (buf1 && got1 == (size_t)(n * sizeof(PersistStationV1))) {
        for (int i = 0; i < n; i++) {
          memset(&buf[i], 0, sizeof(buf[i]));
          strncpy(buf[i].call, buf1[i].call, sizeof(buf[i].call) - 1);
          buf[i].lat = buf1[i].lat; buf[i].lon = buf1[i].lon;
          buf[i].course = buf1[i].course; buf[i].speedKt = buf1[i].speedKt;
          buf[i].altFt = buf1[i].altFt; buf[i].hasAlt = buf1[i].hasAlt;
          strncpy(buf[i].comment, buf1[i].comment, sizeof(buf[i].comment) - 1);
          strncpy(buf[i].via, buf1[i].via, sizeof(buf[i].via) - 1);
          buf[i].lastUnix = buf1[i].lastUnix;
        }
        Serial.println("persist: legacy blob migrated");
      } else {
        Serial.printf("persist: size mismatch (count %d, got %u/%u)\n",
                      n, (unsigned)got, (unsigned)got1);
        free(buf1);
        pr.end();
        return;
      }
      free(buf1);
    }
  }
  pr.end();
  int added = 0;
  for (int i = 0; i < n; i++) {
    if (!buf[i].call[0] || !buf[i].lastUnix) continue;
    uint32_t age = (uint32_t)nowT - buf[i].lastUnix;
    if (age > (uint32_t)STATION_EXPIRE_MIN * 60) continue;
    Station st;
    strncpy(st.call, buf[i].call, sizeof(st.call) - 1);
    st.call[sizeof(st.call) - 1] = 0;
    st.lat = buf[i].lat;
    st.lon = buf[i].lon;
    st.course = buf[i].course;
    st.speedKt = buf[i].speedKt;
    st.hasAlt = buf[i].hasAlt != 0;
    st.altFt = buf[i].altFt;
    strncpy(st.comment, buf[i].comment, sizeof(st.comment) - 1);
    strncpy(st.via, buf[i].via, sizeof(st.via) - 1);
    st.lastUnix = buf[i].lastUnix;
    uint32_t ageMs = age * 1000UL;
    st.lastMs = ageMs < millis() ? millis() - ageMs : 0;
    st.used = true;
    stations[findSlot(st.call)] = st;
    added++;
  }
  if (added) {
    stationsDirty = true;
    Serial.printf("restored %d stations\n", added);
  } else {
    Serial.printf("persist: %d entries, none fresh\n", n);
  }
}

static void parsePacket(const char* line) {
  const char* gt = strchr(line, '>');
  const char* colon = gt ? strchr(gt, ':') : nullptr;
  if (!gt || !colon || colon == gt + 1) return;
  char src[20];
  int sl = min((int)(gt - line), 19);
  memcpy(src, line, sl); src[sl] = 0;
  char dst[16];
  const char* comma = strchr(gt + 1, ',');
  const char* dstEnd = (comma && comma < colon) ? comma : colon;
  int dl = min((int)(dstEnd - (gt + 1)), 15);
  memcpy(dst, gt + 1, dl); dst[dl] = 0;

  const char* info = colon + 1;
  if (!*info) return;
  Station st;
  strncpy(st.call, src, sizeof(st.call) - 1);
  if (comma && comma < colon) {
    int vl = min((int)(colon - (comma + 1)), (int)sizeof(st.via) - 1);
    memcpy(st.via, comma + 1, vl); st.via[vl] = 0;
  }
  bool ok = false;
  const char* rest = "";
  switch (info[0]) {
    case '!': case '=':
      ok = parsePositionAt(info + 1, st, rest);
      break;
    case '/': case '@':
      if (strlen(info) > 8) ok = parsePositionAt(info + 8, st, rest);
      break;
    case ';': {  // object
      if (strlen(info) < 18 || info[10] == '_') return;  // killed object
      char name[12];
      memcpy(name, info + 1, 9); name[9] = 0;
      for (int i = 8; i >= 0; i--) { if (name[i] == ' ') name[i] = 0; else break; }
      strncpy(st.call, name, sizeof(st.call) - 1);
      ok = parsePositionAt(info + 18, st, rest);
      break;
    }
    case ')': {  // item
      int i = 1;
      while (info[i] && info[i] != '!' && info[i] != '_' && i < 11) i++;
      if (!info[i] || info[i] == '_') return;
      char name[12];
      int nl = min(i - 1, 11);
      memcpy(name, info + 1, nl); name[nl] = 0;
      strncpy(st.call, name, sizeof(st.call) - 1);
      ok = parsePositionAt(info + i + 1, st, rest);
      break;
    }
    case '`': case '\'':
      ok = parseMicE(dst, (const uint8_t*)info + 1, strlen(info) - 1, st);
      if (ok) { upsert(st); packetCount++; return; }
      return;
    default:
      return;
  }
  if (!ok) return;
  if (st.lat < -90 || st.lat > 90 || st.lon < -180 || st.lon > 180) return;
  if (st.lat == 0 && st.lon == 0) return;
  finishStation(st, rest);
  upsert(st);
  packetCount++;
}

// ---------------- APRS-IS ----------------
static WiFiClient client;
enum AprsState { A_WIFI, A_CONNECT, A_LOGIN, A_ONLINE };
static AprsState aState = A_WIFI;
static uint32_t aStateMs = 0, lastRxMs = 0, lastKeepaliveMs = 0;
static char lineBuf[1024];
static int lineLen = 0;
static bool filterSent = false;

static const char* aprsStateText() {
  static char buf[40];
  // Count only stations inside the current map: activity outside the
  // map must not change the screen (user request).
  int n = 0;
  const MapDef& m = MAP_LIST[curMap];
  for (int i = 0; i < MAX_STATIONS; i++)
    if (stations[i].used && stations[i].lat <= m.north && stations[i].lat >= m.south &&
        stations[i].lon >= m.west && stations[i].lon <= m.east) n++;
  switch (aState) {
    case A_WIFI:   return "WiFi...";
    case A_CONNECT: return "APRS...";
    case A_LOGIN:  return "LOGIN...";
    default:
      snprintf(buf, sizeof(buf), "APRS %d st", n);
      return buf;
  }
}

static int aprsPasscode(const char* call) {
  char base[16];
  int i = 0;
  while (call[i] && call[i] != '-' && i < 15) { base[i] = toupper((uint8_t)call[i]); i++; }
  base[i] = 0;
  int h = 0x73e2;
  int len = strlen(base);
  for (int k = 0; k + 1 < len; k += 2) { h ^= (base[k] << 8); h ^= base[k + 1]; }
  if (len % 2) h ^= (base[len - 1] << 8);
  return h & 0x7FFF;
}

static void aprsSetup() {
  // wifiPhase() in setup() has already connected (or portal-configured)
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
  }
  aState = (WiFi.status() == WL_CONNECTED) ? A_CONNECT : A_WIFI;
  aStateMs = millis();
}

static void aprsLoop() {
  uint32_t now = millis();
  switch (aState) {
    case A_WIFI:
      if (WiFi.status() == WL_CONNECTED) { aState = A_CONNECT; aStateMs = now; }
      else if (now - aStateMs > 20000) { WiFi.disconnect(); WiFi.begin(cfgSsid.c_str(), cfgPass.c_str()); aStateMs = now; }
      break;
    case A_CONNECT:
      if (WiFi.status() != WL_CONNECTED) { aState = A_WIFI; aStateMs = now; break; }
      client.stop();
      if (client.connect(APRS_HOST, APRS_PORT)) {
        int pass = cfgPasscode;
        if (pass == 0) pass = aprsPasscode(cfgCall.c_str());
        client.printf("user %s pass %d vers MuseAPRSMap 1.0\r\n", cfgCall.c_str(), pass);
        filterSent = false;
        lineLen = 0;
        lastRxMs = now;
        aState = A_LOGIN; aStateMs = now;
      } else if (now - aStateMs > 10000) {
        aStateMs = now;  // retry
      }
      break;
    case A_LOGIN:
    case A_ONLINE: {
      if (!client.connected()) { aState = A_CONNECT; aStateMs = now; break; }
      while (client.available()) {
        char ch = (char)client.read();
        lastRxMs = now;
        if (ch == '\n' || ch == '\r') {
          if (lineLen > 0) {
            lineBuf[lineLen] = 0;
            if (lineBuf[0] == '#') {
              if (aState == A_LOGIN && strstr(lineBuf, "logresp")) {
                client.printf("#filter r/%.2f/%.2f/%d\r\n", homeLat, homeLon, cfgRangeKm);
                filterSent = true;
                aState = A_ONLINE;
              }
            } else {
              parsePacket(lineBuf);
            }
            lineLen = 0;
          }
        } else if (lineLen < (int)sizeof(lineBuf) - 1) {
          lineBuf[lineLen++] = ch;
        }
      }
      if (aState == A_LOGIN && now - aStateMs > 6000 && !filterSent) {
        // server gave no logresp line; send the filter anyway
        client.printf("#filter r/%.2f/%.2f/%d\r\n", homeLat, homeLon, cfgRangeKm);
        filterSent = true;
        aState = A_ONLINE;
      }
      if (aState == A_ONLINE) {
        if (now - lastKeepaliveMs > 240000) { client.print("# keepalive\r\n"); lastKeepaliveMs = now; }
        if (now - lastRxMs > 300000) { client.stop(); aState = A_CONNECT; aStateMs = now; }
      }
      break;
    }
  }
}

// ---------------- AIS ships (Open Waters snapshot) ----------------
// AIS comes from the Open Waters (aiscast) snapshot API: it
// aggregates several AIS networks (AISHub and others) and serves a
// GeoJSON snapshot of the vessels inside a bounding box over plain
// HTTPS - no API key needed. Polled every 60 s while the AIS screen
// is shown (plus once on tab arrival); this is the only AIS source.
// Runs on the Core 0 task; the map chunk is freed first for
// handshake headroom, like an ADS-B fetch.
static void fetchAisSnapshotDo() {
  if (WiFi.status() != WL_CONNECTED) return;
  freeMapChunk();
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(12000);
  char url[192];
  snprintf(url, sizeof(url),
           "https://ais.openwaters.io/v1/vessels?bbox=%.4f,%.4f,%.4f,%.4f&kind=vessel",
           MAP_LIST[1].south, MAP_LIST[1].west, MAP_LIST[1].north, MAP_LIST[1].east);
  if (!http.begin(client, url)) return;
  http.addHeader("User-Agent", "APRSMap-ESP32/1.0");
  // S3 v28 diag: http -1 on v27 - separate DNS / heap / TLS causes
  // before changing anything else (same pattern as fetchFromSource).
  {
    IPAddress ip;
    bool dnsOk = WiFi.hostByName("ais.openwaters.io", ip);
    Serial.printf("AIS snapshot start: dns %s (%s) free %u largest %u dmaLargest %u\n",
                  dnsOk ? "ok" : "FAIL", dnsOk ? ip.toString().c_str() : "-",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
  }
  int code = http.GET();
  if (code < 0) {
    char ebuf[80];
    client.lastError(ebuf, sizeof(ebuf));
    Serial.printf("AIS snapshot http %d ssl: %s\n", code, ebuf);
    http.end();
    delay(700);
    if (!http.begin(client, url)) return;
    http.addHeader("User-Agent", "APRSMap-ESP32/1.0");
    code = http.GET();
    if (code < 0) {
      client.lastError(ebuf, sizeof(ebuf));
      Serial.printf("AIS snapshot retry http %d ssl: %s\n", code, ebuf);
    }
  }
  if (code != 200) {
    Serial.printf("AIS snapshot http %d\n", code);
    http.end();
    return;
  }
  lastTlsOkMs = millis();   // S3 v17 stamp (heap-death exclusion)
  String body = http.getString();
  http.end();
  aisLastSnapOkMs = millis();
  time_t nowT = time(nullptr);
  bool timeOk = nowT > 1700000000;
  int total = 0, added = 0;
  int blen = (int)body.length();
  int pos = 0;
  while (pos < blen) {
    int c = body.indexOf("\"coordinates\":[", pos);
    if (c < 0) break;
    const char* cp = body.c_str() + c + 15;
    double lo = strtod(cp, nullptr);
    const char* comma = strchr(cp, ',');
    if (!comma) break;
    double la = strtod(comma + 1, nullptr);
    int m = body.indexOf("\"mmsi\":", c);
    if (m < 0) break;
    int next = body.indexOf("\"coordinates\":[", c + 15);
    int segEnd = next >= 0 ? next : blen;
    uint32_t mmsi = (uint32_t)strtoul(body.c_str() + m + 7, nullptr, 10);
    pos = m + 7;
    if (!mmsi) continue;
    // Age from "seen" (UTC ISO8601): snapshot entries age out on
    // the same 30 min rule as stream ships.
    uint32_t ageSec = 0;
    if (timeOk) {
      int sk = body.indexOf("\"seen\":\"", m);
      if (sk >= 0 && sk < segEnd) {
        int yy, mo, dd, hh, mi, ss;
        if (sscanf(body.c_str() + sk + 8, "%d-%d-%dT%d:%d:%d",
                   &yy, &mo, &dd, &hh, &mi, &ss) == 6) {
          struct tm tmv = {};
          tmv.tm_year = yy - 1900; tmv.tm_mon = mo - 1; tmv.tm_mday = dd;
          tmv.tm_hour = hh; tmv.tm_min = mi; tmv.tm_sec = ss;
          time_t seenT = mktime(&tmv);   // TZ is UTC (configTime(0,0,...))
          if (seenT > 0 && nowT >= seenT) ageSec = (uint32_t)(nowT - seenT);
        }
      }
    }
    if (ageSec > 1800) continue;
    Ship* s = findShip(mmsi, true);
    if (!s) continue;
    bool isNew = !s->used || !s->hasPos;
    s->used = true; s->hasPos = true; s->mmsi = mmsi;
    s->lat = la; s->lon = lo;
    double v;
    if (jsonNumField(body, "\"sog\":", m, segEnd, v)) s->sog = (v <= 102.2) ? (float)v : 0;
    if (jsonNumField(body, "\"cog\":", m, segEnd, v)) s->cog = (float)v;
    if (jsonNumField(body, "\"nav_status\":", m, segEnd, v)) s->navStatus = (int)v;
    if (jsonNumField(body, "\"type\":", m, segEnd, v)) s->type = (int)v;
    char buf[28];
    if (jsonStrField(body, "\"name\":\"", m, segEnd, buf, sizeof(buf))) {
      trimSpaces(buf);
      if (buf[0]) { strncpy(s->name, buf, sizeof(s->name) - 1); s->name[sizeof(s->name) - 1] = 0; }
    }
    if (jsonStrField(body, "\"destination\":\"", m, segEnd, buf, sizeof(buf))) {
      trimSpaces(buf);
      if (strncmp(buf, "\\u003e", 6) == 0) {   // ">JP ..." port prefix
        memmove(buf + 1, buf + 6, strlen(buf + 6) + 1);
        buf[0] = '>';
      }
      if (buf[0]) { strncpy(s->dest, buf, sizeof(s->dest) - 1); s->dest[sizeof(s->dest) - 1] = 0; }
    }
    s->lastMs = millis() - ageSec * 1000UL;
    if (isNew) {
      added++;
      Serial.printf("AIS ship (snapshot): %s (%lu)\n",
                    s->name[0] ? s->name : "?", (unsigned long)mmsi);
    }
    total++;
    stationsDirty = true;
  }
  Serial.printf("AIS snapshot (Open Waters): %d ships, %d new\n", total, added);
}
// ---------------- touch ----------------
static uint32_t lastTouchMs = 0;

static void clampCenter() {
  const MapDef& m = MAP_LIST[curMap];
  double latSpan = viewLatSpan(), lonSpan = viewLonSpan();
  if (centerLat + latSpan / 2 > m.north) centerLat = m.north - latSpan / 2;
  if (centerLat - latSpan / 2 < m.south) centerLat = m.south + latSpan / 2;
  if (centerLon + lonSpan / 2 > m.east)  centerLon = m.east - lonSpan / 2;
  if (centerLon - lonSpan / 2 < m.west)  centerLon = m.west + lonSpan / 2;
}


// ---- screen switching ------------------------------------------------
static void switchMode(int m) {
  if (!((cfgScreens >> m) & 1)) return;   // v175N: hidden screen
  if (m == screenMode) return;
  // v116: centre and zoom are SHARED by all screens (one map, one
  // view) - switching tabs changes only the objects drawn.
  screenMode = m;
  // S3 v16: arriving on a screen starts its update at once
  // instead of waiting out the interval / retry backoff.
  if (m == 1) lastAcFetchMs = 0;
  if (m == 2) aisSnapReq = true;   // S3 v27: snapshot poll at once on arrival
  curMap = MODE_MAP[m];
  selected = -1;
  selAcFlight[0] = 0;
  selShipMmsi = 0;
  detailIsAc = false;
  detailOpen = false;
  listOpen = false;
  mapDirty = true;
  fullRedraw = true;
}

// ---- list touch -------------------------------------------------------
static void handleListTouch(int x, int y) {
  buildList();
  int pages = max(1, (listCount + LIST_ROWS - 1) / LIST_ROWS);
  if (y >= 218) {   // paging footer
    if (x < 100 && listPage > 0) listPage--;
    else if (x >= 220 && listPage < pages - 1) listPage++;
    fullRedraw = true;
    return;
  }
  if (y < LIST_Y0 - 4 && x >= 280) { listOpen = false; fullRedraw = true; return; }   // X
  int row = (y - LIST_Y0) / LIST_ROWH;
  if (row < 0 || row >= LIST_ROWS) return;
  int idx = listPage * LIST_ROWS + row;
  if (idx >= listCount) return;
  int id = listIdx[idx];
  listOpen = false;
  if (zoomLevel < 1) zoomLevel = 1;   // so the centre move is visible
  if (screenMode == 0) {
    selected = id;
    selAcFlight[0] = 0; detailIsAc = false;
    centerLat = stations[id].lat;
    centerLon = stations[id].lon;
    clampCenter();
    detailOpen = true;
  } else if (screenMode == 2) {
    selShipMmsi = ships[id].mmsi;
    selected = -1; selAcFlight[0] = 0; detailIsAc = false;
    centerLat = ships[id].lat;
    centerLon = ships[id].lon;
    clampCenter();
    detailOpen = true;
  } else {
    strncpy(selAcFlight, aircraft[id].flight, sizeof(selAcFlight) - 1);
    selAcFlight[sizeof(selAcFlight) - 1] = 0;
    selected = -1; detailIsAc = true;
    centerLat = aircraft[id].lat;
    centerLon = aircraft[id].lon;
    clampCenter();
    detailOpen = true;
    ensureRoute(selAcFlight);
  }
  mapDirty = true;
  fullRedraw = true;
}

static void handleTouch(int x, int y) {
  tapX = x; tapY = y; tapMs = millis(); tapShown = false;
  Serial.printf("touch %d,%d mode=%d zoom=%d\n", x, y, screenMode, zoomLevel);   // v115 diag
  if (listOpen) {
    if (y < BAR_H) {   // bar stays live: tabs switch, LIST closes
      if (barTabX[0] >= 0 && x >= barTabX[0] && x < barTabX[0] + barTabW[0]) switchMode(0);
      else if (barTabX[1] >= 0 && x >= barTabX[1] && x < barTabX[1] + barTabW[1]) switchMode(1);
      else if (barTabX[2] >= 0 && x >= barTabX[2] && x < barTabX[2] + barTabW[2]) switchMode(2);
      else if (x >= barListX && x < barListX + 39) listOpen = false;
      fullRedraw = true;
      return;
    }
    handleListTouch(x, y);
    return;
  }
  if (detailOpen) { detailOpen = false; fullRedraw = true; return; }
  // bar: tabs, [LIST], status, [-] [+] (v171N layout). Hit zone
  // reaches y<70 for the button ranges - bar-aimed taps on this
  // unit read up to ~50 px low.
  if (y < BAR_H + 48) {
    bool acted = true;
    if (barTabX[0] >= 0 && x >= barTabX[0] && x < barTabX[0] + barTabW[0]) switchMode(0);
    else if (barTabX[1] >= 0 && x >= barTabX[1] && x < barTabX[1] + barTabW[1]) switchMode(1);
    else if (barTabX[2] >= 0 && x >= barTabX[2] && x < barTabX[2] + barTabW[2]) switchMode(2);
    else if (x >= barListX && x < barListX + 39) { listOpen = true; listPage = 0; }
    else if (x >= 258 && x < 291) { if (zoomLevel > 0) zoomLevel--; clampCenter(); }              // v171N: [-]
    else if (x >= 292) { if (zoomLevel < maxZoomLevel()) zoomLevel++; clampCenter(); }            // v171N: [+]
    else acted = false;
    if (acted) {
      mapDirty = true;
      fullRedraw = true;
      return;
    }
    // S3 v6: a tap just below the bar that hit no control is a
    // bar-aimed tap whose read landed low - swallow it instead of
    // falling through to the object search, where it could re-hit
    // the selected ship near the top edge and open its detail
    // panel straight from a tab tap (user report).
    if (y < BAR_H + 12) return;
  }
  // nearest visible object of the current screen within 16 px
  int best = -1, bestD = 16 * 16;
  if (screenMode == 0) {
    for (int i = 0; i < MAX_STATIONS; i++) {
      if (!stations[i].used) continue;
      int px, py;
      if (!geoToPx(stations[i].lat, stations[i].lon, px, py)) continue;
      if (py < BAR_H + 4) continue;   // S3 v6: not drawn (under the bar) -> not tappable
      int d = (px - x) * (px - x) + (py - y) * (py - y);
      if (d < bestD) { bestD = d; best = i; }
    }
  }
  int bestAc = -1, bestAcD = 16 * 16;
  if (screenMode == 1) {
    for (int i = 0; i < acCount; i++) {
      int px, py;
      if (!geoToPx(aircraft[i].lat, aircraft[i].lon, px, py)) continue;
      if (py < BAR_H + 4) continue;   // S3 v6: not drawn (under the bar) -> not tappable
      int d = (px - x) * (px - x) + (py - y) * (py - y);
      if (d < bestAcD) { bestAcD = d; bestAc = i; }
    }
  }
  int bestShip = -1, bestShipD = 16 * 16;
  if (screenMode == 2) {
    for (int i = 0; i < MAX_SHIPS; i++) {
      if (!ships[i].used || !ships[i].hasPos) continue;
      int px, py;
      if (!geoToPx(ships[i].lat, ships[i].lon, px, py)) continue;
      if (py < BAR_H + 4) continue;   // S3 v6: not drawn (under the bar) -> not tappable
      int d = (px - x) * (px - x) + (py - y) * (py - y);
      if (d < bestShipD) { bestShipD = d; bestShip = i; }
    }
  }
  if (best >= 0) {
    if (best == selected) {
      detailIsAc = false;
      detailOpen = true;
    } else {
      selected = best;
      selAcFlight[0] = 0; detailIsAc = false;
      centerLat = stations[best].lat;
      centerLon = stations[best].lon;
      if (zoomLevel < maxZoomLevel()) zoomLevel++;
      clampCenter();
    }
  } else if (bestAc >= 0) {
    // aircraft: straight to the detail panel (flight + route)
    strncpy(selAcFlight, aircraft[bestAc].flight, sizeof(selAcFlight) - 1);
    selAcFlight[sizeof(selAcFlight) - 1] = 0;
    selected = -1;
    detailIsAc = true;
    detailOpen = true;
    ensureRoute(selAcFlight);   // ~1 s on first tap per flight
  } else if (bestShip >= 0) {
    // ships move slowly: station-style - first tap centres + zooms,
    // a second tap on the same ship opens the detail panel
    if (selShipMmsi != 0 && ships[bestShip].mmsi == selShipMmsi) {
      detailOpen = true;
    } else {
      selShipMmsi = ships[bestShip].mmsi;
      selected = -1; selAcFlight[0] = 0; detailIsAc = false;
      centerLat = ships[bestShip].lat;
      centerLon = ships[bestShip].lon;
      if (zoomLevel < maxZoomLevel()) zoomLevel++;
      clampCenter();
    }
  } else if (x < 48 || x >= 320 - 48 || y < BAR_H + 40 || y >= 240 - 48) {
    // pan: tap any screen edge to shift the view a quarter screen that way
    int dx = (x >= 320 - 48) ? 1 : (x < 48 ? -1 : 0);
    int dy = (y >= 240 - 48) ? -1 : (y < BAR_H + 40 ? 1 : 0);
    centerLon += dx * viewLonSpan() * 0.25;
    centerLat += dy * viewLatSpan() * 0.25;
    clampCenter();
  } else if (selected >= 0 || selShipMmsi != 0 || zoomLevel != defaultZoomLevel()) {
    selected = -1; selAcFlight[0] = 0; selShipMmsi = 0; detailIsAc = false;
    zoomLevel = defaultZoomLevel();
    centerLat = (MAP_LIST[curMap].north + MAP_LIST[curMap].south) / 2;
    centerLon = (MAP_LIST[curMap].east + MAP_LIST[curMap].west) / 2;
  }
  mapDirty = true;
  fullRedraw = true;
}

static uint32_t lastPollMs = 0;
static bool touchWasDown = false;
// v131: confirmed touch read. On the 024 the XPT2046 shares the
// display HSPI bus, and a single garbled SPI frame can report
// z >= 400 with railed coordinates - the device logged phantom
// presses at (319,219) with nobody touching it. A real finger is
// still down 6 ms later at nearly the same spot; a one-frame
// glitch is not. Returns 0 = no touch, 1 = confirmed press,
// 2 = glitch (caller leaves the press-edge state untouched).
static int readTouchConfirmed(int& mx, int& my) {
  // S3 port: LGFX touch device. getTouchRaw gives the raw ADC point
  // (and is PENIRQ-gated inside the driver); getTouch gives the
  // calibrated screen point, which on this panel needs sx = 319 - x
  // with y as-is (diag s3 v12/v17 corner proof).
  lgfx::touch_point_t p, q;
  if (!tft.getTouchRaw(&p, 1)) return 0;
  int32_t gx1, gy1;
  if (!tft.getTouch(&gx1, &gy1)) return 0;
  int x1 = constrain(319 - (int)gx1, 0, 319);
  int y1 = constrain((int)gy1, 0, 239);
  delay(6);
  if (!tft.getTouchRaw(&q, 1)) return 2;
  int32_t gx2, gy2;
  if (!tft.getTouch(&gx2, &gy2)) return 2;
  // PENIRQ (GPIO1) is active LOW when the panel is physically pressed.
  int irq = digitalRead(1);
  // v135: reject railed raw values. The phantom logs showed raw x
  // pinned at exactly 0 with irq LOW - the panel itself is
  // registering a faint edge contact (case/bezel pressure or a
  // failing film), not SPI garbage. A real touch can never reach
  // the rails: the electrodes end inside the glass (a hard corner
  // tap reads raw ~(430,3470) here; calibration window is
  // x 200..3700, y 240..3800). Railed samples are not fingers.
  bool railed = p.x < 120 || p.x > 3980 || p.y < 120 || p.y > 3980 ||
                q.x < 120 || q.x > 3980 || q.y < 120 || q.y > 3980;
  int x2 = constrain(319 - (int)gx2, 0, 319);
  int y2 = constrain((int)gy2, 0, 239);
  if (railed || abs(x2 - x1) > 24 || abs(y2 - y1) > 24) {
    // v136: the phantom contact on the user's unit turned out to
    // be CONTINUOUS (z hovering at the threshold, irq held LOW),
    // and the per-glitch print flooded the serial log. Count the
    // rejections and summarise at most once a minute instead.
    static uint32_t glitchCount = 0, lastGlitchLogMs = 0;
    glitchCount++;
    if (millis() - lastGlitchLogMs > 60000) {
      lastGlitchLogMs = millis();
      Serial.printf("touch glitch x%u suppressed (last raw %d,%d size %d irq %d)\n",
                    (unsigned)glitchCount, p.x, p.y, (int)p.size, irq);
      glitchCount = 0;
    }
    return 2;
  }
  mx = (x1 + x2) / 2;
  my = (y1 + y2) / 2;
  Serial.printf("touch ok raw %d,%d size %d / raw2 %d,%d size2 %d irq %d -> %d,%d\n",
                p.x, p.y, (int)p.size, q.x, q.y, (int)q.size, irq, mx, my);
  return 1;
}
// v112: mid-render press capture - same edge/debounce rules as
// touchLoop, but only records the tap; touchLoop fires it.
static void captureTouchDuringRender() {
  if (pendingTap) return;
  int mx, my;
  int r = readTouchConfirmed(mx, my);
  if (r == 0) { touchWasDown = false; return; }
  if (r != 1) return;
  if (touchWasDown) return;
  touchWasDown = true;
  uint32_t now = millis();
  if (now - lastTouchMs < 300) return;
  lastTouchMs = now;
  pendingTapX = mx;
  pendingTapY = my;
  pendingTap = true;
}
static void touchLoop() {
  if (pendingTap) {   // v112: a tap captured during a render fires now
    pendingTap = false;
    handleTouch(pendingTapX, pendingTapY);
    return;
  }
  uint32_t now = millis();
  if (now - lastPollMs < 40) return;           // poll at ~25 Hz
  lastPollMs = now;
  int mx, my;
  int tr = readTouchConfirmed(mx, my);        // v131: two-sample confirm
  if (tr == 0) { touchWasDown = false; return; }
  if (tr != 1) return;                         // glitch: keep edge state
  // One action per press (v73 fix): while the finger stays down, do not
  // fire handleTouch again. A held press used to re-fire after the
  // 300 ms debounce at the SAME screen spot - but the first fire had
  // already re-centred the view, so the second fire missed the station
  // and fell into the reset branch (or opened and instantly closed the
  // detail panel). That made the second tap on a station unusable.
  if (touchWasDown) return;
  touchWasDown = true;
  if (now - lastTouchMs < 300) return;         // action debounce
  lastTouchMs = now;
  handleTouch(mx, my);
}

// ---- S3 v24: on-demand LAN portal (BOOT long-press) ------------------
// Normal operation runs with the config server and mDNS stopped.
// Holding BOOT (GPIO0) for 2 s starts the portal and shows this
// screen; APRS/AIS reception keeps running behind it, touch is
// ignored while it is up. BOOT 2 s again closes it, and so does
// 5 min without a page access (handleRoot/save/mapgen refresh
// portalLastActivityMs). Saving or a map remake reboots as before.
static void drawRuntimePortalScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::Font2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 12);
  tft.print("SETUP PORTAL ON");
  tft.setFont(&fonts::Font0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 48);  tft.print("Open in browser:");
  tft.setCursor(22, 64);  tft.print("http://aprs-map.local");
  tft.setCursor(22, 80);  tft.printf("or http://%s", WiFi.localIP().toString().c_str());
  tft.setCursor(10, 110); tft.print("Hold BOOT 2s: close portal");
  tft.setCursor(10, 126); tft.print("Auto off after 5 min idle");
}

static void startRuntimePortal() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("setup portal: WiFi not connected, not starting");
    return;
  }
  startConfigServer();
  beginMdns();
  runtimePortalActive = true;
  drawRuntimePortalScreen();
  Serial.printf("setup portal: ON (http://aprs-map.local / http://%s)\n",
                WiFi.localIP().toString().c_str());
}

static void stopRuntimePortal() {
  stopConfigServer();
  endMdns();
  runtimePortalActive = false;
  fullRedraw = true;
  Serial.println("setup portal: OFF");
}

static void runtimePortalLoop() {
  static uint32_t bootLowStartMs = 0;
  static bool bootHandled = false;
  uint32_t now = millis();
  if (digitalRead(0) == LOW) {
    if (!bootLowStartMs) bootLowStartMs = now;
    if (!bootHandled && now - bootLowStartMs >= 2000) {
      bootHandled = true;
      if (runtimePortalActive) stopRuntimePortal();
      else startRuntimePortal();
    }
  } else {
    bootLowStartMs = 0;
    bootHandled = false;
  }
  if (runtimePortalActive) {
    if (WiFi.status() != WL_CONNECTED) stopRuntimePortal();
    else if (now - portalLastActivityMs > 300000) {
      Serial.println("setup portal: idle 5 min, auto off");
      stopRuntimePortal();
    }
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  // v162N: take the TLS reserve FIRST, while the heap is still
  // pristine. Taken at setup end (v130) it failed on this unit
  // ('tls reserve: FAILED') because the DMA pool was already
  // carved up; freed only around TLS handshakes, it gives them
  // a 56 KB contiguous DMA home. The v161N evidence: with the
  // AIS pause NOT actually freeing the stream client's TLS
  // buffers (fetch start dmaFree stays ~44 KB) the handshake
  // dies at a random stage (X509 / PK / SSL / NET_RECV).
  tlsReserveRetake();
  Serial.printf("tls reserve early: %s (%u bytes)\n",
                tlsReserve ? "ok" : "FAILED", (unsigned)tlsReserveBytes);
  tft.init();
  tft.setRotation(1);   // S3: 240x320 portrait-native panel -> landscape
  // S3: the module LED pin is wired straight to 3.3V (Q1 transistor),
  // so the backlight is always fully on - no PWM control exists.
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::Font2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 10);
  tft.print("APRS Map Viewer S3 v28");
  // v77 diagnostic: show WHY the previous boot ended, on the splash
  // itself and on serial. The user saw occasional reboots on station
  // taps with NOTHING on the serial port; a panic or watchdog would
  // print there, a silent reset points at power - this line settles
  // it without needing to capture serial at the right moment.
  {
    esp_reset_reason_t rr = esp_reset_reason();
    const char* rs = rr == ESP_RST_POWERON   ? "POWERON"   :
                     rr == ESP_RST_EXT       ? "EXT_PIN"   :
                     rr == ESP_RST_SW        ? "SW"        :
                     rr == ESP_RST_PANIC     ? "PANIC"     :
                     rr == ESP_RST_INT_WDT   ? "INT_WDT"   :
                     rr == ESP_RST_TASK_WDT  ? "TASK_WDT"  :
                     rr == ESP_RST_WDT       ? "WDT"       :
                     rr == ESP_RST_DEEPSLEEP ? "DEEPSLEEP" :
                     rr == ESP_RST_BROWNOUT  ? "BROWNOUT"  : "OTHER";
    Serial.printf("reset reason: %s (%d)\n", rs, (int)rr);
    tft.setFont(&fonts::Font0);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(10, 34);
    tft.printf("prev reset: %s", rs);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    // v107: after an ABNORMAL reset, hold the splash for 10 s with
    // the reason in large text - the small line was gone before the
    // user could read it (his report). Normal boots stay fast.
    if (rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT ||
        rr == ESP_RST_WDT || rr == ESP_RST_BROWNOUT) {
      tft.setFont(&fonts::Font2);
      tft.setTextSize(2);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setCursor(10, 70);
      tft.printf("RESET: %s", rs);
      tft.setTextSize(1);
      tft.setFont(&fonts::Font0);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      delay(10000);
    }
  }

  // S3: touch is part of the LGFX device (LGFX_S3.h) and was already
  // initialised by tft.init() - nothing to begin here.

  // maps are embedded in flash: fixed 640x480 geometry, always available
  for (int i = 0; i < MAP_COUNT; i++) {
    bmpInfo[i].ok = true;
    bmpInfo[i].w = MAP_W;
    bmpInfo[i].h = MAP_H;
    bmpInfo[i].topDown = true;
  }
  sdOk = true;   // legacy flag: now means "maps available"
  // (mode centres are initialised after mapGenBoot(), once the final
  // map bounds - baked or generated - are known)

  // Phone-based setup (Plane Radar style): settings live in NVS and
  // config.h is only the factory default. Holding BOOT for 2 s after
  // the splash wipes them and forces the setup portal + help screen.
  loadSettings(false);
  bool forcePortal = bootResetWindow();
  if (forcePortal) loadSettings(true);
  // v175N: if the startup screen is hidden by the portal's screen
  // selection, open on the first visible screen instead.
  // S3 v22: "first visible" follows the tab order FLIGHT, AIS, APRS.
  if (!((cfgScreens >> screenMode) & 1)) {
    static const int ORDER[3] = {1, 2, 0};
    for (int k = 0; k < MODE_COUNT; k++)
      if ((cfgScreens >> ORDER[k]) & 1) { screenMode = ORDER[k]; break; }
  }
  curMap = MODE_MAP[screenMode];
  wifiPhase(forcePortal);

  mapGenBoot();   // v79: activate generated maps, or generate them first
  if (!genMapsActive) {   // v103: the baked fallback is single-map too
    MAP_LIST[0] = MAP_LIST[1];
    mapGrayPtr[0] = MAP_GRAY[1];
    mapClassPtr[0] = MAP_CLASS[1];
  }
  if (!genColorActive)   // v127: colour mode needs no sea mask at
    buildSeaMask(0);       // all (v88: pre-build on a fresh heap)
  mgFreeGenBuffers();   // covers mapGenBoot's early returns
  centerLat = (MAP_LIST[0].north + MAP_LIST[0].south) / 2;   // v116: shared view
  centerLon = (MAP_LIST[0].east + MAP_LIST[0].west) / 2;
  zoomLevel = defaultZoomLevel();   // v171N: x0.5 at startup
  // (no frame sprite on this board - see the composition note at G)

  aprsSetup();
  // v104: ADS-B fetch task on Core 0 (see the aircraft globals).
  acMutex = xSemaphoreCreateMutex();
  chunkMutex = xSemaphoreCreateMutex();
  tlsReserveRetake();   // v130, v162N guarded
  Serial.printf("tls reserve: %s\n", tlsReserve ? "ok" : "FAILED");
  xTaskCreatePinnedToCore(acFetchTask, "acFetch", 24576, nullptr, 1, nullptr, 0);
  // S3 v5: the frame is composed into TWO permanent half buffers.
  // A single 240-row block (153,600 B) NEVER exists on this heap
  // (v4 boot log: largest DMA block 102,388 B - the same wall the
  // 028 hit in v37), but two halves totalling the same 153,600 B
  // fit into separate blocks. drawMap composes the whole frame
  // into them without pushing, then pushes both back-to-back, so
  // a tap redraw no longer sweeps the screen over the ~1 s
  // compose. Placement stays at the end of setup (the v3 lesson:
  // never take big blocks before Wi-Fi). If only one half fits,
  // it becomes a permanent single chunk; if none fit, drawMap
  // keeps its dynamic fallback.
  // S3 v11: the two-buffer compose is GONE. The AIS log of
  // 2026-11-20 showed the DMA pool fragmenting over reconnect
  // cycles until dmaLargest fell to 32,756 B and even the TLS
  // reserve retake FAILED - the AIS handshake could no longer
  // run at all. One permanent 120-row buffer frees 76,800 B for
  // good; the map compose falls back to band pushes.
  // S3 v18: 120 -> 60 rows. The reserve is structurally consumed
  // by the AIS session it enables (the session lives inside the
  // freed hole), so on this board the AMBIENT pool has to serve
  // the next handshake - and it was ~38 KB short of the 028's
  // working point. Another 38,400 B back: compose in 4 bands.
  {
    if (!mapChunk2) {
      mapChunk = (uint16_t*)heap_caps_malloc(320 * 60 * sizeof(uint16_t),
                                             MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
      if (mapChunk) {
        mapChunkRowsCap = 60;
        mapChunkPermanent = true;
        Serial.printf("frame buffer: permanent single 60 rows, largest DMA now %u\n",
                      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
      } else {
        Serial.printf("frame buffer: alloc FAILED (largest DMA %u) - dynamic fallback\n",
                      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
      }
    }
  }
  redraw();
}

static uint32_t lastBarMs = 0, lastPurgeMs = 0, lastSdRetryMs = 0, lastStationRedrawMs = 0;
static uint32_t lastMobileDrawMs = 0, lastMobileCheckMs = 0;
void loop() {
  if (cfgServerRunning) cfgServer.handleClient();   // S3 v24: only while a portal is up
  runtimePortalLoop();   // S3 v24: BOOT 2 s toggles the LAN portal
  aprsLoop();
  if (runtimePortalActive) { delay(10); return; }   // portal screen stays up; reception continues
  touchLoop();
  uint32_t now = millis();
  // S3 v15 (user idea): no ADS-B fetches while the FLIGHT screen
  // is not among the portal's selected screens.
  // S3 v16: and only while the FLIGHT screen is on display.
  if ((cfgScreens & 2) && screenMode == 1 && (!acFetchedOnce || now - lastAcFetchMs > acIntervalMs) && WiFi.status() == WL_CONNECTED) {
    lastAcFetchMs = now;
    acFetchedOnce = true;
    acFetchReq = true;   // v104: fetched on Core 0, touch stays live
  }
  // S3 v27: AIS ships come only from the Open Waters snapshot -
  // poll every 60 s while the AIS screen is shown and merge into
  // the ship table. No API key needed.
  if ((cfgScreens & 4) && screenMode == 2 && WiFi.status() == WL_CONNECTED) {
    if (now - aisLastSnapMs >= 60000) {
      aisLastSnapMs = now;
      aisSnapReq = true;   // fetched on Core 0, touch stays live
    }
  }
  if (acDataReady) { acDataReady = false; acDropStationary(); acTrailsUpdate(); stationsDirty = true; }   // S3 v20: trail points per fetch; S3 v21: stationary drop first
  // S3 v14: heap-death self-recovery. On this board the pool
  // fragments over TLS cycles until the largest block sits at
  // 32,756 B - 12 B below the smallest reserve size (32,768) -
  // and every handshake fails forever (2026-11-20 v13 log:
  // reserveBytes 0, 'tls reserve: FAILED' on every retry).
  // Nothing in-process can defragment that, so when the reserve
  // is gone AND no handshake-sized hole has existed for 10
  // straight minutes, reboot: the heap is pristine at boot and
  // everything reconnects. A healthy device never meets both
  // conditions at once.
  {
    static uint32_t heapDeadSince = 0;
    // S3 v27: the v17 aisOnline exclusion went with the stream;
    // with no persistent TLS session, reserve-gone + small hole +
    // 10 min without any TLS success is simply dead.
    bool heapDead = !tlsReserve &&
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) < 40960 &&
      now - lastTlsOkMs > 600000UL;
    if (heapDead && WiFi.status() == WL_CONNECTED) {
      if (!heapDeadSince) heapDeadSince = now;
      else if (now - heapDeadSince > 300000UL) {   // S3 v18: 10 -> 5 min
        Serial.println("heap dead (no reserve, largest < 40K for 10 min) - rebooting");
        delay(50);
        ESP.restart();
      }
    } else heapDeadSince = 0;
  }
  // NTP clock for the persisted station timestamps
  if (!ntpStarted && WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, "pool.ntp.org", "ntp.nict.jp");
    ntpStarted = true;
  }
  if (!stRestored && ntpStarted) {
    if (time(nullptr) > 1700000000) { restoreStations(); stRestored = true; }
    else if (now > 150000) stRestored = true;   // no NTP: skip restore
  }
  if (stPersistDirty && now - lastPersistMs > 20000) {
    lastPersistMs = now;
    stPersistDirty = false;
    saveStations();
  }
  if (now - lastPurgeMs > 30000) {
    lastPurgeMs = now;
    bool changed = false;
    time_t nowT = time(nullptr);
    bool unixOk = nowT > 1700000000;
    for (int i = 0; i < MAX_STATIONS; i++) {
      if (!stations[i].used) continue;
      bool expired = (unixOk && stations[i].lastUnix)
          ? ((uint32_t)nowT - stations[i].lastUnix > (uint32_t)STATION_EXPIRE_MIN * 60)
          : (now - stations[i].lastMs > (uint32_t)STATION_EXPIRE_MIN * 60000UL);
      if (expired) {
        stations[i].used = false;
        if (selected == i) { selected = -1; detailOpen = false; }
        changed = true;
      }
    }
    for (int i = 0; i < MAX_SHIPS; i++) {
      if (!ships[i].used) continue;
      // ships go quiet for good once they leave the area / switch off
      bool expired = !ships[i].hasPos || (now - ships[i].lastMs > 30UL * 60000UL);
      if (expired) {
        if (selShipMmsi == ships[i].mmsi) { selShipMmsi = 0; detailOpen = false; }
        ships[i].used = false;
        changed = true;
      }
    }
    if (changed) { stationsDirty = true; stPersistDirty = true; }
  }
  if (fullRedraw) { redraw(); return; }
  if (stationsDirty && !detailOpen && now - lastStationRedrawMs > 3000) {
    lastStationRedrawMs = now;   // batch station updates: check at most 1x / 3 s
    if (stationsOnScreenChanged()) {
      if (screenMode == 0) lastMobileDrawMs = now;
      redraw(); return;
    }
    stationsDirty = false;       // nothing to show: skip the redraw
  }
  if (stationsDirty && detailOpen) stationsDirty = false;  // don't disturb the panel
  // mobile stations on the APRS screen: repaint once a minute
  if (screenMode == 0 && !detailOpen && now - lastMobileCheckMs > 5000) {
    lastMobileCheckMs = now;
    if (now - lastMobileDrawMs > 60000 && (mobileOnScreenMoved() || trailRepaintNeeded())) {
      lastMobileDrawMs = now;
      redraw(); return;
    }
  }
  if (now - lastBarMs > 5000) { lastBarMs = now; if (!detailOpen) drawBarF(false, true); }
  // tap marker: shown/hidden through composed redraws (no direct draw)
  bool markerNow = (tapX >= 0 && now - tapMs < 500);
  if (markerNow != tapShown) {
    if (tapShown) markerRepair = true;
    tapShown = markerNow;
    fullRedraw = true;
  }
  delay(10);
}
