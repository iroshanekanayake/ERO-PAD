// ==============================================================================
//  OctaPad v2.24 [ERO PAD OS] — Web-based Wireless Sound Manager & Full Settings
//  Dynamic 8-Pad / 10-Pad Layout Select Engine + WebSocket Live Visualizer
//  ESP32-S3 + PCM5102A/UDA1334A + SH1106 OLED
//  [PRODUCTION v2.24.0 - Kit Chains + anti-retrigger fix + round-robin tones + quick-set crosstalk]
// ==============================================================================

#include <Arduino.h>
#include <SD_MMC.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "driver/i2s.h"
#include "USB.h"
#include "USBMIDI.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

USBMIDI MIDI; 
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// === Pin Definitions ===
constexpr uint8_t PAD1_PIN   = 1;
constexpr uint8_t PAD2_PIN   = 2;
constexpr uint8_t PAD3_PIN   = 10;
constexpr uint8_t PAD4_PIN   = 11;
constexpr uint8_t PAD5_PIN   = 6;
constexpr uint8_t PAD6_PIN   = 7;
constexpr uint8_t PAD7_PIN   = 8;
constexpr uint8_t PAD8_PIN   = 9;
constexpr uint8_t PAD9_PIN   = 13;
constexpr uint8_t PAD10_PIN  = 14;

const uint8_t PAD_PINS[10] = { PAD1_PIN, PAD2_PIN, PAD3_PIN, PAD4_PIN, PAD5_PIN, PAD6_PIN, PAD7_PIN, PAD8_PIN, PAD9_PIN, PAD10_PIN };
const uint8_t PAD_MIDI_NOTES[10] = { 36, 38, 42, 46, 48, 45, 49, 51, 53, 55 };
const uint8_t MIDI_CHANNEL = 10; 

// === I2S DAC ===
constexpr uint8_t I2S_BCLK_PIN = 15;
constexpr uint8_t I2S_WSEL_PIN = 16;
constexpr uint8_t I2S_DOUT_PIN = 17;
constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

// === SD CARD ===
constexpr uint8_t SD_MMC_CLK_PIN  = 39;
constexpr uint8_t SD_MMC_CMD_PIN  = 38;
constexpr uint8_t SD_MMC_DATA_PIN = 40;

// === OLED Module ===
constexpr uint8_t OLED_SDA_PIN = 21;
constexpr uint8_t OLED_SCL_PIN = 18;

// === Rotary Encoder & Buttons ===
constexpr uint8_t ENC_A_PIN       = 41;
constexpr uint8_t ENC_B_PIN       = 42;
constexpr uint8_t ENC_BTN_PIN     = 45;
constexpr uint8_t BTN_BACK_PIN    = 48;
constexpr uint8_t BTN_KIT_UP_PIN   = 3;
constexpr uint8_t BTN_KIT_DOWN_PIN = 46;

// === SYSTEM SETTINGS ===
enum SysMode { MODE_SAMPLER, MODE_MIDI_OUT, MODE_HYBRID };
enum MenuScreen { 
    SCREEN_MAIN, 
    SCREEN_MENU_LIST, 
    SCREEN_PAD_SENS_SEL,
    SCREEN_PAD_SENS_SET,
    SCREEN_SET_VOL, 
    SCREEN_SET_VELOCITY, 
    SCREEN_SET_MODE, 
    SCREEN_EDIT_PAD_SEL, 
    SCREEN_EDIT_LAYER_SEL, 
    SCREEN_EDIT_WAV_SEL,
    SCREEN_PAD_VOL_SEL,   
    SCREEN_PAD_VOL_SET,
    SCREEN_SET_LAYOUT,
    SCREEN_ADV_PAD_SEL,
    SCREEN_ADV_TUNE_LIST,
    SCREEN_SET_DEBOUNCE, 
    SCREEN_SET_PEAKWIN,   
    SCREEN_SET_CROSSTALK,
    SCREEN_SET_CROSSTALK_LEVEL,
    SCREEN_PAD_FILT_SEL,   
    SCREEN_PAD_FILT_SET,
    SCREEN_PAD_PITCH_SEL,
    SCREEN_PAD_PITCH_SET,
    SCREEN_PAD_PAN_SEL,
    SCREEN_PAD_PAN_SET,
    SCREEN_CHAIN_SEL,
    SCREEN_CHAIN_EDIT,
    SCREEN_CHAIN_MODE,
    SCREEN_WEB_SERVER     
};

constexpr uint32_t SAMPLE_RATE        = 44100;
constexpr uint8_t  I2S_DMA_BUF_COUNT  = 8;
constexpr uint16_t I2S_DMA_BUF_LEN    = 512;
constexpr uint16_t AUDIO_MIX_SAMPLES  = 256;
constexpr uint8_t  MAX_VOICES         = 16;
constexpr uint8_t  MAX_PADS_POSSIBLE  = 10;

#define MY_NAME   "ERO PAD" 

constexpr uint8_t  MIN_KIT            = 1;
constexpr uint16_t MAX_KITS           = 90;
constexpr uint32_t MAX_WAV_SIZE_BYTES = 350 * 1024;
constexpr uint16_t MAX_TONE_INDEX     = 2000;

constexpr uint8_t  MAX_CHAINS         = 8;
constexpr uint8_t  MAX_CHAIN_STEPS    = 16;

// === GLOBAL DATA ARCHITECTURE ===
SemaphoreHandle_t settingsMutex = nullptr;
SemaphoreHandle_t sdMutex = nullptr;

uint16_t padThreshold[MAX_PADS_POSSIBLE] = {150,150,150,150,150,150,150,150,150,150};
float    globalMasterVolume = 0.8f; 
uint8_t  globalFixedVelocity = 124; 
bool     globalDynamicVelocity = false;

uint8_t  kitChains[MAX_CHAINS + 1][MAX_CHAIN_STEPS] = {0};
bool     globalChainModeActive = false;
uint8_t  globalActiveChain = 1;
int8_t   globalChainStepIndex = 0;
SysMode  globalSystemMode = MODE_HYBRID; 
uint8_t  globalPadCount = 8; 

uint16_t padDebounceMs[MAX_PADS_POSSIBLE]     = {100,100,100,100,100,100,100,100,100,100};
uint16_t padPeakWindowMs[MAX_PADS_POSSIBLE]   = {10,10,10,10,10,10,10,10,10,10};
uint16_t padCrosstalkLockMs[MAX_PADS_POSSIBLE] = {4,4,4,4,4,4,4,4,4,4};

constexpr uint8_t CROSSTALK_MATRIX_DEFAULT_PERCENT = 30;
uint8_t crosstalkMatrix[MAX_PADS_POSSIBLE][MAX_PADS_POSSIBLE];

void initDefaultCrosstalkMatrix() {
    for (int i = 0; i < MAX_PADS_POSSIBLE; i++) {
        for (int j = 0; j < MAX_PADS_POSSIBLE; j++) {
            crosstalkMatrix[i][j] = (i == j) ? 0 : CROSSTALK_MATRIX_DEFAULT_PERCENT;
        }
    }
}

uint16_t globalPadLastVelocity[MAX_PADS_POSSIBLE] = {0}; 
uint32_t globalPadHitTime[MAX_PADS_POSSIBLE] = {0};
uint32_t globalMidiNoteOffTime[MAX_PADS_POSSIBLE] = {0};

volatile bool webServerActive = false; 

struct PadLayerAssignment {
    uint16_t layerA_ToneNum; 
    uint16_t layerB_ToneNum; 
    uint8_t  padVolumePercentA; 
    uint8_t  padVolumePercentB; 
    int8_t   padFilterA; 
    int8_t   padFilterB; 
    int8_t   padPitchA;
    int8_t   padPitchB;
    int8_t   padPanA;
    int8_t   padPanB;
    uint8_t  roundRobinEnabled;
    uint16_t rrToneNum[3];
};
PadLayerAssignment kitAssignments[MAX_KITS + 1][MAX_PADS_POSSIBLE];

constexpr size_t KIT_NAME_MAXLEN = 11;
char kitNames[MAX_KITS + 1][KIT_NAME_MAXLEN + 1];
void loadKitNamesFromSD();
void saveKitNamesToSD();
void loadChainsFromSD();
void saveChainsToSD();
int  stepChain(int8_t direction);

String getKitDisplayName(int kitNum) {
    if (kitNum >= MIN_KIT && kitNum <= (int)MAX_KITS && kitNames[kitNum][0] != '\0') return String(kitNames[kitNum]);
    char buf[10]; snprintf(buf, sizeof(buf), "KIT %02d", kitNum); return String(buf);
}

uint8_t calculateDynamicVelocity(uint16_t peak, uint16_t threshold) {
    constexpr uint16_t ADC_CEILING = 4095;
    constexpr uint8_t MIN_VELOCITY = 20; 
    constexpr uint8_t MAX_VELOCITY = 127;
    if (peak <= threshold || threshold >= ADC_CEILING) return MIN_VELOCITY;
    float normalized = (float)(peak - threshold) / (float)(ADC_CEILING - threshold);
    normalized = constrain(normalized, 0.0f, 1.0f);
    float curved = sqrtf(normalized);
    int vel = MIN_VELOCITY + (int)roundf(curved * (float)(MAX_VELOCITY - MIN_VELOCITY));
    return (uint8_t)constrain(vel, 1, 127);
}

#define SYS_CONFIG_VERSION 5
struct SystemConfig {
    uint32_t configVersion;
    uint16_t padThreshold[MAX_PADS_POSSIBLE];
    uint16_t padDebounceMs[MAX_PADS_POSSIBLE];
    uint16_t padPeakWindowMs[MAX_PADS_POSSIBLE];
    uint16_t padCrosstalkLockMs[MAX_PADS_POSSIBLE];
    uint8_t  crosstalkMatrix[MAX_PADS_POSSIBLE][MAX_PADS_POSSIBLE];
    float    masterVolume;
    uint8_t  systemMode;
    uint8_t  fixedVelocity;    
    uint8_t  dynamicVelocity;
};

void saveKitMappingsToSD();
void loadKitMappingsFromSD();
void startWirelessHost();
void stopWirelessHost();
void broadcastWsEvent(const char* type, String payload);

// === AUDIO VOICE ENGINE ===
struct WAVInfo {
    uint32_t sampleRate; uint16_t bitsPerSample; uint16_t numChannels; uint32_t dataOffset; uint32_t dataSize; bool valid;
};

struct PadSampleData {
    uint8_t* pcm = nullptr;
    uint32_t dataSize = 0;
    uint16_t numChannels = 1;
    uint16_t bitsPerSample = 16;
    uint16_t toneNum = 0;
    bool     valid = false;
};

struct Voice {
    const uint8_t* pcm;
    uint32_t dataSize;
    uint16_t numChannels;
    uint16_t bitsPerSample;
    float    padVolFactor;
    int8_t   filterVal;
    float    lastFilteredSampleL;
    float    lastFilteredSampleR;
    float    framePos;
    float    pitchRatio;
    uint32_t totalFrames;
    uint32_t bytesPerFrame;
    float    panGainL;
    float    panGainR;
    bool     active;
};

inline int16_t softClip(int32_t s) {
    if (s > 32767) return 32767;
    if (s < -32768) return -32768;
    return (int16_t)s;
}

class AudioEngine {
public:
    AudioEngine() {
        _currentKit = 1; _ready = false; _voiceMutex = nullptr;
        _previewVoiceIdx = -1;
        for (int i = 0; i < MAX_VOICES; i++) { _voices[i].active = false; _voices[i].pcm = nullptr; }
    }

    bool begin() {
        _voiceMutex = xSemaphoreCreateMutex();
        if (!_voiceMutex) return false;
        if (!_initI2S()) return false;
        xTaskCreatePinnedToCore(audioTask, "AudioTask", 8192, this, 5, nullptr, 0);
        return true;
    }

    bool loadKit(int kitNumber) {
        if (kitNumber < MIN_KIT || kitNumber > MAX_KITS) return false;
        _ready = false;

        if (xSemaphoreTake(_voiceMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            for (int i = 0; i < MAX_VOICES; i++) _deactivateVoice(_voices[i]);
            xSemaphoreGive(_voiceMutex);
        }

        for (int pad = 0; pad < MAX_PADS_POSSIBLE; pad++) {
            if (_padInfoA[pad].pcm) { free(_padInfoA[pad].pcm); _padInfoA[pad].pcm = nullptr; }
            if (_padInfoB[pad].pcm) { free(_padInfoB[pad].pcm); _padInfoB[pad].pcm = nullptr; }
            for (int s = 0; s < 3; s++) {
                if (_padInfoRR[pad][s].pcm) { free(_padInfoRR[pad][s].pcm); _padInfoRR[pad][s].pcm = nullptr; }
            }
            _rrIndex[pad] = 0;
        }

        uint8_t activePads = 8;
        if (xSemaphoreTake(settingsMutex, pdMS_TO_TICKS(10)) == pdTRUE) { activePads = globalPadCount; xSemaphoreGive(settingsMutex); }

        for (int pad = 0; pad < activePads; pad++) {
            uint16_t tA = kitAssignments[kitNumber][pad].layerA_ToneNum;
            _padInfoA[pad] = PadSampleData{};
            if (tA > 0) {
                char path[30]; snprintf(path, sizeof(path), "/SAMPLES/%04d.WAV", tA);
                if (_loadWAVFile(path, _padInfoA[pad])) { _padInfoA[pad].toneNum = tA; _padInfoA[pad].valid = true; }
            }
            uint16_t tB = kitAssignments[kitNumber][pad].layerB_ToneNum;
            _padInfoB[pad] = PadSampleData{};
            if (tB > 0) {
                char path[30]; snprintf(path, sizeof(path), "/SAMPLES/%04d.WAV", tB);
                if (_loadWAVFile(path, _padInfoB[pad])) { _padInfoB[pad].toneNum = tB; _padInfoB[pad].valid = true; }
            }
            if (kitAssignments[kitNumber][pad].roundRobinEnabled) {
                for (int s = 0; s < 3; s++) {
                    uint16_t tRR = kitAssignments[kitNumber][pad].rrToneNum[s];
                    _padInfoRR[pad][s] = PadSampleData{};
                    if (tRR > 0) {
                        char path[30]; snprintf(path, sizeof(path), "/SAMPLES/%04d.WAV", tRR);
                        if (_loadWAVFile(path, _padInfoRR[pad][s])) { _padInfoRR[pad][s].toneNum = tRR; _padInfoRR[pad][s].valid = true; }
                    }
                }
            }
        }
        _currentKit = kitNumber; _ready = true; return true;
    }

    void triggerPad(uint8_t padIndex) {
        if (webServerActive) return; 
        uint8_t activePads = 8;
        if (xSemaphoreTake(settingsMutex, 0) == pdTRUE) { activePads = globalPadCount; xSemaphoreGive(settingsMutex); }
        if (padIndex >= activePads || !_ready) return;
        if (xSemaphoreTake(_voiceMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;

        float padGainA = (float)kitAssignments[_currentKit][padIndex].padVolumePercentA / 100.0f;
        float padGainB = (float)kitAssignments[_currentKit][padIndex].padVolumePercentB / 100.0f;
        int8_t filtA = kitAssignments[_currentKit][padIndex].padFilterA;
        int8_t filtB = kitAssignments[_currentKit][padIndex].padFilterB;
        int8_t pitchA = kitAssignments[_currentKit][padIndex].padPitchA;
        int8_t pitchB = kitAssignments[_currentKit][padIndex].padPitchB;
        int8_t panA = kitAssignments[_currentKit][padIndex].padPanA;
        int8_t panB = kitAssignments[_currentKit][padIndex].padPanB;

        if (kitAssignments[_currentKit][padIndex].roundRobinEnabled) {
            bool played = false;
            for (int tries = 0; tries < 3; tries++) {
                uint8_t slot = _rrIndex[padIndex];
                _rrIndex[padIndex] = (_rrIndex[padIndex] + 1) % 3;
                PadSampleData &rr = _padInfoRR[padIndex][slot];
                if (rr.valid && rr.pcm && rr.dataSize > 0) {
                    _startVoice(rr, padGainA, filtA, pitchA, panA);
                    played = true;
                    break;
                }
            }
            if (played) {
                xSemaphoreGive(_voiceMutex);
                return;
            }
            // Round-robin enabled වුවද valid sample එකක් නොමැති නම් Layer A/B වෙත Fallback වේ.
        }

        if (_padInfoA[padIndex].valid && _padInfoA[padIndex].pcm && _padInfoA[padIndex].dataSize > 0)
            _startVoice(_padInfoA[padIndex], padGainA, filtA, pitchA, panA);
        if (_padInfoB[padIndex].valid && _padInfoB[padIndex].pcm && _padInfoB[padIndex].dataSize > 0)
            _startVoice(_padInfoB[padIndex], padGainB, filtB, pitchB, panB);

        xSemaphoreGive(_voiceMutex);
    }

    void playPreviewWAV(uint16_t toneNum, int8_t currentFiltVal, int8_t pitchVal = 0, int8_t panVal = 0) {
        if (toneNum == 0) return;
        char path[30]; snprintf(path, sizeof(path), "/SAMPLES/%04d.WAV", toneNum);
        PadSampleData info;
        if (!_loadWAVFile(path, info)) return;
        info.toneNum = toneNum; info.valid = true;

        if (xSemaphoreTake(_voiceMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (_previewVoiceIdx >= 0 && _previewVoiceIdx < MAX_VOICES) {
                _deactivateVoice(_voices[_previewVoiceIdx]);
            }
            if (_previewSample.pcm) { free(_previewSample.pcm); _previewSample.pcm = nullptr; }
            _previewSample = info;
            _previewVoiceIdx = _startVoice(_previewSample, 1.0f, currentFiltVal, pitchVal, panVal);
            xSemaphoreGive(_voiceMutex);
        }
    }

    static void audioTask(void* param) {
        AudioEngine* eng = (AudioEngine*)param;
        static int16_t mixBuf[AUDIO_MIX_SAMPLES * 2];
        size_t written = 0;
        for (;;) {
            eng->_mixVoices(mixBuf, AUDIO_MIX_SAMPLES);
            i2s_write(I2S_PORT, mixBuf, sizeof(mixBuf), &written, portMAX_DELAY);
        }
    }

private:
    Voice    _voices[MAX_VOICES];
    PadSampleData _previewSample;
    PadSampleData _padInfoA[MAX_PADS_POSSIBLE];
    PadSampleData _padInfoB[MAX_PADS_POSSIBLE];
    PadSampleData _padInfoRR[MAX_PADS_POSSIBLE][3];
    uint8_t _rrIndex[MAX_PADS_POSSIBLE] = {0};
    int      _previewVoiceIdx;
    volatile int _currentKit; volatile bool _ready; SemaphoreHandle_t _voiceMutex;

    void _deactivateVoice(Voice& v) {
        v.active = false; v.framePos = 0.0f;
    }

    int _startVoice(const PadSampleData& info, float padVol, int8_t filterValue, int8_t pitchSemitones = 0, int8_t panValue = 0) {
        if (!info.pcm || info.dataSize == 0) return -1;
        int slot = -1;
        for (int i = 0; i < MAX_VOICES; i++) { if (!_voices[i].active) { slot = i; break; } }
        if (slot < 0) {
            uint32_t maxPct = 0; int oldest = 0;
            for (int i = 0; i < MAX_VOICES; i++) {
                uint32_t pct = _voices[i].totalFrames > 0 ? (uint32_t)(_voices[i].framePos * 100.0f / _voices[i].totalFrames) : 0;
                if (pct > maxPct) { maxPct = pct; oldest = i; }
            }
            slot = oldest;
        }
        Voice& v = _voices[slot];
        v.pcm = info.pcm; v.dataSize = info.dataSize;
        v.numChannels = info.numChannels; v.bitsPerSample = info.bitsPerSample;
        v.padVolFactor = padVol; v.filterVal = filterValue;
        v.lastFilteredSampleL = 0.0f; v.lastFilteredSampleR = 0.0f;

        v.bytesPerFrame = (info.bitsPerSample == 16 ? 2u : 1u) * (info.numChannels == 2 ? 2u : 1u);
        v.totalFrames = (v.bytesPerFrame > 0) ? (info.dataSize / v.bytesPerFrame) : 0;
        v.framePos = 0.0f;
        v.pitchRatio = powf(2.0f, constrain((int)pitchSemitones, -12, 12) / 12.0f);

        float panNorm = constrain((int)panValue, -100, 100) / 100.0f;
        v.panGainL = (panNorm >= 0.0f) ? (1.0f - panNorm) : 1.0f;
        v.panGainR = (panNorm <= 0.0f) ? (1.0f + panNorm) : 1.0f;

        v.active = (v.totalFrames > 1);
        return slot;
    }

    bool _initI2S() {
        i2s_config_t cfg = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = SAMPLE_RATE, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
            .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = I2S_DMA_BUF_COUNT, .dma_buf_len = I2S_DMA_BUF_LEN, .use_apll = true, .tx_desc_auto_clear = true, .fixed_mclk = 0
        };
        i2s_pin_config_t pins = { .mck_io_num = I2S_PIN_NO_CHANGE, .bck_io_num = I2S_BCLK_PIN, .ws_io_num = I2S_WSEL_PIN, .data_out_num = I2S_DOUT_PIN, .data_in_num = I2S_PIN_NO_CHANGE };
        if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) return false;
        i2s_set_pin(I2S_PORT, &pins); i2s_zero_dma_buffer(I2S_PORT); return true;
    }

    WAVInfo _parseWAV(uint8_t* data, uint32_t fileSize) {
        WAVInfo info = {}; if (memcmp(data, "RIFF", 4) != 0 || memcmp(data + 8, "WAVE", 4) != 0) return info;
        uint32_t offset = 12;
        while (offset + 8 <= fileSize && offset < 1024) {
            char id[5] = {}; memcpy(id, data + offset, 4); uint32_t sz = *(uint32_t*)(data + offset + 4);
            if (strcmp(id, "fmt ") == 0) {
                info.numChannels = *(uint16_t*)(data + offset + 10); info.sampleRate = *(uint32_t*)(data + offset + 12); info.bitsPerSample = *(uint16_t*)(data + offset + 22);
            } else if (strcmp(id, "data") == 0) {
                info.dataOffset = offset + 8; info.dataSize = sz; info.valid = (info.sampleRate > 0); return info;
            }
            offset += 8 + sz; if (sz & 1) offset++;
        }
        return info;
    }

    bool _loadWAVFile(const char* path, PadSampleData& outInfo) {
        outInfo = PadSampleData{};
        bool ok = false;
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            if (SD_MMC.exists(path)) {
                File f = SD_MMC.open(path, FILE_READ);
                if (f) {
                    uint32_t fileSize = f.size();
                    uint8_t header[1024];
                    size_t headLen = (fileSize > 1024) ? 1024 : fileSize;
                    size_t headRead = f.read(header, headLen);
                    if (headRead == headLen) {
                        WAVInfo info = _parseWAV(header, (uint32_t)headRead);
                        if (info.valid && info.dataSize > 0 && info.dataSize <= MAX_WAV_SIZE_BYTES) {
                            uint8_t* buf = (uint8_t*)ps_malloc(info.dataSize);
                            if (buf) {
                                if (f.seek(info.dataOffset)) {
                                    size_t got = f.read(buf, info.dataSize);
                                    if (got == info.dataSize) {
                                        outInfo.pcm = buf; outInfo.dataSize = info.dataSize;
                                        outInfo.numChannels = info.numChannels; outInfo.bitsPerSample = info.bitsPerSample;
                                        ok = true;
                                    } else {
                                        free(buf);
                                        Serial.printf("[Audio] Short read loading sample: %s\n", path);
                                    }
                                } else {
                                    free(buf);
                                }
                            } else {
                                Serial.printf("[Audio] PSRAM alloc failed for sample: %s\n", path);
                            }
                        }
                    }
                    f.close();
                }
            } else {
                Serial.printf("[Audio] Missing sample: %s\n", path);
            }
            xSemaphoreGive(sdMutex);
        }
        return ok;
    }

    void _mixVoices(int16_t* out, int stereoSamples) {
        memset(out, 0, stereoSamples * 2 * sizeof(int16_t));
        SysMode currentMode = MODE_HYBRID; float currentVol = 0.8f;
        if (xSemaphoreTake(settingsMutex, 0) == pdTRUE) { currentMode = globalSystemMode; currentVol = globalMasterVolume; xSemaphoreGive(settingsMutex); }
        if (currentMode == MODE_MIDI_OUT) return;
        if (xSemaphoreTake(_voiceMutex, 0) != pdTRUE) return;

        for (int v = 0; v < MAX_VOICES; v++) {
            Voice& voice = _voices[v]; if (!voice.active) continue;

            for (int s = 0; s < stereoSamples; s++) {
                uint32_t frame0 = (uint32_t)voice.framePos;
                if (frame0 + 1 >= voice.totalFrames) {
                    _deactivateVoice(voice);
                    break;
                }
                float frac = voice.framePos - (float)frame0;

                int16_t l0, r0, l1, r1;
                _readFrame(voice, frame0, l0, r0);
                _readFrame(voice, frame0 + 1, l1, r1);
                float fL = l0 + ((float)(l1 - l0) * frac);
                float fR = r0 + ((float)(r1 - r0) * frac);

                int16_t sampleL, sampleR;
                if (voice.filterVal < 0) { 
                    float alpha = 1.0f - (abs(voice.filterVal) * 0.018f);
                    alpha = constrain(alpha, 0.05f, 0.95f);
                    float filteredL = (alpha * fL) + ((1.0f - alpha) * voice.lastFilteredSampleL);
                    float filteredR = (alpha * fR) + ((1.0f - alpha) * voice.lastFilteredSampleR);
                    voice.lastFilteredSampleL = filteredL; voice.lastFilteredSampleR = filteredR;
                    sampleL = (int16_t)filteredL; sampleR = (int16_t)filteredR;
                } 
                else if (voice.filterVal > 0) { 
                    float alpha = voice.filterVal * 0.015f;
                    alpha = constrain(alpha, 0.02f, 0.85f);
                    float lpfL = ((1.0f - alpha) * fL) + (alpha * voice.lastFilteredSampleL);
                    float lpfR = ((1.0f - alpha) * fR) + (alpha * voice.lastFilteredSampleR);
                    voice.lastFilteredSampleL = lpfL; voice.lastFilteredSampleR = lpfR;
                    sampleL = (int16_t)(fL - lpfL); sampleR = (int16_t)(fR - lpfR);
                }
                else {
                    sampleL = (int16_t)fL; sampleR = (int16_t)fR;
                }

                int32_t scaledL = (int32_t)(sampleL * currentVol * voice.padVolFactor * voice.panGainL);
                int32_t scaledR = (int32_t)(sampleR * currentVol * voice.padVolFactor * voice.panGainR);
                out[s*2]   = softClip((int32_t)out[s*2]   + scaledL);
                out[s*2+1] = softClip((int32_t)out[s*2+1] + scaledR);

                voice.framePos += voice.pitchRatio;
            }
        }
        xSemaphoreGive(_voiceMutex);
    }

    inline void _readFrame(const Voice& v, uint32_t frameIndex, int16_t& outL, int16_t& outR) {
        uint32_t byteOffset = frameIndex * v.bytesPerFrame;
        if (v.bitsPerSample == 16) {
            uint8_t loL = v.pcm[byteOffset], hiL = v.pcm[byteOffset + 1];
            outL = (int16_t)((hiL << 8) | loL);
            if (v.numChannels == 2) {
                uint8_t loR = v.pcm[byteOffset + 2], hiR = v.pcm[byteOffset + 3];
                outR = (int16_t)((hiR << 8) | loR);
            } else {
                outR = outL;
            }
        } else {
            outL = ((int16_t)v.pcm[byteOffset] - 128) * 256;
            outR = (v.numChannels == 2) ? (((int16_t)v.pcm[byteOffset + 1] - 128) * 256) : outL;
        }
    }
};

extern AudioEngine audioEngine; 

// === PERSISTENT STORAGE MEMORY MANAGEMENT ===
void loadKitMappingsFromSD() {
    xSemaphoreTake(settingsMutex, portMAX_DELAY);
    initDefaultCrosstalkMatrix();
    for (int k = 1; k <= MAX_KITS; k++) {
        for (int p = 0; p < MAX_PADS_POSSIBLE; p++) {
            kitAssignments[k][p].layerA_ToneNum = p + 1; 
            kitAssignments[k][p].layerB_ToneNum = 0; 
            kitAssignments[k][p].padVolumePercentA = 100; 
            kitAssignments[k][p].padVolumePercentB = 100; 
            kitAssignments[k][p].padFilterA = 0; 
            kitAssignments[k][p].padFilterB = 0; 
            kitAssignments[k][p].padPitchA = 0;
            kitAssignments[k][p].padPitchB = 0;
            kitAssignments[k][p].padPanA = 0;
            kitAssignments[k][p].padPanB = 0;
            kitAssignments[k][p].roundRobinEnabled = 0;
            kitAssignments[k][p].rrToneNum[0] = 0;
            kitAssignments[k][p].rrToneNum[1] = 0;
            kitAssignments[k][p].rrToneNum[2] = 0;
        }
    }
    
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
        if (SD_MMC.exists("/LAYOUT_CONFIG.DAT")) {
            File lf = SD_MMC.open("/LAYOUT_CONFIG.DAT", FILE_READ);
            if (lf) { lf.read((uint8_t*)&globalPadCount, sizeof(globalPadCount)); lf.close(); }
        }

        if (SD_MMC.exists("/SYS_CONFIG.DAT")) {
            File sf = SD_MMC.open("/SYS_CONFIG.DAT", FILE_READ);
            if (sf) {
                SystemConfig sc;
                size_t bytesRead = sf.read((uint8_t*)&sc, sizeof(sc));
                if (bytesRead >= sizeof(SystemConfig) && sc.configVersion == SYS_CONFIG_VERSION) {
                    for (int p = 0; p < MAX_PADS_POSSIBLE; p++) {
                        padThreshold[p] = constrain((int)sc.padThreshold[p], 100, 4095);
                        padDebounceMs[p] = constrain((int)sc.padDebounceMs[p], 10, 300);
                        padPeakWindowMs[p] = constrain((int)sc.padPeakWindowMs[p], 2, 30);
                        padCrosstalkLockMs[p] = constrain((int)sc.padCrosstalkLockMs[p], 2, 150);
                    }
                    for (int i = 0; i < MAX_PADS_POSSIBLE; i++)
                        for (int j = 0; j < MAX_PADS_POSSIBLE; j++)
                            crosstalkMatrix[i][j] = (i == j) ? 0 : constrain((int)sc.crosstalkMatrix[i][j], 0, 100);
                    globalMasterVolume = sc.masterVolume;
                    globalSystemMode = (SysMode)sc.systemMode;
                    globalFixedVelocity = sc.fixedVelocity; 
                    globalDynamicVelocity = (sc.dynamicVelocity != 0);
                }
                sf.close();
            }
        }

        if (SD_MMC.exists("/KITS_V23.DAT")) {
            File f = SD_MMC.open("/KITS_V23.DAT", FILE_READ); 
            if (f) { f.read((uint8_t*)kitAssignments, sizeof(kitAssignments)); f.close(); }
        }
        xSemaphoreGive(sdMutex);
    }
    xSemaphoreGive(settingsMutex);
}

void saveKitMappingsToSD() {
    xSemaphoreTake(settingsMutex, portMAX_DELAY);
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
    File f = SD_MMC.open("/KITS_V23.DAT", FILE_WRITE);
    if (f) { f.write((uint8_t*)kitAssignments, sizeof(kitAssignments)); f.close(); }
    
    File lf = SD_MMC.open("/LAYOUT_CONFIG.DAT", FILE_WRITE);
    if (lf) { lf.write((uint8_t*)&globalPadCount, sizeof(globalPadCount)); lf.close(); }

    File sf = SD_MMC.open("/SYS_CONFIG.DAT", FILE_WRITE);
    if (sf) {
        SystemConfig sc;
        sc.configVersion = SYS_CONFIG_VERSION;
        for (int p = 0; p < MAX_PADS_POSSIBLE; p++) {
            sc.padThreshold[p] = padThreshold[p];
            sc.padDebounceMs[p] = padDebounceMs[p];
            sc.padPeakWindowMs[p] = padPeakWindowMs[p];
            sc.padCrosstalkLockMs[p] = padCrosstalkLockMs[p];
        }
        for (int i = 0; i < MAX_PADS_POSSIBLE; i++)
            for (int j = 0; j < MAX_PADS_POSSIBLE; j++)
                sc.crosstalkMatrix[i][j] = crosstalkMatrix[i][j];
        sc.masterVolume = globalMasterVolume;
        sc.systemMode = (uint8_t)globalSystemMode;
        sc.fixedVelocity = globalFixedVelocity; 
        sc.dynamicVelocity = globalDynamicVelocity ? 1 : 0;
        sf.write((uint8_t*)&sc, sizeof(sc));
        sf.close();
    }
    xSemaphoreGive(sdMutex);
    }
    xSemaphoreGive(settingsMutex);
}

void loadKitNamesFromSD() {
    for (int k = 0; k <= (int)MAX_KITS; k++) kitNames[k][0] = '\0';
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
        if (SD_MMC.exists("/KITNAMES.DAT")) {
            File f = SD_MMC.open("/KITNAMES.DAT", FILE_READ);
            if (f) { f.read((uint8_t*)kitNames, sizeof(kitNames)); f.close(); }
        }
        xSemaphoreGive(sdMutex);
    }
    for (int k = 0; k <= (int)MAX_KITS; k++) kitNames[k][KIT_NAME_MAXLEN] = '\0';
}
void saveKitNamesToSD() {
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
        File f = SD_MMC.open("/KITNAMES.DAT", FILE_WRITE);
        if (f) { f.write((uint8_t*)kitNames, sizeof(kitNames)); f.close(); }
        xSemaphoreGive(sdMutex);
    }
}

void loadChainsFromSD() {
    memset(kitChains, 0, sizeof(kitChains));
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
        if (SD_MMC.exists("/CHAINS.DAT")) {
            File f = SD_MMC.open("/CHAINS.DAT", FILE_READ);
            if (f) { f.read((uint8_t*)kitChains, sizeof(kitChains)); f.close(); }
        }
        xSemaphoreGive(sdMutex);
    }
}
void saveChainsToSD() {
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
        File f = SD_MMC.open("/CHAINS.DAT", FILE_WRITE);
        if (f) { f.write((uint8_t*)kitChains, sizeof(kitChains)); f.close(); }
        xSemaphoreGive(sdMutex);
    }
}

int stepChain(int8_t direction) {
    uint8_t chain = constrain((int)globalActiveChain, 1, (int)MAX_CHAINS);
    int validSteps[MAX_CHAIN_STEPS]; int count = 0;
    for (int s = 0; s < MAX_CHAIN_STEPS; s++) {
        if (kitChains[chain][s] != 0) validSteps[count++] = s;
    }
    if (count == 0) return -1;
    int curPos = 0;
    for (int i = 0; i < count; i++) { if (validSteps[i] == globalChainStepIndex) { curPos = i; break; } }
    curPos = ((curPos + direction) % count + count) % count;
    globalChainStepIndex = validSteps[curPos];
    return kitChains[chain][globalChainStepIndex];
}

// === PIEZO HANDLER ===
struct PadState { 
    bool hit; 
    uint16_t peakValue; 
    uint32_t lastHitTime; 
    bool waitingPeak; 
    uint16_t peakTracked; 
    uint32_t peakStartMs; 
    uint16_t lastHitPeak;
};

constexpr uint32_t HARD_MASK_MS = 3;
constexpr float ADAPTIVE_CEILING_CAP_MULTIPLIER = 2.5f;
constexpr float ADAPTIVE_DECAY_FLOOR_PERCENT = 0.35f;

class PiezoHandler {
public:
    void begin() { 
        for (int i = 0; i < MAX_PADS_POSSIBLE; i++) { 
            analogSetPinAttenuation(PAD_PINS[i], ADC_11db); 
            _pads[i] = {}; 
        } 
    }
    
    void scan() {
        uint32_t now = millis(); 
        uint16_t localThreshold[MAX_PADS_POSSIBLE];
        uint16_t localDebounce[MAX_PADS_POSSIBLE];
        uint16_t localPeakWin[MAX_PADS_POSSIBLE];
        uint16_t localCrosstalk[MAX_PADS_POSSIBLE];
        static uint8_t localMatrix[MAX_PADS_POSSIBLE][MAX_PADS_POSSIBLE];
        uint8_t activePads = 8;
        
        if (xSemaphoreTake(settingsMutex, 0) == pdTRUE) { 
            for (int p = 0; p < MAX_PADS_POSSIBLE; p++) {
                localThreshold[p] = padThreshold[p];
                localDebounce[p] = padDebounceMs[p];
                localPeakWin[p] = padPeakWindowMs[p];
                localCrosstalk[p] = padCrosstalkLockMs[p];
            }
            memcpy(localMatrix, crosstalkMatrix, sizeof(localMatrix));
            activePads = globalPadCount; 
            xSemaphoreGive(settingsMutex); 
        } else {
            for (int p = 0; p < MAX_PADS_POSSIBLE; p++) {
                localThreshold[p] = 600; localDebounce[p] = 100; localPeakWin[p] = 10; localCrosstalk[p] = 4;
            }
        }
        
        for (int i = 0; i < activePads; i++) { 
            uint16_t val = analogRead(PAD_PINS[i]); 
            _pads[i].hit = false;
            
            uint16_t currentDebounce = localDebounce[i];
            uint16_t currentPeakWin = localPeakWin[i];
            uint32_t sinceLastHit = now - _pads[i].lastHitTime;

            if (_pads[i].lastHitTime > 0 && sinceLastHit < HARD_MASK_MS) continue;

            uint16_t th = localThreshold[i];
            if (_pads[i].lastHitTime > 0 && sinceLastHit < currentDebounce && _pads[i].lastHitPeak > th) {
                uint16_t cappedStartPeak = min((uint32_t)_pads[i].lastHitPeak, (uint32_t)(th * ADAPTIVE_CEILING_CAP_MULTIPLIER));
                uint16_t decayFloor = max((uint32_t)th, (uint32_t)(cappedStartPeak * ADAPTIVE_DECAY_FLOOR_PERCENT));
                float decayProgress = (float)sinceLastHit / (float)currentDebounce;
                uint16_t adaptiveCeiling = decayFloor + (uint16_t)((cappedStartPeak - decayFloor) * (1.0f - decayProgress));
                if (adaptiveCeiling > th) th = adaptiveCeiling;
            }
            
            if (!_pads[i].waitingPeak) {
                if (val >= th) { 
                    _pads[i].waitingPeak = true; 
                    _pads[i].peakTracked = val; 
                    _pads[i].peakStartMs = now; 
                }
            } else {
                if (val > _pads[i].peakTracked) _pads[i].peakTracked = val;
                
                if ((now - _pads[i].peakStartMs) >= currentPeakWin) { 
                    _pads[i].peakValue = _pads[i].peakTracked; 
                    _pads[i].waitingPeak = false; 
                    _pads[i].peakTracked = 0;
                    
                    uint16_t myCrosstalkWindow = localCrosstalk[i];
                    uint16_t maxPeakInWindow = 0; int aggressor = -1;
                    for (int p = 0; p < activePads; p++) {
                        if (p != i && (now - _pads[p].lastHitTime) < myCrosstalkWindow) {
                            if (_pads[p].peakValue > maxPeakInWindow) {
                                maxPeakInWindow = _pads[p].peakValue;
                                aggressor = p;
                            }
                        }
                    }
                    
                    bool looksLikeWeakBleed = false;
                    if (aggressor >= 0) {
                        uint8_t pct = localMatrix[i][aggressor];
                        if (pct > 0) {
                            float ratio = pct / 100.0f;
                            looksLikeWeakBleed = (float)_pads[i].peakValue < ratio * (float)maxPeakInWindow;
                        }
                    }

                    if (looksLikeWeakBleed) {
                        _pads[i].peakValue = 0; 
                    } else {
                        _pads[i].hit = true; 
                        _pads[i].lastHitTime = now; 
                        _pads[i].lastHitPeak = _pads[i].peakValue;
                    }
                }
            }
        }
    }
    
    bool wasHit(uint8_t pad) { return pad < MAX_PADS_POSSIBLE ? _pads[pad].hit : false; }
    uint16_t getPeak(uint8_t pad) { return pad < MAX_PADS_POSSIBLE ? _pads[pad].peakValue : 0; }
    
private:
    PadState _pads[MAX_PADS_POSSIBLE]; 
};

// === INTERFACE GRAPHICS ENGINE ===
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);

const char index_html[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" type="image/svg+xml" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 64 64'%3E%3Crect width='64' height='64' rx='13' fill='%230e0f13'/%3E%3Cpath d='M15 14h34v8H24v7h21v8H24v7h25v8H15z' fill='%236fb3ff'/%3E%3C/svg%3E">
<meta name="theme-color" content="#0e0f13">
<title>ERO PAD OS v2.24</title>
<style>
    :root {
        --bg: #0e0f13;
        --panel: #171922;
        --panel2: #1e212c;
        --border: #2a2d3a;
        --accent: #6fb3ff;
        --accent2: #ffb347;
        --accent-grad: linear-gradient(135deg, #6fb3ff, #9ecfff);
        --blue: #3ea6ff;
        --green: #33d17a;
        --red: #ff5c5c;
        --purple: #a86bff;
        --text: #eef0f5;
        --text-dim: #8b8fa3;
    }
    * { box-sizing: border-box; }
    body {
        font-family: 'Segoe UI', system-ui, Arial, sans-serif;
        background: radial-gradient(circle at top, #191b24 0%, #0b0c10 70%);
        color: var(--text);
        margin: 0;
        padding: 0;
        min-height: 100vh;
    }
    .app { max-width: 720px; margin: 0 auto; padding: 18px 16px 40px; }
    .hero {
        text-align: center;
        padding: 22px 10px 18px;
    }
    .hero h1 {
        margin: 0;
        font-size: 26px;
        font-weight: 800;
        letter-spacing: 0.5px;
        background: var(--accent-grad);
        -webkit-background-clip: text;
        background-clip: text;
        color: transparent;
    }
    .hero p { margin: 4px 0 0; color: var(--text-dim); font-size: 13px; }

    .tabs {
        display: flex;
        gap: 6px;
        background: var(--panel);
        border: 1px solid var(--border);
        border-radius: 14px;
        padding: 6px;
        margin-bottom: 18px;
        overflow-x: auto;
    }
    .tab-btn {
        flex: 1 0 auto;
        white-space: nowrap;
        background: transparent;
        color: var(--text-dim);
        border: none;
        padding: 10px 12px;
        border-radius: 10px;
        font-size: 13px;
        font-weight: 600;
        cursor: pointer;
        transition: 0.2s;
    }
    .tab-btn.active {
        background: var(--accent-grad);
        color: #1a1200;
    }

    .tab-panel { display: none; animation: fadein 0.25s ease; }
    .tab-panel.active { display: block; }
    @keyframes fadein { from { opacity: 0; transform: translateY(4px); } to { opacity: 1; transform: translateY(0); } }

    .card {
        background: var(--panel);
        border: 1px solid var(--border);
        border-radius: 16px;
        padding: 18px;
        margin-bottom: 16px;
        box-shadow: 0 8px 24px rgba(0,0,0,0.25);
    }
    .card h2 {
        font-size: 15px;
        margin: 0 0 14px;
        color: var(--accent2);
        display: flex;
        align-items: center;
        gap: 8px;
    }
    .card h2 .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--accent-grad); display: inline-block; }

    canvas { background: #0b0c10; border-radius: 12px; border: 1px solid var(--border); width: 100%; height: 130px; display: block; }

    .btn {
        background: var(--accent-grad);
        color: #1a1200;
        border: none;
        padding: 12px 18px;
        font-size: 14px;
        font-weight: 700;
        border-radius: 10px;
        cursor: pointer;
        width: 100%;
        transition: 0.15s;
    }
    .btn:active { transform: scale(0.98); }
    .btn-outline {
        background: transparent;
        color: var(--text);
        border: 1px solid var(--border);
    }
    .btn-blue { background: var(--blue); color: #06202f; }
    .btn-green { background: var(--green); color: #063017; }
    .btn-purple { background: var(--purple); color: #1b0838; }
    .btn-red { background: var(--red); color: #350404; }
    .btn-sm { padding: 7px 10px; font-size: 12px; width: auto; border-radius: 8px; }

    .file-input { display: none; }
    .status { margin-top: 12px; font-weight: 600; font-size: 13px; text-align: center; min-height: 16px; }

    .settings-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .field { display: flex; flex-direction: column; gap: 5px; }
    .checkbox-field { flex-direction: row; align-items: center; gap: 8px; }
    .checkbox-field label { display: flex; align-items: center; gap: 8px; font-size: 13px; }
    .checkbox-field input[type="checkbox"] { width: 18px; height: 18px; flex-shrink: 0; }
    .field label { font-size: 11px; color: var(--text-dim); font-weight: 600; text-transform: uppercase; letter-spacing: 0.4px; }
    .field input, .field select {
        background: var(--panel2);
        border: 1px solid var(--border);
        color: var(--text);
        padding: 9px 10px;
        border-radius: 9px;
        font-size: 14px;
        width: 100%;
    }
    .field input:focus, .field select:focus { outline: none; border-color: var(--accent); }

    .pad-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 12px; }
    .pad-card { background: var(--panel2); border: 1px solid var(--border); border-radius: 12px; padding: 12px; }
    .pad-card h4 {
        margin: 0 0 10px; font-size: 13px; color: var(--accent2);
        border-bottom: 1px solid var(--border); padding-bottom: 6px;
        display: flex; justify-content: space-between; align-items: center;
    }
    .layer-label { font-size: 10px; color: var(--blue); font-weight: 700; text-transform: uppercase; margin: 8px 0 4px; letter-spacing: 0.5px; }
    .mini-row { display: flex; gap: 6px; margin-bottom: 6px; }
    .mini-row .field { flex: 1; }
    .mini-row input { padding: 6px 8px; font-size: 12px; }
    .mini-row label { font-size: 9px; }

    .file-row {
        display: flex; align-items: center; justify-content: space-between;
        background: var(--panel2); border: 1px solid var(--border);
        border-radius: 10px; padding: 10px 12px; margin-bottom: 8px;
    }
    .file-row .fname { font-size: 13px; font-weight: 600; }
    .file-row .fsize { font-size: 11px; color: var(--text-dim); margin-left: 8px; }
    .file-empty { text-align: center; color: var(--text-dim); font-size: 13px; padding: 20px 0; }

    .footer { text-align: center; font-size: 11px; color: var(--text-dim); margin-top: 20px; }
    .connection-panel {
        background: linear-gradient(145deg, rgba(30,33,44,.98), rgba(19,21,30,.98));
        border: 1px solid var(--border);
        border-radius: 14px;
        padding: 13px 14px;
        margin: 0 0 18px;
        box-shadow: 0 6px 18px rgba(0,0,0,.18);
    }
    .connection-main { display:flex; align-items:center; gap:10px; }
    .connection-led { width:11px; height:11px; border-radius:50%; flex:0 0 11px; background:#8b8fa3; box-shadow:0 0 0 4px rgba(139,143,163,.12); }
    .connection-led.online { background:#33d17a; box-shadow:0 0 0 4px rgba(51,209,122,.13); }
    .connection-led.waiting { background:#ffb347; box-shadow:0 0 0 4px rgba(255,179,71,.13); }
    .connection-led.offline { background:#ff5c5c; box-shadow:0 0 0 4px rgba(255,92,92,.13); }
    .connection-copy { min-width:0; flex:1; }
    .connection-title { font-size:14px; font-weight:800; letter-spacing:.2px; color:var(--text); }
    .connection-detail { font-size:11px; line-height:1.45; color:var(--text-dim); margin-top:3px; }
    .connection-badge { font-size:10px; font-weight:800; letter-spacing:.5px; padding:6px 9px; border-radius:999px; background:#292c37; color:var(--text-dim); white-space:nowrap; }
    .connection-badge.online { background:rgba(51,209,122,.14); color:#33d17a; }
    .connection-badge.waiting { background:rgba(255,179,71,.14); color:#ffb347; }
    .connection-badge.offline { background:rgba(255,92,92,.14); color:#ff7777; }
    .connection-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:7px; margin-top:12px; }
    .connection-item { background:rgba(10,12,18,.42); border:1px solid var(--border); border-radius:9px; padding:8px; min-width:0; }
    .connection-item span { display:block; font-size:9px; color:var(--text-dim); line-height:1.3; }
    .connection-item strong { display:block; margin-top:4px; font-size:11px; color:var(--text); overflow-wrap:anywhere; }
    .connection-action { display:none; margin-top:11px; }
    .connection-action.visible { display:block; }
    @media (max-width:420px) {
        .connection-grid { grid-template-columns:1fr; }
        .connection-item { display:flex; justify-content:space-between; align-items:center; gap:8px; }
        .connection-item strong { margin-top:0; text-align:right; }
    }

</style>
</head>
<body>
<div class="app">
    <div class="hero">
        <h1>ERO PAD OS</h1>
        <p>v2.24 — Wireless Sound & Settings Center</p>
    </div>

    <section class="connection-panel" id="connectionPanel" aria-live="polite">
        <div class="connection-main">
            <span class="connection-led waiting" id="connectionLed"></span>
            <div class="connection-copy">
                <div class="connection-title" id="connectionTitle">CHECKING ESP32 CONNECTION...</div>
                <div class="connection-detail" id="connectionDetail">Checking the board and live control link.</div>
            </div>
            <span class="connection-badge waiting" id="connectionBadge">CHECKING</span>
        </div>
        <div class="connection-grid">
            <div class="connection-item"><span>PHONE ↔ BOARD</span><strong id="phoneBoardState">Checking</strong></div>
            <div class="connection-item"><span>ESP32 API</span><strong id="apiConnectionState">Checking</strong></div>
            <div class="connection-item"><span>LIVE WEBSOCKET</span><strong id="wsConnectionState">Checking</strong></div>
        </div>
        <div class="connection-action" id="connectionAction">
            <button class="btn btn-blue" type="button" onclick="openEsp32ControlPage()">OPEN ESP32 CONTROL (192.168.4.1)</button>
        </div>
    </section>

    <div class="tabs">
        <button class="tab-btn active" onclick="showTab('dash')">Dashboard</button>
        <button class="tab-btn" onclick="showTab('settings')">Settings</button>
        <button class="tab-btn" onclick="showTab('sounds')">Sounds</button>
        <button class="tab-btn" onclick="showTab('files')">Files</button>
        <button class="tab-btn" onclick="showTab('backup')">Backup</button>
    </div>

    <!-- DASHBOARD TAB -->
    <div id="tab-dash" class="tab-panel active">
        <div class="card">
            <h2><span class="dot"></span>Kit Chains (Live)</h2>
            <p style="font-size:12px; color:var(--text-dim); margin-top:-6px;">Step through a saved kit playlist -- works like a wireless footswitch. When Chain Mode is active, the hardware Kit Up/Down buttons step the chain too, instead of the raw kit number.</p>
            <div class="mini-row">
                <div class="field"><label>Chain (1-8)</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="chainNumSelector" min="1" max="8" value="1" onchange="loadChainForEditing()"></div>
                <div class="field checkbox-field"><label><input type="checkbox" id="chainModeToggle" onchange="toggleChainMode()"> Chain Mode Active</label></div>
            </div>
            <div class="field"><label>Steps (kit numbers, in order, comma-separated)</label><input type="text" inputmode="numeric" id="chainStepsInput" placeholder="1,5,12,20"></div>
            <div style="margin-top:8px; display:flex; gap:8px;">
                <button class="btn btn-outline" onclick="loadChainForEditing()">Reload</button>
                <button class="btn btn-green" onclick="saveChainSteps()">Save Steps</button>
            </div>
            <div style="margin-top:14px; display:flex; gap:8px;">
                <button class="btn btn-blue" style="flex:1;" onclick="chainStep(-1)">&#9664; Prev Kit</button>
                <button class="btn btn-blue" style="flex:1;" onclick="chainStep(1)">Next Kit &#9654;</button>
            </div>
            <div id="chainStatus" class="status"></div>
        </div>
        <div class="card">
            <h2><span class="dot"></span>Live Piezo Visualizer</h2>
            <canvas id="visCanvas"></canvas>
        </div>
    </div>

    <!-- SETTINGS TAB -->
    <div id="tab-settings" class="tab-panel">
        <div class="card">
            <h2><span class="dot"></span>Pad Calibration (per-pad)</h2>
            <p style="font-size:12px; color:var(--text-dim); margin-top:-6px;">Threshold, Debounce, Peak Window and Crosstalk Lock are all tunable per pad — helpful for roll/bounce technique on some pads and crosstalk isolation on others.</p>
            <div class="pad-grid" id="calibration-container"></div>
        </div>
        <div class="card">
            <h2><span class="dot"></span>System Settings</h2>
            <div class="settings-grid">
                <div class="field"><label>Master Volume %</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="s_masterVolume" min="0" max="100"></div>
                <div class="field"><label>Fixed MIDI Velocity</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="s_fixedVelocity" min="1" max="127"></div>
                <div class="field checkbox-field"><label><input type="checkbox" id="s_dynamicVelocity"> Dynamic Velocity (peak-driven, ignores the fixed number above)</label></div>
                <div class="field"><label>System Mode</label>
                    <select id="s_systemMode">
                        <option value="0">Sampler (SD)</option>
                        <option value="1">MIDI Out (USB)</option>
                        <option value="2">Hybrid</option>
                    </select>
                </div>
                <div class="field"><label>Pad Layout</label>
                    <select id="s_padCount"><option value="8">8 Pads</option><option value="10">10 Pads</option></select>
                </div>
            </div>
            <div style="margin-top:14px;"><button class="btn" onclick="saveSettings()">Save System Settings</button></div>
            <div id="settingsStatus" class="status"></div>
        </div>
        <div class="card">
            <h2><span class="dot"></span>Crosstalk Matrix (Trigger Engine)</h2>
            <p style="font-size:12px; color:var(--text-dim); margin-top:-6px;">Row = the pad being checked, column = the other pad it's compared against. Value = how big a % of the column pad's peak this row pad needs to reach, within its Crosstalk Lock window, to count as a genuine hit instead of bleed-through. 0 = never reject (use for pads that never bleed into each other on your hardware). Hit pads one at a time and watch the Live Visualizer on the Dashboard tab to find which pairs actually bleed -- distance on the pad layout does NOT reliably predict this on a rigid/plastic base.</p>
            <div class="card" style="background:var(--panel2); margin:14px 0; padding:14px;">
                <h3 style="margin:0 0 8px; color:var(--accent); font-size:14px;">Roland-Style Crosstalk Cancel Level</h3>
                <p style="font-size:12px; color:var(--text-dim); margin:0 0 12px;">Choose one pad and apply a 50%, 60%, 70% or 80% level to that pad's matrix row. This matches the OLED Crosstalk Level control. Other pads' rows are not changed.</p>
                <div class="settings-grid" style="align-items:end;">
                    <div class="field">
                        <label for="ctLevelPad">Select Pad</label>
                        <select id="ctLevelPad"></select>
                    </div>
                    <div class="field">
                        <label for="ctLevelValue">Crosstalk Level</label>
                        <select id="ctLevelValue">
                            <option value="50">50%</option>
                            <option value="60">60%</option>
                            <option value="70">70%</option>
                            <option value="80">80%</option>
                        </select>
                    </div>
                </div>
                <div style="margin-top:12px;">
                    <button class="btn btn-blue" onclick="applyCrosstalkLevelToSelectedPad()">Apply Level to Selected Pad & Save</button>
                </div>
                <div id="ctLevelStatus" class="status"></div>
            </div>
            <div style="display:flex; align-items:center; gap:10px; flex-wrap:wrap; margin:12px 0;">
                <label style="font-size:12px; color:var(--text-dim);">QUICK SET (30-100%, applies to every box below)</label>
                <input type="number" inputmode="numeric" pattern="[0-9]*" id="ctmQuickValue" min="30" max="100" value="30" style="width:70px; text-align:center; background:#0b0c10; border:1px solid var(--border); border-radius:6px; color:var(--text); padding:8px 4px;">
                <button class="btn" onclick="applyCrosstalkToAll()">Set All Pads & Save</button>
            </div>
            <div id="matrix-container" style="overflow-x:auto;"></div>
            <div style="margin-top:14px; display:flex; gap:10px; flex-wrap:wrap;">
                <button class="btn" onclick="saveCrosstalkMatrix()">Save Crosstalk Matrix</button>
                <button class="btn btn-outline" onclick="resetCrosstalkMatrix()">Reset All to 30%</button>
            </div>
            <div id="matrixStatus" class="status"></div>
        </div>
    </div>

    <!-- SOUNDS TAB -->
    <div id="tab-sounds" class="tab-panel">
        <div class="card">
            <h2><span class="dot"></span>Kit Selector</h2>
            <p style="font-size:12px; color:var(--text-dim); margin-top:-6px;">The Pad Sound Matrix below always edits ONE kit -- whichever number is loaded here. This also switches the kit playing live on the device (same as pressing the hardware Kit Up/Down buttons).</p>
            <div class="mini-row">
                <div class="field"><label>Kit Number (1-90)</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="kitSelector" min="1" max="90" value="1"></div>
                <button class="btn btn-blue" onclick="loadKitFromSelector()" style="align-self:flex-end;">Load Kit</button>
            </div>
            <div class="mini-row" style="margin-top:10px;">
                <div class="field"><label>Kit Name (optional, max 11 chars)</label><input type="text" id="kitNameInput" maxlength="11" placeholder="e.g. Congo, Dholki, Bongo"></div>
                <button class="btn btn-outline" onclick="saveKitName()" style="align-self:flex-end;">Save Name</button>
            </div>
            <div id="kitSelectorStatus" class="status"></div>
        </div>

        <div class="card">
            <h2><span class="dot"></span>Upload WAV Sample</h2>
            <form id="uploadForm">
                <input class="file-input" type="file" id="file" name="file" accept=".wav" onchange="uploadFileBtn()">
                <button type="button" class="btn btn-blue" onclick="document.getElementById('file').click()">Select WAV File</button>
            </form>
            <div id="uploadStatus" class="status" style="color:var(--text-dim);">No file uploading...</div>
        </div>

        <div class="card">
            <h2><span class="dot"></span>Pad Sound Matrix</h2>
            <p style="font-size:12px; color:var(--text-dim); margin-top:-6px;">Tap a tone or filter field to hear it instantly on your phone. Tone fields show a pick-list of the WAV files actually on your SD card -- start typing or tap the field to see it.</p>
            <div class="pad-grid" id="pads-container"></div>
            <datalist id="sdToneList"></datalist>
            <div style="margin-top:16px;"><button class="btn btn-green" onclick="saveAssignments()">Save All Assignments</button></div>
            <div id="saveStatus" class="status"></div>
        </div>
    </div>

    <!-- FILES TAB -->
    <div id="tab-files" class="tab-panel">
        <div class="card">
            <h2><span class="dot"></span>SD Card Samples</h2>
            <div style="margin-bottom:12px;"><button class="btn btn-outline" onclick="loadFileList()">Refresh List</button></div>
            <div id="fileListContainer"><div class="file-empty">Loading...</div></div>
            <div id="fileStatus" class="status"></div>
        </div>
    </div>

    <!-- BACKUP TAB -->
    <div id="tab-backup" class="tab-panel">
        <div class="card">
            <h2><span class="dot"></span>JSON Config Management</h2>
            <button type="button" class="btn btn-blue" onclick="window.location.href='/backup-json'">Download JSON Backup</button>
            <div style="margin-top:12px;">
                <input class="file-input" type="file" id="jsonFile" accept=".json" onchange="restoreJson()">
                <button type="button" class="btn btn-purple" onclick="document.getElementById('jsonFile').click()">Restore JSON Config</button>
            </div>
            <div id="jsonStatus" class="status"></div>
        </div>
    </div>

    <p class="footer">ERO Pad OS v2.24 — ESP32-S3 Wireless Engine</p>
</div>

<script>
    function showTab(name) {
        document.querySelectorAll('.tab-panel').forEach(el => el.classList.remove('active'));
        document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));
        document.getElementById('tab-' + name).classList.add('active');
        event.currentTarget.classList.add('active');
        if (name === 'files') loadFileList();
        if (name === 'sounds') populateSdToneList();
    }

    let calContainer = document.getElementById('calibration-container');
    for (let i = 1; i <= 10; i++) {
        calContainer.innerHTML += `
            <div class="pad-card">
                <h4>PAD ${i}</h4>
                <div class="mini-row">
                    <div class="field"><label>Threshold</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="p${i}th" min="200" max="2000"></div>
                    <div class="field"><label>Debounce ms</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="p${i}db" min="10" max="300"></div>
                </div>
                <div class="mini-row">
                    <div class="field"><label>Peak Win ms</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="p${i}pw" min="2" max="30"></div>
                    <div class="field"><label>Crosstalk ms</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="p${i}ct" min="5" max="150"></div>
                </div>
            </div>
        `;
    }

    let container = document.getElementById('pads-container');
    for (let i = 1; i <= 10; i++) {
        container.innerHTML += `
            <div class="pad-card">
                <h4>PAD ${i}</h4>
                <div class="layer-label">Layer A</div>
                <div class="mini-row">
                    <div class="field"><label>Tone</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="p${i}a" min="0" max="2000" placeholder="0" list="sdToneList" onchange="previewToneFromPad(${i}, 'a')"></div>
                    <div class="field"><label>Vol %</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="p${i}va" min="0" max="100" placeholder="100"></div>
                    <div class="field"><label>Filt</label><input type="number" inputmode="numeric" id="p${i}fa" min="-50" max="50" placeholder="0" onchange="previewToneFromPad(${i}, 'a')"></div>
                </div>
                <div class="layer-label">Layer B</div>
                <div class="mini-row">
                    <div class="field"><label>Tone</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="p${i}b" min="0" max="2000" placeholder="0" list="sdToneList" onchange="previewToneFromPad(${i}, 'b')"></div>
                    <div class="field"><label>Vol %</label><input type="number" inputmode="numeric" pattern="[0-9]*" id="p${i}vb" min="0" max="100" placeholder="100"></div>
                    <div class="field"><label>Filt</label><input type="number" inputmode="numeric" id="p${i}fb" min="-50" max="50" placeholder="0" onchange="previewToneFromPad(${i}, 'b')"></div>
                </div>
            </div>
        `;
    }

    const ESP32_HOST = "192.168.4.1";
    const isEsp32ControlPage = (location.hostname === ESP32_HOST);
    let ws = null;
    let wsIsConnected = false;
    let apiIsConnected = false;
    let wsReconnectTimer = null;
    let apiMonitorTimer = null;

    function setConnectionText(id, value) {
        const el = document.getElementById(id);
        if (el) el.textContent = value;
    }

    function updateConnectionPanel() {
        const led = document.getElementById("connectionLed");
        const badge = document.getElementById("connectionBadge");
        const title = document.getElementById("connectionTitle");
        const detail = document.getElementById("connectionDetail");
        const action = document.getElementById("connectionAction");
        if (!led || !badge || !title || !detail) return;

        led.className = "connection-led";
        badge.className = "connection-badge";
        if (!isEsp32ControlPage) {
            led.classList.add("waiting");
            badge.classList.add("waiting");
            title.textContent = "GITHUB PAGES PREVIEW";
            detail.textContent = "This HTTPS page cannot directly control the ESP32 HTTP/Wi-Fi service. Connect to ERO_PAD_WIFI, then open the board's local address.";
            badge.textContent = "PREVIEW";
            setConnectionText("phoneBoardState", "Not verified");
            setConnectionText("apiConnectionState", "Not connected");
            setConnectionText("wsConnectionState", "Not connected");
            if (action) action.classList.add("visible");
            return;
        }

        if (action) action.classList.remove("visible");
        setConnectionText("phoneBoardState", apiIsConnected ? "Board reachable" : "Not reachable");
        setConnectionText("apiConnectionState", apiIsConnected ? "ONLINE" : "OFFLINE");
        setConnectionText("wsConnectionState", wsIsConnected ? "CONNECTED" : "DISCONNECTED");

        if (apiIsConnected && wsIsConnected) {
            led.classList.add("online");
            badge.classList.add("online");
            title.textContent = "ESP32 BOARD CONNECTED";
            detail.textContent = "Your phone can reach ERO_PAD_WIFI and the live WebSocket link is active.";
            badge.textContent = "ONLINE";
        } else if (apiIsConnected || wsIsConnected) {
            led.classList.add("waiting");
            badge.classList.add("waiting");
            title.textContent = "ESP32 FOUND — CONNECTING LIVE LINK";
            detail.textContent = "The board is reachable. Waiting for both the API and live WebSocket connection.";
            badge.textContent = "CONNECTING";
        } else {
            led.classList.add("offline");
            badge.classList.add("offline");
            title.textContent = "ESP32 BOARD NOT REACHABLE";
            detail.textContent = "Connect your phone to ERO_PAD_WIFI (password: 12345678) and open http://192.168.4.1/.";
            badge.textContent = "OFFLINE";
        }
    }

    function openEsp32ControlPage() {
        window.location.href = "http://" + ESP32_HOST + "/";
    }

    async function checkEsp32Api() {
        if (!isEsp32ControlPage) return;
        try {
            const response = await fetch("/get-current-kit?_=" + Date.now(), {
                method: "GET",
                cache: "no-store"
            });
            if (!response.ok) throw new Error("HTTP " + response.status);
            const kitText = (await response.text()).trim();
            if (!/^\d+$/.test(kitText)) throw new Error("Unexpected board response");
            apiIsConnected = true;
        } catch (error) {
            apiIsConnected = false;
        }
        updateConnectionPanel();
    }

    function connectEsp32WebSocket() {
        if (!isEsp32ControlPage) return;
        try {
            const wsProtocol = location.protocol === "https:" ? "wss:" : "ws:";
            ws = new WebSocket(wsProtocol + "//" + location.host + "/ws");
            ws.onopen = function() {
                wsIsConnected = true;
                updateConnectionPanel();
            };
            ws.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    if (data.type === "PAD_HIT" && Number.isInteger(data.pad) && data.pad >= 0 && data.pad < 10) {
                        padPeaks[data.pad] = data.peak;
                        padTriggers[data.pad] = 10;
                    }
                } catch (error) {
                    // Ignore malformed live packets; keep the connection alive.
                }
            };
            ws.onerror = function() {
                wsIsConnected = false;
                updateConnectionPanel();
            };
            ws.onclose = function() {
                wsIsConnected = false;
                updateConnectionPanel();
                if (wsReconnectTimer) clearTimeout(wsReconnectTimer);
                wsReconnectTimer = setTimeout(connectEsp32WebSocket, 2500);
            };
        } catch (error) {
            wsIsConnected = false;
            updateConnectionPanel();
            if (wsReconnectTimer) clearTimeout(wsReconnectTimer);
            wsReconnectTimer = setTimeout(connectEsp32WebSocket, 2500);
        }
    }

    function startConnectionMonitor() {
        if (!isEsp32ControlPage) {
            updateConnectionPanel();
            return;
        }
        updateConnectionPanel();
        checkEsp32Api();
        connectEsp32WebSocket();
        if (apiMonitorTimer) clearInterval(apiMonitorTimer);
        apiMonitorTimer = setInterval(checkEsp32Api, 5000);
    }

    let padPeaks = new Array(10).fill(0);
    let padTriggers = new Array(10).fill(0);
    let canvas = document.getElementById('visCanvas');
    let ctx = canvas.getContext('2d');
    function renderVisualizer() {
        canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight;
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        let barWidth = (canvas.width / 10) - 6;
        for (let i = 0; i < 10; i++) {
            let x = i * (barWidth + 6) + 3;
            let h = (padPeaks[i] / 4095) * (canvas.height - 30);
            let y = canvas.height - h - 20;
            ctx.fillStyle = padTriggers[i] > 0 ? "#ff7a1a" : "#3ea6ff";
            ctx.fillRect(x, y, barWidth, h);
            if (padPeaks[i] > 0) padPeaks[i] = Math.max(0, padPeaks[i] - 150);
            if (padTriggers[i] > 0) padTriggers[i]--;
            ctx.fillStyle = "#8b8fa3"; ctx.font = "10px Arial";
            ctx.fillText("P" + (i + 1), x + (barWidth / 4), canvas.height - 5);
        }
        requestAnimationFrame(renderVisualizer);
    }
    renderVisualizer();

    function populateSdToneList() {
        fetch('/list-files').then(r => r.json()).then(data => {
            let dl = document.getElementById('sdToneList');
            dl.innerHTML = '';
            (data.files || []).forEach(f => {
                let m = f.name.match(/(\d+)\.WAV$/i);
                if (!m) return;
                let opt = document.createElement('option');
                opt.value = parseInt(m[1], 10);
                opt.label = f.name;
                dl.appendChild(opt);
            });
        }).catch(() => {});
    }

    function loadKitFromSelector() {
        let kitNum = parseInt(document.getElementById('kitSelector').value, 10);
        if (!kitNum || kitNum < 1 || kitNum > 90) { alert('Enter a kit number between 1 and 90.'); return; }
        let status = document.getElementById('kitSelectorStatus');
        status.innerHTML = 'Loading Kit ' + kitNum + '...'; status.style.color = '#ffb347';
        fetch('/select-kit?kit=' + kitNum).then(r => r.text()).then(() => {
            status.innerHTML = 'Kit ' + kitNum + ' loaded -- now editing this kit.'; status.style.color = '#33d17a';
            fetchAssignmentsForCurrentKit();
            fetchKitNameForSelector();
        }).catch(() => { status.innerHTML = 'Failed to switch kit.'; status.style.color = '#ff5c5c'; });
    }

    function fetchKitNameForSelector() {
        let kitNum = parseInt(document.getElementById('kitSelector').value, 10) || 1;
        fetch('/get-kit-name?kit=' + kitNum).then(r => r.text()).then(name => {
            document.getElementById('kitNameInput').value = name;
        });
    }

    function saveKitName() {
        let kitNum = parseInt(document.getElementById('kitSelector').value, 10) || 1;
        let name = document.getElementById('kitNameInput').value.trim();
        let status = document.getElementById('kitSelectorStatus');
        status.innerHTML = 'Saving name...'; status.style.color = '#ffb347';
        fetch('/save-kit-name?kit=' + kitNum + '&name=' + encodeURIComponent(name)).then(r => r.text()).then(() => {
            status.innerHTML = name ? `Kit ${kitNum} renamed to "${name}".` : `Kit ${kitNum} name cleared.`;
            status.style.color = '#33d17a';
        }).catch(() => { status.innerHTML = 'Failed to save name.'; status.style.color = '#ff5c5c'; });
    }

    function fetchAssignmentsForCurrentKit() {
        let kitNum = parseInt(document.getElementById('kitSelector').value, 10) || 1;
        fetch('/get-assignments?kit=' + kitNum).then(r => r.json()).then(data => {
            for (let i = 1; i <= 10; i++) {
                document.getElementById(`p${i}a`).value = data[`p${i}a`] || 0;
                document.getElementById(`p${i}b`).value = data[`p${i}b`] || 0;
                document.getElementById(`p${i}va`).value = data[`p${i}va`] ?? 100;
                document.getElementById(`p${i}vb`).value = data[`p${i}vb`] ?? 100;
                document.getElementById(`p${i}fa`).value = data[`p${i}fa`] ?? 0;
                document.getElementById(`p${i}fb`).value = data[`p${i}fb`] ?? 0;
            }
        });
    }

    function loadChainForEditing() {
        let n = parseInt(document.getElementById('chainNumSelector').value, 10) || 1;
        fetch('/get-chain?chain=' + n).then(r => r.json()).then(data => {
            document.getElementById('chainStepsInput').value = (data.steps || []).filter(v => v > 0).join(',');
        });
    }

    function saveChainSteps() {
        let n = parseInt(document.getElementById('chainNumSelector').value, 10) || 1;
        let raw = document.getElementById('chainStepsInput').value.trim();
        let steps = raw.length ? raw.split(',').map(s => parseInt(s.trim(), 10)).filter(v => !isNaN(v) && v > 0) : [];
        let status = document.getElementById('chainStatus');
        status.innerHTML = 'Saving...'; status.style.color = '#ffb347';
        fetch('/save-chain?chain=' + n + '&steps=' + steps.join(',')).then(r => r.text()).then(() => {
            status.innerHTML = 'Chain ' + n + ' saved (' + steps.length + ' steps).'; status.style.color = '#33d17a';
        }).catch(() => { status.innerHTML = 'Save failed.'; status.style.color = '#ff5c5c'; });
    }

    function toggleChainMode() {
        let on = document.getElementById('chainModeToggle').checked;
        let n = parseInt(document.getElementById('chainNumSelector').value, 10) || 1;
        fetch('/set-chain-mode?on=' + (on ? 1 : 0) + '&chain=' + n);
    }

    function chainStep(dir) {
        let status = document.getElementById('chainStatus');
        fetch('/chain-step?dir=' + dir).then(r => r.text()).then(kit => {
            status.innerHTML = 'Now playing Kit ' + kit + '.'; status.style.color = '#33d17a';
        }).catch(() => { status.innerHTML = 'Step failed -- check connection.'; status.style.color = '#ff5c5c'; });
    }

    window.onload = function() {
        startConnectionMonitor();
        if (!isEsp32ControlPage) {
            // GitHub Pages is a secure HTTPS preview, not the ESP32 control origin.
            // Browser mixed-content rules prevent its page from calling the ESP32 HTTP/WS API.
            return;
        }
        populateSdToneList();
        fetch('/get-chain-mode').then(r => r.json()).then(data => {
            document.getElementById('chainModeToggle').checked = !!data.active;
            document.getElementById('chainNumSelector').value = data.chain;
            loadChainForEditing();
        });
        fetch('/get-current-kit').then(r => r.text()).then(k => {
            document.getElementById('kitSelector').value = k;
            fetchAssignmentsForCurrentKit();
            fetchKitNameForSelector();
        });
        fetch('/get-settings').then(r => r.json()).then(data => {
            for (let i = 1; i <= 10; i++) {
                document.getElementById(`p${i}th`).value = data.padThreshold?.[i - 1] ?? 600;
                document.getElementById(`p${i}db`).value = data.padDebounceMs?.[i - 1] ?? 100;
                document.getElementById(`p${i}pw`).value = data.padPeakWindowMs?.[i - 1] ?? 10;
                document.getElementById(`p${i}ct`).value = data.padCrosstalkLockMs?.[i - 1] ?? 4;
            }
            document.getElementById('s_masterVolume').value = Math.round(data.masterVolume * 100);
            document.getElementById('s_fixedVelocity').value = data.fixedVelocity;
            document.getElementById('s_dynamicVelocity').checked = !!data.dynamicVelocity;
            document.getElementById('s_systemMode').value = data.systemMode;
            document.getElementById('s_padCount').value = data.padCount;
            padCount = data.padCount;
            loadCrosstalkMatrix();
        });
    };

    let padCount = 8;
    const MATRIX_DIM = 10;

    function syncCrosstalkLevelPadOptions() {
        const select = document.getElementById('ctLevelPad');
        if (!select) return;
        const previous = parseInt(select.value, 10) || 1;
        select.innerHTML = '';
        for (let i = 1; i <= padCount; i++) {
            const option = document.createElement('option');
            option.value = String(i);
            option.textContent = 'PAD ' + i;
            select.appendChild(option);
        }
        select.value = String(Math.min(previous, padCount));
    }

    function buildMatrixGrid(values) {
        let n = padCount;
        syncCrosstalkLevelPadOptions();
        let html = '<table style="border-collapse:collapse; font-size:12px;"><tr><td></td>';
        for (let c = 0; c < n; c++) html += `<td style="padding:4px; text-align:center; color:var(--text-dim);">P${c + 1}</td>`;
        html += '</tr>';
        for (let r = 0; r < n; r++) {
            html += `<tr><td style="padding:4px; color:var(--text-dim);">P${r + 1}</td>`;
            for (let c = 0; c < n; c++) {
                if (r === c) {
                    html += `<td style="padding:2px;"><input disabled value="—" style="width:42px; text-align:center; background:transparent; border:1px solid var(--border); border-radius:6px; color:var(--text-dim); padding:5px 2px;"></td>`;
                } else {
                    let v = values[r * MATRIX_DIM + c] ?? 30;
                    html += `<td style="padding:2px;"><input type="number" inputmode="numeric" pattern="[0-9]*" min="0" max="100" id="ctm_${r}_${c}" value="${v}" style="width:42px; text-align:center; background:#0b0c10; border:1px solid var(--border); border-radius:6px; color:var(--text); padding:5px 2px;"></td>`;
                }
            }
            html += '</tr>';
        }
        html += '</table>';
        document.getElementById('matrix-container').innerHTML = html;
    }

    function loadCrosstalkMatrix() {
        fetch('/get-crosstalk-matrix').then(r => r.json()).then(data => {
            padCount = data.padCount || 8;
            buildMatrixGrid(data.matrix || []);
        });
    }

    function saveCrosstalkMatrix(statusElementId = "matrixStatus") {
        let matrix = new Array(MATRIX_DIM * MATRIX_DIM).fill(0);
        for (let r = 0; r < padCount; r++) {
            for (let c = 0; c < padCount; c++) {
                if (r === c) continue;
                let el = document.getElementById(`ctm_${r}_${c}`);
                matrix[r * MATRIX_DIM + c] = el ? parseInt(el.value) || 0 : 30;
            }
        }
        let status = document.getElementById(statusElementId);
        if (status) { status.innerHTML = "Saving..."; status.style.color = "#ffb347"; }
        fetch('/save-crosstalk-matrix', {
            method: 'POST', headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ matrix })
        }).then(r => {
            if (!r.ok) throw new Error("HTTP " + r.status);
            return r.text();
        }).then(() => {
            if (status) { status.innerHTML = "Crosstalk Matrix Saved!"; status.style.color = "#33d17a"; }
        }).catch(error => {
            if (status) { status.innerHTML = "Save failed: " + error.message; status.style.color = "#ff5c5c"; }
        });
    }

    function applyCrosstalkLevelToSelectedPad() {
        const pad = parseInt(document.getElementById('ctLevelPad').value, 10) - 1;
        const level = parseInt(document.getElementById('ctLevelValue').value, 10);
        const status = document.getElementById('ctLevelStatus');

        if (!Number.isInteger(pad) || pad < 0 || pad >= padCount ||
            ![50, 60, 70, 80].includes(level)) {
            status.innerHTML = "Select a valid pad and level.";
            status.style.color = "#ff5c5c";
            return;
        }

        for (let other = 0; other < padCount; other++) {
            if (other === pad) continue;
            const cell = document.getElementById(`ctm_${pad}_${other}`);
            if (cell) cell.value = level;
        }

        status.innerHTML = `Applying ${level}% to PAD ${pad + 1}...`;
        status.style.color = "#ffb347";
        saveCrosstalkMatrix("ctLevelStatus");
    }

    function resetCrosstalkMatrix() {
        for (let r = 0; r < padCount; r++) {
            for (let c = 0; c < padCount; c++) {
                if (r === c) continue;
                let el = document.getElementById(`ctm_${r}_${c}`);
                if (el) el.value = 30;
            }
        }
    }

    function applyCrosstalkToAll() {
        let v = parseInt(document.getElementById('ctmQuickValue').value);
        if (isNaN(v)) v = 30;
        v = Math.max(30, Math.min(100, v));
        for (let r = 0; r < padCount; r++) {
            for (let c = 0; c < padCount; c++) {
                if (r === c) continue;
                let el = document.getElementById(`ctm_${r}_${c}`);
                if (el) el.value = v;
            }
        }
        saveCrosstalkMatrix();
    }

    let previewDebounceTimer = null;
    let phoneAudio = new Audio();
    function previewToneFromPad(padIndex, layer) {
        clearTimeout(previewDebounceTimer);
        previewDebounceTimer = setTimeout(() => {
            let toneVal = document.getElementById(`p${padIndex}${layer}`).value;
            let filtVal = document.getElementById(`p${padIndex}f${layer}`).value;
            if (!toneVal || toneVal == 0) return;
            fetch(`/preview-tone?tone=${toneVal}&filter=${filtVal || 0}`);
            phoneAudio.pause();
            phoneAudio.src = `/get-wav?tone=${toneVal}&t=${Date.now()}`;
            phoneAudio.play().catch(() => {});
        }, 250);
    }

    function saveSettings() {
        let params = new URLSearchParams();
        for (let i = 1; i <= 10; i++) {
            params.append(`p${i}th`, document.getElementById(`p${i}th`).value);
            params.append(`p${i}db`, document.getElementById(`p${i}db`).value);
            params.append(`p${i}pw`, document.getElementById(`p${i}pw`).value);
            params.append(`p${i}ct`, document.getElementById(`p${i}ct`).value);
        }
        params.append('masterVolume', (parseFloat(document.getElementById('s_masterVolume').value) / 100).toFixed(2));
        params.append('fixedVelocity', document.getElementById('s_fixedVelocity').value);
        params.append('dynamicVelocity', document.getElementById('s_dynamicVelocity').checked ? '1' : '0');
        params.append('systemMode', document.getElementById('s_systemMode').value);
        params.append('padCount', document.getElementById('s_padCount').value);
        let status = document.getElementById('settingsStatus');
        status.innerHTML = "Saving..."; status.style.color = "#ffb347";
        fetch('/save-settings?' + params.toString()).then(r => r.text()).then(() => {
            status.innerHTML = "Settings Saved!"; status.style.color = "#33d17a";
        });
    }

    function uploadFileBtn() {
        let fileInput = document.getElementById('file');
        if (fileInput.files.length === 0) return;
        let file = fileInput.files[0];
        let formData = new FormData(); formData.append("file", file, file.name);
        let status = document.getElementById('uploadStatus');
        status.innerHTML = "Uploading " + file.name + "..."; status.style.color = "#ffb347";
        fetch('/upload', { method: 'POST', body: formData }).then(r => r.text()).then(() => {
            status.innerHTML = "Uploaded: " + file.name; status.style.color = "#33d17a";
        }).catch(() => { status.innerHTML = "Upload Failed!"; status.style.color = "#ff5c5c"; });
    }

    function saveAssignments() {
        let kitNum = parseInt(document.getElementById('kitSelector').value, 10) || 1;
        let params = new URLSearchParams();
        params.append('kit', kitNum);
        for (let i = 1; i <= 10; i++) {
            params.append(`p${i}a`, document.getElementById(`p${i}a`).value);
            params.append(`p${i}b`, document.getElementById(`p${i}b`).value);
            params.append(`p${i}va`, document.getElementById(`p${i}va`).value);
            params.append(`p${i}vb`, document.getElementById(`p${i}vb`).value);
            params.append(`p${i}fa`, document.getElementById(`p${i}fa`).value);
            params.append(`p${i}fb`, document.getElementById(`p${i}fb`).value);
        }
        let status = document.getElementById('saveStatus');
        status.innerHTML = "Saving..."; status.style.color = "#ffb347";
        fetch('/save-assignments?' + params.toString()).then(r => r.text()).then(() => {
            status.innerHTML = "Saved! Pads updated instantly."; status.style.color = "#33d17a";
        });
    }

    function loadFileList() {
        let listEl = document.getElementById('fileListContainer');
        listEl.innerHTML = '<div class="file-empty">Loading...</div>';
        let controller = new AbortController();
        let timeoutId = setTimeout(() => controller.abort(), 8000);
        fetch('/list-files', { signal: controller.signal }).then(r => r.json()).then(data => {
            clearTimeout(timeoutId);
            if (!data.files || data.files.length === 0) {
                listEl.innerHTML = '<div class="file-empty">No sample files on SD card yet.</div>';
                return;
            }
            listEl.innerHTML = '';
            if (data.truncated) {
                let notice = document.createElement('div');
                notice.className = 'file-empty';
                notice.style.color = '#ffb347';
                notice.innerHTML = `Showing first ${data.shown} files -- more exist on the SD card but were left out to protect device memory. Use the Tone field's pick-list (Sounds tab) to reach tone numbers beyond this list.`;
                listEl.appendChild(notice);
            }
            data.files.forEach(f => {
                let sizeKb = (f.size / 1024).toFixed(1);
                let row = document.createElement('div');
                row.className = 'file-row';
                row.innerHTML = `<div><span class="fname">${f.name}</span><span class="fsize">${sizeKb} KB</span></div>
                    <button class="btn btn-red btn-sm" onclick="deleteFile('${f.name}')">Delete</button>`;
                listEl.appendChild(row);
            });
        }).catch(() => {
            clearTimeout(timeoutId);
            listEl.innerHTML = '<div class="file-empty">Could not load file list -- the device may have restarted. <button class="btn btn-outline btn-sm" onclick="loadFileList()">Retry</button></div>';
        });
    }

    function deleteFile(name) {
        if (!confirm(`Delete ${name} from the SD card? This cannot be undone.`)) return;
        let status = document.getElementById('fileStatus');
        status.innerHTML = "Deleting..."; status.style.color = "#ffb347";
        fetch('/delete-file?name=' + encodeURIComponent(name)).then(r => r.text()).then(() => {
            status.innerHTML = "Deleted " + name; status.style.color = "#33d17a";
            loadFileList();
        });
    }

    function restoreJson() {
        let fileInput = document.getElementById('jsonFile');
        if (fileInput.files.length === 0) return;
        let file = fileInput.files[0];
        let formData = new FormData(); formData.append("file", file, file.name);
        let status = document.getElementById('jsonStatus');
        status.innerHTML = "Restoring..."; status.style.color = "#ffb347";
        fetch('/restore-json', { method: 'POST', body: formData }).then(r => r.text()).then(() => {
            status.innerHTML = "Restored! Reloading..."; status.style.color = "#33d17a";
            setTimeout(() => location.reload(), 1000);
        });
    }
</script>
</body>
</html>

)rawhtml";

class UIHandler {
public:
    UIHandler() : _currentKit(1), _padActive(0), _btnUpPrev(HIGH), _btnDownPrev(HIGH), _btnBackPrev(HIGH), _btnEncPrev(HIGH), _encLastA(0), _encAccum(0), _currentScreen(SCREEN_MAIN), _menuIdx(0), _menuScrollWinStart(0), _selectedEditPad(0), _selectedLayer(0), _selectedToneNum(1), _selectedPadVolPercent(100), _btnUpPressedTime(0), _btnDownPressedTime(0), _lastScrollTime(0), _isUpLongPress(false), _isDownLongPress(false), _advMenuIdx(0), _lastToneChangeTime(0), _previewTriggered(false), _selectedPadFiltVal(0), _selectedPadPitchVal(0), _selectedPadPanVal(0), _selectedChainStep(0) {}
    
    bool begin() {
        pinMode(BTN_KIT_UP_PIN, INPUT_PULLUP); pinMode(BTN_KIT_DOWN_PIN, INPUT_PULLUP); pinMode(BTN_BACK_PIN, INPUT_PULLUP); pinMode(ENC_BTN_PIN, INPUT_PULLUP); pinMode(ENC_A_PIN, INPUT_PULLUP); pinMode(ENC_B_PIN, INPUT_PULLUP);
        _encLastA = digitalRead(ENC_A_PIN); u8g2.begin(); return true;
    }
    void showSplash() {
        u8g2.clearBuffer(); u8g2.drawFrame(0, 0, 128, 64); u8g2.setFont(u8g2_font_ncenB14_tr); u8g2.drawStr(18, 28, "OctaPad"); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(10, 44, "ERO DSP FILTER"); u8g2.drawStr(24, 57, "ESP32-S3 v2.24"); u8g2.sendBuffer(); delay(1200);
    }
    void showMain() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(0, 8, MY_NAME); 
        char kitStr[8]; snprintf(kitStr, sizeof(kitStr), "%02d/90", _currentKit); u8g2.drawStr(92, 8, kitStr); 
        u8g2.drawLine(0, 10, 127, 10); u8g2.setFont(u8g2_font_5x7_tr);
        xSemaphoreTake(settingsMutex, portMAX_DELAY); SysMode sm = globalSystemMode; uint8_t padsActiveCount = globalPadCount; xSemaphoreGive(settingsMutex);
        if(sm == MODE_SAMPLER) u8g2.drawStr(0, 18, "MODE: SAMPLER"); else if(sm == MODE_MIDI_OUT) u8g2.drawStr(0, 18, "MODE: MIDI OUT"); else u8g2.drawStr(0, 18, "MODE: HYBRID OS");
        u8g2.setFont(u8g2_font_ncenB14_tr); String buf = getKitDisplayName(_currentKit); u8g2.drawStr((128 - u8g2.getStrWidth(buf.c_str())) / 2, 34, buf.c_str());
        u8g2.setFont(u8g2_font_5x8_tr);
        if (padsActiveCount == 8) {
            for (int i = 0; i < 8; i++) {
                int x = 2 + (i % 4) * 32; int y = 39 + (i / 4) * 12;
                if ((_padActive >> i) & 1) { u8g2.drawBox(x, y, 28, 11); u8g2.setDrawColor(0); } else { u8g2.drawFrame(x, y, 28, 11); u8g2.setDrawColor(1); }
                char num[3]; snprintf(num, sizeof(num), "P%d", i + 1); u8g2.drawStr(x + 7, y + 9, num); u8g2.setDrawColor(1);
            }
        } else { 
            for (int i = 0; i < 10; i++) {
                int x = 2 + (i % 5) * 25; int y = 39 + (i / 5) * 12;
                if ((_padActive >> i) & 1) { u8g2.drawBox(x, y, 22, 11); u8g2.setDrawColor(0); } else { u8g2.drawFrame(x, y, 22, 11); u8g2.setDrawColor(1); }
                char num[4]; snprintf(num, sizeof(num), "%d", i + 1); u8g2.drawStr(x + (22 - u8g2.getStrWidth(num))/2, y + 9, num); u8g2.setDrawColor(1);
            }
        }
        u8g2.sendBuffer();
    }
    void showMenuList() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 9, "--- SETTINGS ---"); u8g2.drawLine(0, 11, 127, 11);
        const int totalItems = 14; 
        const char* menuItems[totalItems] = { "1. Pad Sensitivity", "2. Master Volume", "3. MIDI Velocity", "4. System Mode", "5. Edit Pad Tone", "6. Pad Volume", "7. Pad Layout", "8. Advanced Tune", "9. Pad Filter", "10. Wireless Setup", "11. Pad Pitch/Tune", "12. Pad Pan", "13. Kit Chains", "14. Chain Mode" }; 
        if (_menuIdx < _menuScrollWinStart) { _menuScrollWinStart = _menuIdx; } else if (_menuIdx >= _menuScrollWinStart + 3) { _menuScrollWinStart = _menuIdx - 2; }
        for (int i = 0; i < 3; i++) {
            int itemActualIdx = _menuScrollWinStart + i; if (itemActualIdx >= totalItems) break;
            int yPos = 26 + (i * 15); 
            if (itemActualIdx == _menuIdx) { u8g2.drawBox(0, yPos - 11, 128, 15); u8g2.setDrawColor(0); } else { u8g2.setDrawColor(1); }
            u8g2.drawStr(5, yPos, menuItems[itemActualIdx]); u8g2.setDrawColor(1);
        }
        u8g2.sendBuffer();
    }
    
    void showWebServerScreen() {
        u8g2.clearBuffer();
        u8g2.drawFrame(0, 0, 128, 64);
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.drawStr(12, 15, "WEB MANAGEMENT");
        u8g2.drawLine(4, 18, 124, 18);
        u8g2.setFont(u8g2_font_5x8_tr);
        u8g2.drawStr(8, 30, "SSID: ERO_PAD_WIFI");
        u8g2.drawStr(8, 41, "IP  : 192.168.4.1");
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawStr(8, 55, "Press BACK to normal play");
        u8g2.sendBuffer();
    }

    void showPadSensSel() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "PAD SENSITIVITY"); u8g2.drawLine(0, 13, 127, 13); u8g2.setFont(u8g2_font_6x12_tr); char padStr[25]; snprintf(padStr, sizeof(padStr), "SELECT PAD: %d", _selectedEditPad + 1); u8g2.drawStr((128 - u8g2.getStrWidth(padStr))/2, 42, padStr); u8g2.sendBuffer();
    }

    void showPadSensSet() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr);
        char title[24]; snprintf(title, sizeof(title), "PAD %d THRESHOLD", _selectedEditPad + 1);
        u8g2.drawStr(5, 10, title); u8g2.drawLine(0, 13, 127, 13);
        xSemaphoreTake(settingsMutex, portMAX_DELAY); uint16_t currentThresh = padThreshold[_selectedEditPad]; xSemaphoreGive(settingsMutex);
        char valStr[10]; snprintf(valStr, sizeof(valStr), "%d", currentThresh); u8g2.setFont(u8g2_font_ncenB12_tr); u8g2.drawStr(15, 52, valStr);
        u8g2.drawFrame(60, 42, 62, 10); u8g2.drawBox(61, 43, constrain(map(currentThresh, 200, 2000, 0, 60), 0, 60), 8); u8g2.sendBuffer();
    }

    void showSetVol() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "[SETTINGS] > VOL"); u8g2.drawLine(0, 13, 127, 13); u8g2.drawStr(10, 32, "Master Volume:");
        xSemaphoreTake(settingsMutex, portMAX_DELAY); int volPercent = (int)(globalMasterVolume * 100.0f); xSemaphoreGive(settingsMutex);
        char valStr[10]; snprintf(valStr, sizeof(valStr), "%d %%", volPercent); u8g2.setFont(u8g2_font_ncenB12_tr); u8g2.drawStr(15, 52, valStr);
        u8g2.drawFrame(60, 42, 62, 10); u8g2.drawBox(61, 43, constrain(map(volPercent, 0, 100, 0, 60), 0, 60), 8); u8g2.sendBuffer();
    }
    void showSetVelocity() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "[SETTINGS] > VELOCITY"); u8g2.drawLine(0, 13, 127, 13);
        xSemaphoreTake(settingsMutex, portMAX_DELAY); uint8_t currentVel = globalFixedVelocity; bool dynOn = globalDynamicVelocity; xSemaphoreGive(settingsMutex);
        u8g2.drawStr(10, 24, "Encoder Click: Toggle Mode");
        if (dynOn) {
            u8g2.setFont(u8g2_font_ncenB12_tr); u8g2.drawStr(8, 44, "MODE: DYNAMIC");
            u8g2.setFont(u8g2_font_5x8_tr); u8g2.drawStr(10, 58, "Velocity follows hit strength");
        } else {
            u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(10, 34, "MODE: FIXED");
            char valStr[10]; snprintf(valStr, sizeof(valStr), "%d", currentVel); u8g2.setFont(u8g2_font_ncenB12_tr); u8g2.drawStr(15, 56, valStr);
            u8g2.drawFrame(60, 46, 62, 10); u8g2.drawBox(61, 47, constrain(map(currentVel, 1, 127, 0, 60), 0, 60), 8);
        }
        u8g2.sendBuffer();
    }
    void showSetMode() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "[SETTINGS] > SYS MODE"); u8g2.drawLine(0, 13, 127, 13);
        xSemaphoreTake(settingsMutex, portMAX_DELAY); SysMode sm = globalSystemMode; xSemaphoreGive(settingsMutex);
        const char* modes[3] = { "  SAMPLER (SD Card)", "  MIDI OUT (USB Only)", "  HYBRID (Dual Play)" };
        for(int i=0; i<3; i++) {
            int y = 28 + (i * 12); if(i == (int)sm) { u8g2.drawBox(0, y - 9, 128, 11); u8g2.setDrawColor(0); u8g2.drawStr(2, y, ">"); } else u8g2.setDrawColor(1);
            u8g2.drawStr(6, y, modes[i]); u8g2.setDrawColor(1);
        }
        u8g2.sendBuffer();
    }
    void showEditPadSel() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "SELECT TARGET PAD"); u8g2.drawLine(0, 13, 127, 13);
        char padStr[15]; snprintf(padStr, sizeof(padStr), "> PAD %d <", _selectedEditPad + 1); u8g2.setFont(u8g2_font_ncenB12_tr); u8g2.drawStr(24, 46, padStr); u8g2.sendBuffer();
    }
    void showEditLayerSel() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "SELECT TARGET LAYER"); u8g2.drawLine(0, 13, 127, 13);
        const char* layers[2] = { " Layer A (Primary)", " Layer B (Secondary mixed)" };
        for(int i=0; i<2; i++) {
            int y = 32 + (i * 15); if(i == _selectedLayer) { u8g2.drawBox(0, y - 10, 128, 14); u8g2.setDrawColor(0); } else u8g2.setDrawColor(1);
            u8g2.drawStr(5, y, layers[i]); u8g2.setDrawColor(1);
        }
        u8g2.sendBuffer();
    }
    void showEditWavSel() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr);
        char title[30]; snprintf(title, sizeof(title), "PAD %d - LAYER %c", _selectedEditPad + 1, (_selectedLayer == 0) ? 'A' : 'B');
        u8g2.drawStr(5, 10, title); u8g2.drawLine(0, 13, 127, 13); u8g2.drawStr(10, 28, "Set Tone Number:");
        u8g2.setFont(u8g2_font_ncenB14_tr); char tStr[20]; if (_selectedToneNum == 0) snprintf(tStr, sizeof(tStr), "--- (EMPTY)"); else snprintf(tStr, sizeof(tStr), "TONE %04d.WAV", _selectedToneNum);
        u8g2.drawStr(10, 52, tStr); u8g2.sendBuffer();
    }
    void showPadVolSel() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "PAD INDIVIDUAL VOL"); u8g2.drawLine(0, 13, 127, 13); u8g2.setFont(u8g2_font_6x12_tr); char padStr[25]; snprintf(padStr, sizeof(padStr), "SELECT PAD: %d", _selectedEditPad + 1); int textWidth = u8g2.getStrWidth(padStr); int xPos = (128 - textWidth) / 2; u8g2.drawStr(xPos, 42, padStr); u8g2.sendBuffer();
    }
    void showPadVolSet() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr);
        char title[30]; snprintf(title, sizeof(title), "PAD %d L%c VOL ADJUST", _selectedEditPad + 1, (_selectedLayer == 0) ? 'A' : 'B');
        u8g2.drawStr(5, 10, title); u8g2.drawLine(0, 13, 127, 13); u8g2.drawStr(10, 26, "Encoder Click: Change Layer");
        u8g2.setFont(u8g2_font_ncenB14_tr); char valStr[12]; snprintf(valStr, sizeof(valStr), "%d %%", _selectedPadVolPercent); u8g2.drawStr(15, 52, valStr);
        u8g2.drawFrame(75, 42, 48, 10); u8g2.drawBox(76, 43, constrain(map(_selectedPadVolPercent, 0, 100, 0, 46), 0, 46), 8); u8g2.sendBuffer();
    }
    void showPadFiltSel() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "PAD REALTIME FILTER"); u8g2.drawLine(0, 13, 127, 13); u8g2.setFont(u8g2_font_6x12_tr); char padStr[25]; snprintf(padStr, sizeof(padStr), "SELECT PAD: %d", _selectedEditPad + 1); u8g2.drawStr((128 - u8g2.getStrWidth(padStr))/2, 42, padStr); u8g2.sendBuffer();
    }
    void showPadFiltSet() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr);
        char title[30]; snprintf(title, sizeof(title), "PAD %d L%c DSP FILTER", _selectedEditPad + 1, (_selectedLayer == 0) ? 'A' : 'B');
        u8g2.drawStr(5, 10, title); u8g2.drawLine(0, 13, 127, 13);
        
        u8g2.setFont(u8g2_font_5x8_tr);
        if (_selectedPadFiltVal == 0) u8g2.drawStr(10, 26, "Type: BYPASS (Normal)");
        else if (_selectedPadFiltVal < 0) u8g2.drawStr(10, 26, "Type: LOW-PASS (Heavy/Bass)");
        else u8g2.drawStr(10, 26, "Type: HIGH-PASS (Crisp/Thin)");
        
        u8g2.setFont(u8g2_font_ncenB14_tr); char valStr[15];
        snprintf(valStr, sizeof(valStr), "%d", _selectedPadFiltVal); u8g2.drawStr(15, 52, valStr);
        
        u8g2.drawFrame(60, 42, 62, 10);
        int barWidth = map(_selectedPadFiltVal, -50, 50, 0, 58);
        u8g2.drawBox(62, 44, barWidth, 6);
        u8g2.sendBuffer();
    }
    void showPadPitchSel() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "PAD PITCH / TUNE"); u8g2.drawLine(0, 13, 127, 13); u8g2.setFont(u8g2_font_6x12_tr); char padStr[25]; snprintf(padStr, sizeof(padStr), "SELECT PAD: %d", _selectedEditPad + 1); u8g2.drawStr((128 - u8g2.getStrWidth(padStr))/2, 42, padStr); u8g2.sendBuffer();
    }
    void showPadPitchSet() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr);
        char title[30]; snprintf(title, sizeof(title), "PAD %d L%c PITCH/TUNE", _selectedEditPad + 1, (_selectedLayer == 0) ? 'A' : 'B');
        u8g2.drawStr(5, 10, title); u8g2.drawLine(0, 13, 127, 13);

        u8g2.setFont(u8g2_font_5x8_tr);
        if (_selectedPadPitchVal == 0) u8g2.drawStr(10, 26, "Normal Pitch (no shift)");
        else if (_selectedPadPitchVal > 0) u8g2.drawStr(10, 26, "Higher / Thinner");
        else u8g2.drawStr(10, 26, "Lower / Deeper");

        u8g2.setFont(u8g2_font_ncenB14_tr); char valStr[8];
        snprintf(valStr, sizeof(valStr), "%+d", _selectedPadPitchVal);
        u8g2.drawStr(15, 52, valStr);

        u8g2.drawFrame(60, 42, 62, 10);
        int barWidth = map(_selectedPadPitchVal, -12, 12, 0, 58);
        u8g2.drawBox(62, 44, barWidth, 6);
        u8g2.sendBuffer();
    }
    void showPadPanSel() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "PAD PAN (L/R)"); u8g2.drawLine(0, 13, 127, 13); u8g2.setFont(u8g2_font_6x12_tr); char padStr[25]; snprintf(padStr, sizeof(padStr), "SELECT PAD: %d", _selectedEditPad + 1); u8g2.drawStr((128 - u8g2.getStrWidth(padStr))/2, 42, padStr); u8g2.sendBuffer();
    }
    void showPadPanSet() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr);
        char title[30]; snprintf(title, sizeof(title), "PAD %d L%c PAN", _selectedEditPad + 1, (_selectedLayer == 0) ? 'A' : 'B');
        u8g2.drawStr(5, 10, title); u8g2.drawLine(0, 13, 127, 13); u8g2.drawStr(10, 24, "Encoder Click: Change Layer");

        constexpr int trackX = 14, trackW = 100, trackY = 40;
        u8g2.setFont(u8g2_font_5x8_tr);
        u8g2.drawStr(2, trackY + 3, "L"); u8g2.drawStr(122, trackY + 3, "R");
        u8g2.drawLine(trackX, trackY, trackX + trackW, trackY);
        u8g2.drawLine(trackX, trackY - 3, trackX, trackY + 3);
        u8g2.drawLine(trackX + trackW, trackY - 3, trackX + trackW, trackY + 3);
        u8g2.drawLine(trackX + trackW / 2, trackY - 2, trackX + trackW / 2, trackY + 2);

        int thumbX = trackX + map(_selectedPadPanVal, -100, 100, 0, trackW);
        u8g2.drawBox(thumbX - 3, trackY - 5, 6, 10);

        u8g2.setFont(u8g2_font_ncenB10_tr); char valStr[12];
        if (_selectedPadPanVal == 0) snprintf(valStr, sizeof(valStr), "CENTER");
        else if (_selectedPadPanVal < 0) snprintf(valStr, sizeof(valStr), "L %d", -_selectedPadPanVal);
        else snprintf(valStr, sizeof(valStr), "R %d", _selectedPadPanVal);
        u8g2.drawStr((128 - u8g2.getStrWidth(valStr)) / 2, 58, valStr);
        u8g2.sendBuffer();
    }
    void showChainSel() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "KIT CHAINS"); u8g2.drawLine(0, 13, 127, 13);
        char title[20]; snprintf(title, sizeof(title), "Chain %d / %d", globalActiveChain, MAX_CHAINS);
        u8g2.setFont(u8g2_font_ncenB12_tr); u8g2.drawStr((128 - u8g2.getStrWidth(title)) / 2, 34, title);

        char preview[40] = ""; int written = 0; bool any = false;
        for (int s = 0; s < MAX_CHAIN_STEPS && written < 5; s++) {
            if (kitChains[globalActiveChain][s] == 0) continue;
            char part[8]; snprintf(part, sizeof(part), written == 0 ? "%d" : ">%d", kitChains[globalActiveChain][s]);
            strncat(preview, part, sizeof(preview) - strlen(preview) - 1);
            written++; any = true;
        }
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.drawStr((128 - u8g2.getStrWidth(any ? preview : "(empty)")) / 2, 50, any ? preview : "(empty)");
        u8g2.drawStr(5, 62, "Click: Edit steps");
        u8g2.sendBuffer();
    }
    void showChainEdit() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr);
        char title[24]; snprintf(title, sizeof(title), "Chain %d - Step %d/%d", globalActiveChain, _selectedChainStep + 1, MAX_CHAIN_STEPS);
        u8g2.drawStr(5, 10, title); u8g2.drawLine(0, 13, 127, 13); u8g2.drawStr(10, 26, "Encoder Click: Next Step");

        uint8_t val = kitChains[globalActiveChain][_selectedChainStep];
        u8g2.setFont(u8g2_font_ncenB14_tr); char valStr[14];
        if (val == 0) snprintf(valStr, sizeof(valStr), "-- (unused)");
        else snprintf(valStr, sizeof(valStr), "Kit %d", val);
        u8g2.drawStr((128 - u8g2.getStrWidth(valStr)) / 2, 50, valStr);
        u8g2.sendBuffer();
    }
    void showChainMode() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "CHAIN MODE"); u8g2.drawLine(0, 13, 127, 13);
        char sub[20]; snprintf(sub, sizeof(sub), "Active Chain: %d", globalActiveChain);
        u8g2.drawStr(10, 26, sub);
        u8g2.setFont(u8g2_font_ncenB14_tr);
        const char* stateStr = globalChainModeActive ? "ON" : "OFF";
        u8g2.drawStr((128 - u8g2.getStrWidth(stateStr)) / 2, 52, stateStr);
        u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(10, 62, "Encoder Click: Toggle");
        u8g2.sendBuffer();
    }
    void showSetLayout() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "[SETTINGS] > LAYOUT"); u8g2.drawLine(0, 13, 127, 13); u8g2.drawStr(10, 28, "Select Pad Layout:");
        xSemaphoreTake(settingsMutex, portMAX_DELAY); uint8_t currentCount = globalPadCount; xSemaphoreGive(settingsMutex);
        u8g2.setFont(u8g2_font_ncenB14_tr); char layoutStr[15]; snprintf(layoutStr, sizeof(layoutStr), "%d PADS MODE", currentCount);
        u8g2.drawStr(15, 50, layoutStr); u8g2.sendBuffer();
    }
    void showAdvPadSel() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 10, "ADV TUNE: SELECT PAD"); u8g2.drawLine(0, 13, 127, 13); u8g2.setFont(u8g2_font_6x12_tr); char padStr[25]; snprintf(padStr, sizeof(padStr), "PAD: %d", _selectedEditPad + 1); u8g2.drawStr((128 - u8g2.getStrWidth(padStr))/2, 42, padStr); u8g2.sendBuffer();
    }
    void showAdvTuneList() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); char hdr[24]; snprintf(hdr, sizeof(hdr), "-- PAD %d ADV TUNE --", _selectedEditPad + 1); u8g2.drawStr(2, 9, hdr); u8g2.drawLine(0, 11, 127, 11);
        const char* advItems[4] = { "1. Debounce (ms)", "2. Peak Window(ms)", "3. Crosstalk Lock(ms)", "4. CT Level (50-80%)" };
        for (int i = 0; i < 4; i++) { int yPos = 18 + (i * 11); if (i == _advMenuIdx) { u8g2.drawBox(0, yPos - 8, 128, 11); u8g2.setDrawColor(0); } else { u8g2.setDrawColor(1); } u8g2.drawStr(5, yPos, advItems[i]); u8g2.setDrawColor(1); }
        u8g2.sendBuffer();
    }
    void showSetDebounce() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); char hdr[24]; snprintf(hdr, sizeof(hdr), "PAD %d > DEBOUNCE", _selectedEditPad + 1); u8g2.drawStr(5, 10, hdr); u8g2.drawLine(0, 13, 127, 13); u8g2.drawStr(10, 30, "Debounce Time:");
        xSemaphoreTake(settingsMutex, portMAX_DELAY); uint16_t currentDeb = padDebounceMs[_selectedEditPad]; xSemaphoreGive(settingsMutex);
        char valStr[15]; snprintf(valStr, sizeof(valStr), "%d ms", currentDeb); u8g2.setFont(u8g2_font_ncenB12_tr); u8g2.drawStr(15, 52, valStr);
        u8g2.drawFrame(70, 42, 52, 10); u8g2.drawBox(71, 43, constrain(map(currentDeb, 10, 300, 0, 50), 0, 50), 8); u8g2.sendBuffer();
    }
    void showSetPeakWin() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); char hdr[24]; snprintf(hdr, sizeof(hdr), "PAD %d > PEAK WINDOW", _selectedEditPad + 1); u8g2.drawStr(5, 10, hdr); u8g2.drawLine(0, 13, 127, 13); u8g2.drawStr(10, 30, "Peak Track Window:");
        xSemaphoreTake(settingsMutex, portMAX_DELAY); uint16_t currentPeak = padPeakWindowMs[_selectedEditPad]; xSemaphoreGive(settingsMutex);
        char valStr[15]; snprintf(valStr, sizeof(valStr), "%d ms", currentPeak); u8g2.setFont(u8g2_font_ncenB12_tr); u8g2.drawStr(15, 52, valStr);
        u8g2.drawFrame(70, 42, 52, 10); u8g2.drawBox(71, 43, constrain(map(currentPeak, 2, 30, 0, 50), 0, 50), 8); u8g2.sendBuffer();
    }
    void showSetCrosstalk() {
        u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tr); char hdr[24]; snprintf(hdr, sizeof(hdr), "PAD %d > CROSSTALK", _selectedEditPad + 1); u8g2.drawStr(5, 10, hdr); u8g2.drawLine(0, 13, 127, 13); u8g2.drawStr(10, 30, "Crosstalk Reject:");
        xSemaphoreTake(settingsMutex, portMAX_DELAY); uint16_t currentCross = padCrosstalkLockMs[_selectedEditPad]; xSemaphoreGive(settingsMutex);
        char valStr[15]; snprintf(valStr, sizeof(valStr), "%d ms", currentCross); u8g2.setFont(u8g2_font_ncenB12_tr); u8g2.drawStr(15, 52, valStr);
        u8g2.drawFrame(70, 42, 52, 10); u8g2.drawBox(71, 43, constrain(map(currentCross, 2, 150, 0, 50), 0, 50), 8); u8g2.sendBuffer();
    }

    uint8_t getPadCrosstalkLevel(uint8_t pad) {
        uint16_t total = 0;
        uint8_t count = 0;
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        uint8_t activeCount = (uint8_t)constrain((int)globalPadCount, 1, MAX_PADS_POSSIBLE);
        if (pad < activeCount) {
            for (uint8_t other = 0; other < activeCount; other++) {
                if (other == pad) continue;
                total += crosstalkMatrix[pad][other];
                count++;
            }
        }
        xSemaphoreGive(settingsMutex);

        if (count == 0) return CROSSTALK_MATRIX_DEFAULT_PERCENT;
        uint16_t average = (total + (count / 2)) / count;
        return (uint8_t)constrain((int)(((average + 5) / 10) * 10), 0, 100);
    }

    void showSetCrosstalkLevel() {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tr);
        char hdr[24];
        snprintf(hdr, sizeof(hdr), "PAD %d > CT LEVEL", _selectedEditPad + 1);
        u8g2.drawStr(5, 10, hdr);
        u8g2.drawLine(0, 13, 127, 13);
        u8g2.drawStr(8, 28, "CROSSTALK CANCEL");
        uint8_t level = getPadCrosstalkLevel(_selectedEditPad);
        char valStr[12];
        snprintf(valStr, sizeof(valStr), "%u%%", (unsigned)level);
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(15, 51, valStr);
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawStr(61, 54, "TURN: 50 60 70 80");
        u8g2.sendBuffer();
    }
    void showLoading(int kitNum) { u8g2.clearBuffer(); u8g2.drawFrame(0, 0, 128, 64); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(35, 22, "Loading..."); u8g2.setFont(u8g2_font_ncenB14_tr); String buf = getKitDisplayName(kitNum); u8g2.drawStr((128 - u8g2.getStrWidth(buf.c_str())) / 2, 48, buf.c_str()); u8g2.sendBuffer(); }
    void showError(const char* msg) { u8g2.clearBuffer(); u8g2.setFont(u8g2_font_ncenB10_tr); u8g2.drawStr(30, 20, "! ERROR !"); u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(5, 40, msg); u8g2.sendBuffer(); }

    int update() {
        int encoderChange = 0; int stateKitAction = 0; bool upNow = digitalRead(BTN_KIT_UP_PIN); bool downNow = digitalRead(BTN_KIT_DOWN_PIN); bool backNow = digitalRead(BTN_BACK_PIN); bool pshNow = digitalRead(ENC_BTN_PIN); uint32_t currentMillis = millis();
        int8_t encA = digitalRead(ENC_A_PIN); 
        
        if (encA != _encLastA) { 
            if (digitalRead(ENC_B_PIN) != encA) { _encAccum++; } else { _encAccum--; }
            _encLastA = encA; 
            if (_encAccum >= 2) { encoderChange = +1; _encAccum = 0; } 
            else if (_encAccum <= -2) { encoderChange = -1; _encAccum = 0; } 
        }
        
        if (_currentScreen != SCREEN_MAIN && _currentScreen != SCREEN_EDIT_WAV_SEL) {
            if (_btnUpPrev == HIGH && upNow == LOW) { encoderChange = +1; }
            if (_btnDownPrev == HIGH && downNow == LOW) { encoderChange = -1; }
        }

        int kitBtnToneChange = 0;
        if (_currentScreen == SCREEN_EDIT_WAV_SEL) {
            if (upNow == LOW) { if (_btnUpPrev == HIGH) { _btnUpPressedTime = currentMillis; _isUpLongPress = false; kitBtnToneChange = +1; } else if (!_isUpLongPress && (currentMillis - _btnUpPressedTime >= 400)) { _isUpLongPress = true; _lastScrollTime = currentMillis; } if (_isUpLongPress && (currentMillis - _lastScrollTime >= 60)) { kitBtnToneChange = +1; _lastScrollTime = currentMillis; } }
            if (downNow == LOW) { if (_btnDownPrev == HIGH) { _btnDownPressedTime = currentMillis; _isDownLongPress = false; kitBtnToneChange = -1; } else if (!_isDownLongPress && (currentMillis - _btnDownPressedTime >= 400)) { _isDownLongPress = true; _lastScrollTime = currentMillis; } if (_isDownLongPress && (currentMillis - _lastScrollTime >= 60)) { kitBtnToneChange = -1; _lastScrollTime = currentMillis; } }
        }
        
        uint8_t activePadsCount = 8;
        if (xSemaphoreTake(settingsMutex, 0) == pdTRUE) { activePadsCount = globalPadCount; xSemaphoreGive(settingsMutex); }

        switch (_currentScreen) {
            case SCREEN_MAIN: 
                if (_btnUpPrev == HIGH && upNow == LOW) stateKitAction = +1; 
                if (_btnDownPrev == HIGH && downNow == LOW) stateKitAction = -1; 
                if (encoderChange != 0) stateKitAction = encoderChange; 
                if (_btnEncPrev == HIGH && pshNow == LOW) { _currentScreen = SCREEN_MENU_LIST; _menuIdx = 0; _menuScrollWinStart = 0; } 
                break;

            case SCREEN_MENU_LIST: 
                if (encoderChange != 0) _menuIdx = constrain(_menuIdx + encoderChange, 0, 13); 
                if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MAIN; 
                if (_btnEncPrev == HIGH && pshNow == LOW) { 
                    if (_menuIdx == 0) { _currentScreen = SCREEN_PAD_SENS_SEL; _selectedEditPad = 0; } 
                    if (_menuIdx == 1) _currentScreen = SCREEN_SET_VOL; 
                    if (_menuIdx == 2) _currentScreen = SCREEN_SET_VELOCITY; 
                    if (_menuIdx == 3) _currentScreen = SCREEN_SET_MODE; 
                    if (_menuIdx == 4) { _currentScreen = SCREEN_EDIT_PAD_SEL; _selectedEditPad = 0; } 
                    if (_menuIdx == 5) { _currentScreen = SCREEN_PAD_VOL_SEL; _selectedEditPad = 0; } 
                    if (_menuIdx == 6) _currentScreen = SCREEN_SET_LAYOUT; 
                    if (_menuIdx == 7) { _currentScreen = SCREEN_ADV_PAD_SEL; _selectedEditPad = 0; } 
                    if (_menuIdx == 8) { _currentScreen = SCREEN_PAD_FILT_SEL; _selectedEditPad = 0; } 
                    if (_menuIdx == 9) { _currentScreen = SCREEN_WEB_SERVER; startWirelessHost(); } 
                    if (_menuIdx == 10) { _currentScreen = SCREEN_PAD_PITCH_SEL; _selectedEditPad = 0; }
                    if (_menuIdx == 11) { _currentScreen = SCREEN_PAD_PAN_SEL; _selectedEditPad = 0; }
                    if (_menuIdx == 12) { _currentScreen = SCREEN_CHAIN_SEL; _selectedChainStep = 0; }
                    if (_menuIdx == 13) { _currentScreen = SCREEN_CHAIN_MODE; }
                } 
                break;
            
            case SCREEN_PAD_SENS_SEL: 
                if (encoderChange != 0) _selectedEditPad = constrain(_selectedEditPad + encoderChange, 0, activePadsCount - 1); 
                if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MENU_LIST; 
                if (_btnEncPrev == HIGH && pshNow == LOW) { _currentScreen = SCREEN_PAD_SENS_SET; } 
                break;
            case SCREEN_PAD_SENS_SET: 
                if (encoderChange != 0) { 
                    xSemaphoreTake(settingsMutex, portMAX_DELAY); 
                    padThreshold[_selectedEditPad] = constrain((int)padThreshold[_selectedEditPad] + (encoderChange * 50), 200, 2000); 
                    xSemaphoreGive(settingsMutex); 
                } 
                if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_PAD_SENS_SEL; 
                if (_btnEncPrev == HIGH && pshNow == LOW) { saveKitMappingsToSD(); _currentScreen = SCREEN_PAD_SENS_SEL; } 
                break;
            case SCREEN_SET_VOL: if (encoderChange != 0) { xSemaphoreTake(settingsMutex, portMAX_DELAY); globalMasterVolume = constrain(globalMasterVolume + (encoderChange * 0.05f), 0.0f, 1.0f); xSemaphoreGive(settingsMutex); } if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MENU_LIST; if (_btnEncPrev == HIGH && pshNow == LOW) { saveKitMappingsToSD(); _currentScreen = SCREEN_MENU_LIST; } break;
            
            case SCREEN_SET_VELOCITY: 
                if (encoderChange != 0) { 
                    xSemaphoreTake(settingsMutex, portMAX_DELAY); 
                    if (!globalDynamicVelocity) globalFixedVelocity = constrain((int)globalFixedVelocity + encoderChange, 1, 127); 
                    xSemaphoreGive(settingsMutex); 
                } 
                if (_btnBackPrev == HIGH && backNow == LOW) { saveKitMappingsToSD(); _currentScreen = SCREEN_MENU_LIST; } 
                if (_btnEncPrev == HIGH && pshNow == LOW) { 
                    xSemaphoreTake(settingsMutex, portMAX_DELAY); 
                    globalDynamicVelocity = !globalDynamicVelocity; 
                    xSemaphoreGive(settingsMutex); 
                } 
                break;
                
            case SCREEN_SET_MODE: if (encoderChange != 0) { xSemaphoreTake(settingsMutex, portMAX_DELAY); int cm = (int)globalSystemMode + encoderChange; if(cm < 0) cm = 2; if(cm > 2) cm = 0; globalSystemMode = (SysMode)cm; xSemaphoreGive(settingsMutex); } if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MENU_LIST; if (_btnEncPrev == HIGH && pshNow == LOW) { saveKitMappingsToSD(); _currentScreen = SCREEN_MENU_LIST; } break;
            case SCREEN_EDIT_PAD_SEL: if (encoderChange != 0) _selectedEditPad = constrain(_selectedEditPad + encoderChange, 0, activePadsCount - 1); if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MENU_LIST; if (_btnEncPrev == HIGH && pshNow == LOW) { _currentScreen = SCREEN_EDIT_LAYER_SEL; _selectedLayer = 0; } break;
            case SCREEN_EDIT_LAYER_SEL: if (encoderChange != 0) _selectedLayer = constrain(_selectedLayer + encoderChange, 0, 1); if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_EDIT_PAD_SEL; if (_btnEncPrev == HIGH && pshNow == LOW) { _currentScreen = SCREEN_EDIT_WAV_SEL; _selectedToneNum = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].layerA_ToneNum : kitAssignments[_currentKit][_selectedEditPad].layerB_ToneNum; _previewTriggered = false; } break;
            
            case SCREEN_EDIT_WAV_SEL: 
                if (encoderChange != 0 || kitBtnToneChange != 0) { 
                    int prevTone = _selectedToneNum;
                    if (encoderChange != 0) _selectedToneNum = constrain((int)_selectedToneNum + encoderChange, 0, MAX_TONE_INDEX); 
                    if (kitBtnToneChange != 0) _selectedToneNum = constrain((int)_selectedToneNum + kitBtnToneChange, 0, MAX_TONE_INDEX); 
                    if (_selectedToneNum != prevTone) { _lastToneChangeTime = currentMillis; _previewTriggered = false; }
                } 
                if (!_previewTriggered && (currentMillis - _lastToneChangeTime >= 200)) {
                    int8_t activeFilt = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].padFilterA : kitAssignments[_currentKit][_selectedEditPad].padFilterB;
                    audioEngine.playPreviewWAV(_selectedToneNum, activeFilt);
                    _previewTriggered = true; 
                }
                if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_EDIT_LAYER_SEL; 
                if (_btnEncPrev == HIGH && pshNow == LOW) { 
                    if (_selectedLayer == 0) kitAssignments[_currentKit][_selectedEditPad].layerA_ToneNum = _selectedToneNum; else kitAssignments[_currentKit][_selectedEditPad].layerB_ToneNum = _selectedToneNum; 
                    saveKitMappingsToSD(); stateKitAction = 99; _currentScreen = SCREEN_EDIT_PAD_SEL; 
                } 
                break;

            case SCREEN_PAD_VOL_SEL: if (encoderChange != 0) _selectedEditPad = constrain(_selectedEditPad + encoderChange, 0, activePadsCount - 1); if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MENU_LIST; if (_btnEncPrev == HIGH && pshNow == LOW) { _currentScreen = SCREEN_PAD_VOL_SET; _selectedLayer = 0; _selectedPadVolPercent = kitAssignments[_currentKit][_selectedEditPad].padVolumePercentA; } break;
            case SCREEN_PAD_VOL_SET: if (encoderChange != 0) { _selectedPadVolPercent = constrain((int)_selectedPadVolPercent + (encoderChange * 5), 0, 100); if (_selectedLayer == 0) kitAssignments[_currentKit][_selectedEditPad].padVolumePercentA = _selectedPadVolPercent; else kitAssignments[_currentKit][_selectedEditPad].padVolumePercentB = _selectedPadVolPercent; } if (_btnEncPrev == HIGH && pshNow == LOW) { _selectedLayer = (_selectedLayer == 0) ? 1 : 0; _selectedPadVolPercent = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].padVolumePercentA : kitAssignments[_currentKit][_selectedEditPad].padVolumePercentB; } if (_btnBackPrev == HIGH && backNow == LOW) { saveKitMappingsToSD(); _currentScreen = SCREEN_PAD_VOL_SEL; } break;
            
            case SCREEN_PAD_FILT_SEL: 
                if (encoderChange != 0) _selectedEditPad = constrain(_selectedEditPad + encoderChange, 0, activePadsCount - 1); 
                if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MENU_LIST; 
                if (_btnEncPrev == HIGH && pshNow == LOW) { _currentScreen = SCREEN_PAD_FILT_SET; _selectedLayer = 0; _selectedPadFiltVal = kitAssignments[_currentKit][_selectedEditPad].padFilterA; } 
                break;
                
            case SCREEN_PAD_FILT_SET: 
                if (encoderChange != 0) { 
                    _selectedPadFiltVal = constrain((int)_selectedPadFiltVal + encoderChange, -50, 50); 
                    if (_selectedLayer == 0) kitAssignments[_currentKit][_selectedEditPad].padFilterA = _selectedPadFiltVal; 
                    else kitAssignments[_currentKit][_selectedEditPad].padFilterB = _selectedPadFiltVal;
                    uint16_t currentTone = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].layerA_ToneNum : kitAssignments[_currentKit][_selectedEditPad].layerB_ToneNum;
                    audioEngine.playPreviewWAV(currentTone, _selectedPadFiltVal);
                } 
                if (_btnEncPrev == HIGH && pshNow == LOW) { 
                    _selectedLayer = (_selectedLayer == 0) ? 1 : 0; 
                    _selectedPadFiltVal = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].padFilterA : kitAssignments[_currentKit][_selectedEditPad].padFilterB; 
                } 
                if (_btnBackPrev == HIGH && backNow == LOW) { saveKitMappingsToSD(); stateKitAction = 99; _currentScreen = SCREEN_PAD_FILT_SEL; } 
                break;

            case SCREEN_PAD_PITCH_SEL:
                if (encoderChange != 0) _selectedEditPad = constrain(_selectedEditPad + encoderChange, 0, activePadsCount - 1);
                if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MENU_LIST;
                if (_btnEncPrev == HIGH && pshNow == LOW) { _currentScreen = SCREEN_PAD_PITCH_SET; _selectedLayer = 0; _selectedPadPitchVal = kitAssignments[_currentKit][_selectedEditPad].padPitchA; }
                break;

            case SCREEN_PAD_PITCH_SET:
                if (encoderChange != 0) {
                    _selectedPadPitchVal = constrain((int)_selectedPadPitchVal + encoderChange, -12, 12);
                    if (_selectedLayer == 0) kitAssignments[_currentKit][_selectedEditPad].padPitchA = _selectedPadPitchVal;
                    else kitAssignments[_currentKit][_selectedEditPad].padPitchB = _selectedPadPitchVal;
                    uint16_t currentTone = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].layerA_ToneNum : kitAssignments[_currentKit][_selectedEditPad].layerB_ToneNum;
                    int8_t currentFilt = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].padFilterA : kitAssignments[_currentKit][_selectedEditPad].padFilterB;
                    int8_t currentPan = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].padPanA : kitAssignments[_currentKit][_selectedEditPad].padPanB;
                    audioEngine.playPreviewWAV(currentTone, currentFilt, _selectedPadPitchVal, currentPan);
                }
                if (_btnEncPrev == HIGH && pshNow == LOW) {
                    _selectedLayer = (_selectedLayer == 0) ? 1 : 0;
                    _selectedPadPitchVal = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].padPitchA : kitAssignments[_currentKit][_selectedEditPad].padPitchB;
                }
                if (_btnBackPrev == HIGH && backNow == LOW) { saveKitMappingsToSD(); stateKitAction = 99; _currentScreen = SCREEN_PAD_PITCH_SEL; }
                break;

            case SCREEN_PAD_PAN_SEL:
                if (encoderChange != 0) _selectedEditPad = constrain(_selectedEditPad + encoderChange, 0, activePadsCount - 1);
                if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MENU_LIST;
                if (_btnEncPrev == HIGH && pshNow == LOW) { _currentScreen = SCREEN_PAD_PAN_SET; _selectedLayer = 0; _selectedPadPanVal = kitAssignments[_currentKit][_selectedEditPad].padPanA; }
                break;

            case SCREEN_PAD_PAN_SET:
                if (encoderChange != 0) {
                    _selectedPadPanVal = constrain((int)_selectedPadPanVal + (encoderChange * 5), -100, 100);
                    if (_selectedLayer == 0) kitAssignments[_currentKit][_selectedEditPad].padPanA = _selectedPadPanVal;
                    else kitAssignments[_currentKit][_selectedEditPad].padPanB = _selectedPadPanVal;
                    uint16_t currentTone = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].layerA_ToneNum : kitAssignments[_currentKit][_selectedEditPad].layerB_ToneNum;
                    int8_t currentFilt = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].padFilterA : kitAssignments[_currentKit][_selectedEditPad].padFilterB;
                    int8_t currentPitch = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].padPitchA : kitAssignments[_currentKit][_selectedEditPad].padPitchB;
                    audioEngine.playPreviewWAV(currentTone, currentFilt, currentPitch, _selectedPadPanVal);
                }
                if (_btnEncPrev == HIGH && pshNow == LOW) {
                    _selectedLayer = (_selectedLayer == 0) ? 1 : 0;
                    _selectedPadPanVal = (_selectedLayer == 0) ? kitAssignments[_currentKit][_selectedEditPad].padPanA : kitAssignments[_currentKit][_selectedEditPad].padPanB;
                }
                if (_btnBackPrev == HIGH && backNow == LOW) { saveKitMappingsToSD(); stateKitAction = 99; _currentScreen = SCREEN_PAD_PAN_SEL; }
                break;

            case SCREEN_CHAIN_SEL:
                if (encoderChange != 0) {
                    xSemaphoreTake(settingsMutex, portMAX_DELAY);
                    globalActiveChain = constrain((int)globalActiveChain + encoderChange, 1, (int)MAX_CHAINS);
                    xSemaphoreGive(settingsMutex);
                }
                if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MENU_LIST;
                if (_btnEncPrev == HIGH && pshNow == LOW) { _currentScreen = SCREEN_CHAIN_EDIT; _selectedChainStep = 0; }
                break;

            case SCREEN_CHAIN_EDIT:
                if (encoderChange != 0) {
                    xSemaphoreTake(settingsMutex, portMAX_DELAY);
                    int cur = (int)kitChains[globalActiveChain][_selectedChainStep];
                    kitChains[globalActiveChain][_selectedChainStep] = (uint8_t)constrain(cur + encoderChange, 0, (int)MAX_KITS);
                    xSemaphoreGive(settingsMutex);
                }
                if (_btnEncPrev == HIGH && pshNow == LOW) { _selectedChainStep = (_selectedChainStep + 1) % MAX_CHAIN_STEPS; }
                if (_btnBackPrev == HIGH && backNow == LOW) { saveChainsToSD(); _currentScreen = SCREEN_CHAIN_SEL; }
                break;

            case SCREEN_CHAIN_MODE:
                if (_btnBackPrev == HIGH && backNow == LOW) { saveChainsToSD(); _currentScreen = SCREEN_MENU_LIST; }
                if (_btnEncPrev == HIGH && pshNow == LOW) {
                    xSemaphoreTake(settingsMutex, portMAX_DELAY);
                    globalChainModeActive = !globalChainModeActive;
                    xSemaphoreGive(settingsMutex);
                }
                break;

            case SCREEN_SET_LAYOUT: if (encoderChange != 0) { xSemaphoreTake(settingsMutex, portMAX_DELAY); globalPadCount = (globalPadCount == 8) ? 10 : 8; xSemaphoreGive(settingsMutex); } if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MENU_LIST; if (_btnEncPrev == HIGH && pshNow == LOW) { saveKitMappingsToSD(); stateKitAction = 99; _currentScreen = SCREEN_MENU_LIST; } break;
            case SCREEN_ADV_PAD_SEL:
                if (encoderChange != 0) _selectedEditPad = constrain(_selectedEditPad + encoderChange, 0, activePadsCount - 1);
                if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_MENU_LIST;
                if (_btnEncPrev == HIGH && pshNow == LOW) { _currentScreen = SCREEN_ADV_TUNE_LIST; _advMenuIdx = 0; }
                break;
            case SCREEN_ADV_TUNE_LIST: if (encoderChange != 0) _advMenuIdx = constrain(_advMenuIdx + encoderChange, 0, 3); if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_ADV_PAD_SEL; if (_btnEncPrev == HIGH && pshNow == LOW) { if (_advMenuIdx == 0) _currentScreen = SCREEN_SET_DEBOUNCE; if (_advMenuIdx == 1) _currentScreen = SCREEN_SET_PEAKWIN; if (_advMenuIdx == 2) _currentScreen = SCREEN_SET_CROSSTALK; if (_advMenuIdx == 3) _currentScreen = SCREEN_SET_CROSSTALK_LEVEL; } break;
            case SCREEN_SET_DEBOUNCE: if (encoderChange != 0) { xSemaphoreTake(settingsMutex, portMAX_DELAY); padDebounceMs[_selectedEditPad] = constrain((int)padDebounceMs[_selectedEditPad] + (encoderChange * 5), 10, 300); xSemaphoreGive(settingsMutex); } if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_ADV_TUNE_LIST; if (_btnEncPrev == HIGH && pshNow == LOW) { saveKitMappingsToSD(); _currentScreen = SCREEN_ADV_TUNE_LIST; } break;
            case SCREEN_SET_PEAKWIN: if (encoderChange != 0) { xSemaphoreTake(settingsMutex, portMAX_DELAY); padPeakWindowMs[_selectedEditPad] = constrain((int)padPeakWindowMs[_selectedEditPad] + encoderChange, 2, 30); xSemaphoreGive(settingsMutex); } if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_ADV_TUNE_LIST; if (_btnEncPrev == HIGH && pshNow == LOW) { saveKitMappingsToSD(); _currentScreen = SCREEN_ADV_TUNE_LIST; } break;
            case SCREEN_SET_CROSSTALK: if (encoderChange != 0) { xSemaphoreTake(settingsMutex, portMAX_DELAY); padCrosstalkLockMs[_selectedEditPad] = constrain((int)padCrosstalkLockMs[_selectedEditPad] + encoderChange, 2, 150); xSemaphoreGive(settingsMutex); } if (_btnBackPrev == HIGH && backNow == LOW) _currentScreen = SCREEN_ADV_TUNE_LIST; if (_btnEncPrev == HIGH && pshNow == LOW) { saveKitMappingsToSD(); _currentScreen = SCREEN_ADV_TUNE_LIST; } break;
            case SCREEN_SET_CROSSTALK_LEVEL:
                if (encoderChange != 0) {
                    int currentLevel = (int)getPadCrosstalkLevel(_selectedEditPad);
                    int nextLevel = currentLevel;
                    if (encoderChange > 0) {
                        nextLevel = (currentLevel < 50) ? 50 : min(80, currentLevel + 10);
                    } else {
                        nextLevel = (currentLevel > 80) ? 80 : max(50, currentLevel - 10);
                    }

                    xSemaphoreTake(settingsMutex, portMAX_DELAY);
                    uint8_t activeCount = (uint8_t)constrain((int)globalPadCount, 1, MAX_PADS_POSSIBLE);
                    if (_selectedEditPad < activeCount) {
                        for (uint8_t other = 0; other < activeCount; other++) {
                            if (other != _selectedEditPad) {
                                crosstalkMatrix[_selectedEditPad][other] = (uint8_t)nextLevel;
                            }
                        }
                    }
                    xSemaphoreGive(settingsMutex);
                }
                if (_btnBackPrev == HIGH && backNow == LOW) {
                    _currentScreen = SCREEN_ADV_TUNE_LIST;
                }
                if (_btnEncPrev == HIGH && pshNow == LOW) {
                    saveKitMappingsToSD();
                    _currentScreen = SCREEN_ADV_TUNE_LIST;
                }
                break;
            
            case SCREEN_WEB_SERVER:
                if (_btnBackPrev == HIGH && backNow == LOW) {
                    stopWirelessHost();
                    stateKitAction = 99; 
                    _currentScreen = SCREEN_MENU_LIST;
                }
                break;
        }
        _btnUpPrev = upNow; _btnDownPrev = downNow; _btnBackPrev = backNow; _btnEncPrev = pshNow; return stateKitAction;
    }

    void refreshRender() {
        switch (_currentScreen) {
            case SCREEN_MAIN: showMain(); break; case SCREEN_MENU_LIST: showMenuList(); break; case SCREEN_PAD_SENS_SEL: showPadSensSel(); break; case SCREEN_PAD_SENS_SET: showPadSensSet(); break; case SCREEN_SET_VOL: showSetVol(); break; case SCREEN_SET_VELOCITY: showSetVelocity(); break; 
            case SCREEN_SET_MODE: showSetMode(); break; case SCREEN_EDIT_PAD_SEL: showEditPadSel(); break; case SCREEN_EDIT_LAYER_SEL: showEditLayerSel(); break; case SCREEN_EDIT_WAV_SEL: showEditWavSel(); break;
            case SCREEN_PAD_VOL_SEL: showPadVolSel(); break; case SCREEN_PAD_VOL_SET: showPadVolSet(); break; case SCREEN_SET_LAYOUT: showSetLayout(); break;
            case SCREEN_ADV_PAD_SEL: showAdvPadSel(); break; case SCREEN_ADV_TUNE_LIST: showAdvTuneList(); break; case SCREEN_SET_DEBOUNCE: showSetDebounce(); break; case SCREEN_SET_PEAKWIN: showSetPeakWin(); break; case SCREEN_SET_CROSSTALK: showSetCrosstalk(); break; case SCREEN_SET_CROSSTALK_LEVEL: showSetCrosstalkLevel(); break; 
            case SCREEN_PAD_FILT_SEL: showPadFiltSel(); break; case SCREEN_PAD_FILT_SET: showPadFiltSet(); break; 
            case SCREEN_PAD_PITCH_SEL: showPadPitchSel(); break; case SCREEN_PAD_PITCH_SET: showPadPitchSet(); break;
            case SCREEN_PAD_PAN_SEL: showPadPanSel(); break; case SCREEN_PAD_PAN_SET: showPadPanSet(); break;
            case SCREEN_CHAIN_SEL: showChainSel(); break; case SCREEN_CHAIN_EDIT: showChainEdit(); break; case SCREEN_CHAIN_MODE: showChainMode(); break;
            case SCREEN_WEB_SERVER: showWebServerScreen(); break; 
        }
    }
    MenuScreen getCurrentScreen() const { return _currentScreen; }
    void setKit(int k) { _currentKit = k; }
    int getKit() const { return _currentKit; }
    void setPadActive(uint8_t pad) { _padActive |= (1 << pad); }
    void clearPadActive() { _padActive = 0; }

private:
    int _currentKit; uint16_t _padActive; bool _btnUpPrev, _btnDownPrev, _btnBackPrev, _btnEncPrev; int8_t _encLastA; int _encAccum; MenuScreen _currentScreen; int _menuIdx; int _menuScrollWinStart; int _selectedEditPad; int _selectedLayer; uint16_t _selectedToneNum; uint8_t _selectedPadVolPercent;
    uint32_t _btnUpPressedTime; uint32_t _btnDownPressedTime; uint32_t _lastScrollTime; bool _isUpLongPress; bool _isDownLongPress;
    int _advMenuIdx; 
    uint32_t _lastToneChangeTime; bool _previewTriggered;
    int8_t  _selectedPadFiltVal; 
    int8_t  _selectedPadPitchVal;
    int8_t  _selectedPadPanVal;
    int8_t  _selectedChainStep;
};

// === INTER-CORE INSTANCES ===
AudioEngine audioEngine; PiezoHandler piezo; UIHandler ui;

void broadcastWsEvent(const char* type, String payload) {
    if (ws.count() > 0) {
        String json = "{\"type\":\"" + String(type) + "\"," + payload + "}";
        ws.textAll(json);
    }
}

// CORS Headers Injector for GitHub External PWA Client
void addCORSHeaders(AsyncWebServerResponse *response) {
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    response->addHeader("Access-Control-Allow-Headers", "*");
}

void startWirelessHost() {
    webServerActive = true;
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false); // Fix Wi-Fi Disconnect / AP Auto Cut-off Issue
    WiFi.softAP("ERO_PAD_WIFI", "12345678"); 
    
    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {});
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html);
        addCORSHeaders(response); request->send(response);
    });

    server.on("/get-assignments", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
        int targetKit = ui.getKit();
        if (request->hasParam("kit")) targetKit = constrain(request->getParam("kit")->value().toInt(), MIN_KIT, (int)MAX_KITS);
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        for (int i = 0; i < MAX_PADS_POSSIBLE; i++) {
            json += "\"p" + String(i + 1) + "a\":" + String(kitAssignments[targetKit][i].layerA_ToneNum) + ",";
            json += "\"p" + String(i + 1) + "b\":" + String(kitAssignments[targetKit][i].layerB_ToneNum) + ",";
            json += "\"p" + String(i + 1) + "va\":" + String(kitAssignments[targetKit][i].padVolumePercentA) + ",";
            json += "\"p" + String(i + 1) + "vb\":" + String(kitAssignments[targetKit][i].padVolumePercentB) + ",";
            json += "\"p" + String(i + 1) + "fa\":" + String(kitAssignments[targetKit][i].padFilterA) + ",";
            json += "\"p" + String(i + 1) + "fb\":" + String(kitAssignments[targetKit][i].padFilterB) + ",";
            json += "\"p" + String(i + 1) + "rre\":" + String(kitAssignments[targetKit][i].roundRobinEnabled) + ",";
            json += "\"p" + String(i + 1) + "rr1\":" + String(kitAssignments[targetKit][i].rrToneNum[0]) + ",";
            json += "\"p" + String(i + 1) + "rr2\":" + String(kitAssignments[targetKit][i].rrToneNum[1]) + ",";
            json += "\"p" + String(i + 1) + "rr3\":" + String(kitAssignments[targetKit][i].rrToneNum[2]);
            if (i < MAX_PADS_POSSIBLE - 1) json += ",";
        }
        xSemaphoreGive(settingsMutex);
        json += "}";
        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", json);
        addCORSHeaders(response); request->send(response);
    });

    server.on("/save-assignments", HTTP_GET, [](AsyncWebServerRequest *request) {
        int targetKit = ui.getKit();
        if (request->hasParam("kit")) targetKit = constrain(request->getParam("kit")->value().toInt(), MIN_KIT, (int)MAX_KITS);
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        for (int i = 0; i < MAX_PADS_POSSIBLE; i++) {
            String argA = "p" + String(i + 1) + "a";
            String argB = "p" + String(i + 1) + "b";
            String argVA = "p" + String(i + 1) + "va";
            String argVB = "p" + String(i + 1) + "vb";
            String argFA = "p" + String(i + 1) + "fa";
            String argFB = "p" + String(i + 1) + "fb";
            if (request->hasParam(argA)) kitAssignments[targetKit][i].layerA_ToneNum = request->getParam(argA)->value().toInt();
            if (request->hasParam(argB)) kitAssignments[targetKit][i].layerB_ToneNum = request->getParam(argB)->value().toInt();
            if (request->hasParam(argVA)) kitAssignments[targetKit][i].padVolumePercentA = constrain(request->getParam(argVA)->value().toInt(), 0, 100);
            if (request->hasParam(argVB)) kitAssignments[targetKit][i].padVolumePercentB = constrain(request->getParam(argVB)->value().toInt(), 0, 100);
            if (request->hasParam(argFA)) kitAssignments[targetKit][i].padFilterA = constrain(request->getParam(argFA)->value().toInt(), -50, 50);
            if (request->hasParam(argFB)) kitAssignments[targetKit][i].padFilterB = constrain(request->getParam(argFB)->value().toInt(), -50, 50);
            String argRRE = "p" + String(i + 1) + "rre";
            String argRR1 = "p" + String(i + 1) + "rr1";
            String argRR2 = "p" + String(i + 1) + "rr2";
            String argRR3 = "p" + String(i + 1) + "rr3";
            if (request->hasParam(argRRE)) kitAssignments[targetKit][i].roundRobinEnabled = request->getParam(argRRE)->value().toInt() ? 1 : 0;
            if (request->hasParam(argRR1)) kitAssignments[targetKit][i].rrToneNum[0] = request->getParam(argRR1)->value().toInt();
            if (request->hasParam(argRR2)) kitAssignments[targetKit][i].rrToneNum[1] = request->getParam(argRR2)->value().toInt();
            if (request->hasParam(argRR3)) kitAssignments[targetKit][i].rrToneNum[2] = request->getParam(argRR3)->value().toInt();
        }
        xSemaphoreGive(settingsMutex);
        saveKitMappingsToSD();
        audioEngine.loadKit(targetKit);
        ui.setKit(targetKit);
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        addCORSHeaders(response); request->send(response);
    });

    server.on("/select-kit", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("kit")) { AsyncWebServerResponse *response = request->beginResponse(400, "text/plain", "Missing kit"); addCORSHeaders(response); request->send(response); return; }
        int kitNum = constrain(request->getParam("kit")->value().toInt(), MIN_KIT, (int)MAX_KITS);
        ui.showLoading(kitNum);
        audioEngine.loadKit(kitNum);
        ui.setKit(kitNum);
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        addCORSHeaders(response); request->send(response);
    });

    server.on("/get-current-kit", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", String(ui.getKit()));
        addCORSHeaders(response); request->send(response);
    });

    server.on("/get-chain", HTTP_GET, [](AsyncWebServerRequest *request) {
        int chainNum = 1;
        if (request->hasParam("chain")) chainNum = constrain(request->getParam("chain")->value().toInt(), 1, (int)MAX_CHAINS);
        String json = "{\"steps\":[";
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        for (int s = 0; s < MAX_CHAIN_STEPS; s++) {
            json += String(kitChains[chainNum][s]);
            if (s < MAX_CHAIN_STEPS - 1) json += ",";
        }
        xSemaphoreGive(settingsMutex);
        json += "]}";
        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", json);
        addCORSHeaders(response); request->send(response);
    });

    server.on("/save-chain", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("chain")) { AsyncWebServerResponse *response = request->beginResponse(400, "text/plain", "Missing chain"); addCORSHeaders(response); request->send(response); return; }
        int chainNum = constrain(request->getParam("chain")->value().toInt(), 1, (int)MAX_CHAINS);
        String stepsStr = request->hasParam("steps") ? request->getParam("steps")->value() : "";
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        for (int s = 0; s < MAX_CHAIN_STEPS; s++) kitChains[chainNum][s] = 0;
        int stepIdx = 0, startPos = 0;
        while (stepIdx < MAX_CHAIN_STEPS && startPos <= (int)stepsStr.length()) {
            int commaPos = stepsStr.indexOf(',', startPos);
            String piece = (commaPos == -1) ? stepsStr.substring(startPos) : stepsStr.substring(startPos, commaPos);
            piece.trim();
            if (piece.length() > 0) kitChains[chainNum][stepIdx++] = (uint8_t)constrain(piece.toInt(), 0, (int)MAX_KITS);
            if (commaPos == -1) break;
            startPos = commaPos + 1;
        }
        xSemaphoreGive(settingsMutex);
        saveChainsToSD();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        addCORSHeaders(response); request->send(response);
    });

    server.on("/get-chain-mode", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{\"active\":" + String(globalChainModeActive ? "true" : "false") + ",\"chain\":" + String(globalActiveChain) + "}";
        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", json);
        addCORSHeaders(response); request->send(response);
    });

    server.on("/set-chain-mode", HTTP_GET, [](AsyncWebServerRequest *request) {
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        if (request->hasParam("on")) globalChainModeActive = (request->getParam("on")->value().toInt() != 0);
        if (request->hasParam("chain")) globalActiveChain = constrain(request->getParam("chain")->value().toInt(), 1, (int)MAX_CHAINS);
        xSemaphoreGive(settingsMutex);
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        addCORSHeaders(response); request->send(response);
    });

    server.on("/chain-step", HTTP_GET, [](AsyncWebServerRequest *request) {
        int dir = 1;
        if (request->hasParam("dir")) dir = (request->getParam("dir")->value().toInt() < 0) ? -1 : 1;
        int stepped = stepChain((int8_t)dir);
        if (stepped > 0) { ui.showLoading(stepped); audioEngine.loadKit(stepped); ui.setKit(stepped); }
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", String(stepped > 0 ? stepped : ui.getKit()));
        addCORSHeaders(response); request->send(response);
    });

    server.on("/get-kit-name", HTTP_GET, [](AsyncWebServerRequest *request) {
        int kitNum = ui.getKit();
        if (request->hasParam("kit")) kitNum = constrain(request->getParam("kit")->value().toInt(), MIN_KIT, (int)MAX_KITS);
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", String(kitNames[kitNum]));
        addCORSHeaders(response); request->send(response);
    });

    server.on("/save-kit-name", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("kit")) { AsyncWebServerResponse *response = request->beginResponse(400, "text/plain", "Missing kit"); addCORSHeaders(response); request->send(response); return; }
        int kitNum = constrain(request->getParam("kit")->value().toInt(), MIN_KIT, (int)MAX_KITS);
        String nm = request->hasParam("name") ? request->getParam("name")->value() : "";
        nm.trim();
        if (nm.length() > KIT_NAME_MAXLEN) nm = nm.substring(0, KIT_NAME_MAXLEN);
        strncpy(kitNames[kitNum], nm.c_str(), KIT_NAME_MAXLEN);
        kitNames[kitNum][KIT_NAME_MAXLEN] = '\0';
        saveKitNamesToSD();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        addCORSHeaders(response); request->send(response);
    });

    server.on("/preview-tone", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("tone")) { AsyncWebServerResponse *response = request->beginResponse(400, "text/plain", "Missing tone"); addCORSHeaders(response); request->send(response); return; }
        uint16_t toneNum = request->getParam("tone")->value().toInt();
        int8_t filterVal = 0;
        if (request->hasParam("filter")) filterVal = request->getParam("filter")->value().toInt();
        audioEngine.playPreviewWAV(toneNum, filterVal);
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        addCORSHeaders(response); request->send(response);
    });

    server.on("/get-settings", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        JsonArray thArr = doc["padThreshold"].to<JsonArray>();
        JsonArray dbArr = doc["padDebounceMs"].to<JsonArray>();
        JsonArray pwArr = doc["padPeakWindowMs"].to<JsonArray>();
        JsonArray ctArr = doc["padCrosstalkLockMs"].to<JsonArray>();
        for (int p = 0; p < MAX_PADS_POSSIBLE; p++) {
            thArr.add(padThreshold[p]);
            dbArr.add(padDebounceMs[p]);
            pwArr.add(padPeakWindowMs[p]);
            ctArr.add(padCrosstalkLockMs[p]);
        }
        doc["masterVolume"] = globalMasterVolume;
        doc["fixedVelocity"] = globalFixedVelocity;
        doc["dynamicVelocity"] = globalDynamicVelocity ? 1 : 0;
        doc["systemMode"] = (int)globalSystemMode;
        doc["padCount"] = globalPadCount;
        xSemaphoreGive(settingsMutex);
        String jsonResponse;
        serializeJson(doc, jsonResponse);
        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", jsonResponse);
        addCORSHeaders(response); request->send(response);
    });

    server.on("/save-settings", HTTP_GET, [](AsyncWebServerRequest *request) {
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        for (int p = 0; p < MAX_PADS_POSSIBLE; p++) {
            String kTh = "p" + String(p + 1) + "th";
            String kDb = "p" + String(p + 1) + "db";
            String kPw = "p" + String(p + 1) + "pw";
            String kCt = "p" + String(p + 1) + "ct";
            if (request->hasParam(kTh)) padThreshold[p] = constrain(request->getParam(kTh)->value().toInt(), 200, 2000);
            if (request->hasParam(kDb)) padDebounceMs[p] = constrain(request->getParam(kDb)->value().toInt(), 10, 300);
            if (request->hasParam(kPw)) padPeakWindowMs[p] = constrain(request->getParam(kPw)->value().toInt(), 2, 30);
            if (request->hasParam(kCt)) padCrosstalkLockMs[p] = constrain(request->getParam(kCt)->value().toInt(), 2, 150);
        }
        if (request->hasParam("masterVolume")) globalMasterVolume = constrain(request->getParam("masterVolume")->value().toFloat(), 0.0f, 1.0f);
        if (request->hasParam("fixedVelocity")) globalFixedVelocity = constrain(request->getParam("fixedVelocity")->value().toInt(), 1, 127);
        if (request->hasParam("dynamicVelocity")) globalDynamicVelocity = (request->getParam("dynamicVelocity")->value().toInt() != 0);
        if (request->hasParam("systemMode")) {
            int sm = request->getParam("systemMode")->value().toInt();
            if (sm >= 0 && sm <= 2) globalSystemMode = (SysMode)sm;
        }
        if (request->hasParam("padCount")) {
            int pc = request->getParam("padCount")->value().toInt();
            globalPadCount = (pc == 10) ? 10 : 8;
        }
        xSemaphoreGive(settingsMutex);
        saveKitMappingsToSD();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        addCORSHeaders(response); request->send(response);
    });

    server.on("/get-crosstalk-matrix", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        JsonArray m = doc["matrix"].to<JsonArray>();
        for (int i = 0; i < MAX_PADS_POSSIBLE; i++)
            for (int j = 0; j < MAX_PADS_POSSIBLE; j++)
                m.add(crosstalkMatrix[i][j]);
        doc["padCount"] = globalPadCount;
        xSemaphoreGive(settingsMutex);
        String jsonResponse;
        serializeJson(doc, jsonResponse);
        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", jsonResponse);
        addCORSHeaders(response); request->send(response);
    });

    server.on("/save-crosstalk-matrix", HTTP_POST, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        addCORSHeaders(response); request->send(response);
    }, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        static String jsonBuf;
        if (index == 0) jsonBuf = "";
        jsonBuf.concat((const char*)data, len);
        if (index + len == total) {
            JsonDocument doc;
            if (!deserializeJson(doc, jsonBuf) && doc["matrix"].is<JsonArray>()) {
                JsonArray m = doc["matrix"];
                xSemaphoreTake(settingsMutex, portMAX_DELAY);
                int idx = 0;
                for (int i = 0; i < MAX_PADS_POSSIBLE; i++) {
                    for (int j = 0; j < MAX_PADS_POSSIBLE; j++) {
                        if ((size_t)idx < m.size()) {
                            crosstalkMatrix[i][j] = (i == j) ? 0 : constrain(m[idx].as<int>(), 0, 100);
                        }
                        idx++;
                    }
                }
                xSemaphoreGive(settingsMutex);
                saveKitMappingsToSD();
            }
        }
    });

    // JSON Backup Route
    server.on("/backup-json", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        JsonArray thArr = doc["padThreshold"].to<JsonArray>();
        JsonArray dbArr = doc["padDebounceMs"].to<JsonArray>();
        JsonArray pwArr = doc["padPeakWindowMs"].to<JsonArray>();
        JsonArray ctArr = doc["padCrosstalkLockMs"].to<JsonArray>();
        for (int p = 0; p < MAX_PADS_POSSIBLE; p++) {
            thArr.add(padThreshold[p]);
            dbArr.add(padDebounceMs[p]);
            pwArr.add(padPeakWindowMs[p]);
            ctArr.add(padCrosstalkLockMs[p]);
        }
        doc["masterVolume"] = globalMasterVolume;
        doc["systemMode"] = (int)globalSystemMode;
        doc["padCount"] = globalPadCount;
        doc["fixedVelocity"] = globalFixedVelocity;
        doc["dynamicVelocity"] = globalDynamicVelocity ? 1 : 0;
        JsonArray ctmArr = doc["crosstalkMatrix"].to<JsonArray>();
        for (int i = 0; i < MAX_PADS_POSSIBLE; i++)
            for (int j = 0; j < MAX_PADS_POSSIBLE; j++)
                ctmArr.add(crosstalkMatrix[i][j]);

        JsonArray kits = doc["kits"].to<JsonArray>();
        for (int k = 1; k <= MAX_KITS; k++) {
            JsonObject kitObj = kits.add<JsonObject>();
            kitObj["id"] = k;
            if (kitNames[k][0] != '\0') kitObj["name"] = kitNames[k];
            JsonArray pads = kitObj["pads"].to<JsonArray>();
            for (int p = 0; p < MAX_PADS_POSSIBLE; p++) {
                JsonObject padObj = pads.add<JsonObject>();
                padObj["la"] = kitAssignments[k][p].layerA_ToneNum;
                padObj["lb"] = kitAssignments[k][p].layerB_ToneNum;
                padObj["va"] = kitAssignments[k][p].padVolumePercentA;
                padObj["vb"] = kitAssignments[k][p].padVolumePercentB;
                padObj["fa"] = kitAssignments[k][p].padFilterA;
                padObj["fb"] = kitAssignments[k][p].padFilterB;
                padObj["pta"] = kitAssignments[k][p].padPitchA;
                padObj["ptb"] = kitAssignments[k][p].padPitchB;
                padObj["pna"] = kitAssignments[k][p].padPanA;
                padObj["pnb"] = kitAssignments[k][p].padPanB;
                padObj["rre"] = kitAssignments[k][p].roundRobinEnabled;
                padObj["rr1"] = kitAssignments[k][p].rrToneNum[0];
                padObj["rr2"] = kitAssignments[k][p].rrToneNum[1];
                padObj["rr3"] = kitAssignments[k][p].rrToneNum[2];
            }
        }
        xSemaphoreGive(settingsMutex);

        String jsonResponse;
        serializeJson(doc, jsonResponse);
        
        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", jsonResponse);
        response->addHeader("Content-Disposition", "attachment; filename=ERO_PAD_CONFIG.json");
        addCORSHeaders(response); request->send(response);
    });

    // JSON Restore Route
    server.on("/restore-json", HTTP_POST, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        addCORSHeaders(response); request->send(response);
    }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
        static String jsonBuf;
        if (index == 0) jsonBuf = "";
        for (size_t i = 0; i < len; i++) jsonBuf += (char)data[i];
        
        if (final) {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, jsonBuf);
            if (!err) {
                xSemaphoreTake(settingsMutex, portMAX_DELAY);
                if (doc["padThreshold"].is<JsonArray>()) {
                    JsonArray thArr = doc["padThreshold"];
                    int p = 0;
                    for (JsonVariant v : thArr) {
                        if (p >= MAX_PADS_POSSIBLE) break;
                        padThreshold[p] = constrain(v.as<int>(), 200, 2000);
                        p++;
                    }
                } else if (doc["threshold"]) {
                    uint16_t oldTh = doc["threshold"];
                    for (int p = 0; p < MAX_PADS_POSSIBLE; p++) padThreshold[p] = oldTh;
                    if (doc["pad10Threshold"]) padThreshold[9] = doc["pad10Threshold"];
                }
                if (doc["padDebounceMs"].is<JsonArray>()) {
                    JsonArray a = doc["padDebounceMs"]; int p = 0;
                    for (JsonVariant v : a) { if (p >= MAX_PADS_POSSIBLE) break; padDebounceMs[p] = constrain(v.as<int>(), 10, 300); p++; }
                } else if (doc["debounceMs"]) {
                    uint16_t old = doc["debounceMs"];
                    for (int p = 0; p < MAX_PADS_POSSIBLE; p++) padDebounceMs[p] = old;
                }
                if (doc["padPeakWindowMs"].is<JsonArray>()) {
                    JsonArray a = doc["padPeakWindowMs"]; int p = 0;
                    for (JsonVariant v : a) { if (p >= MAX_PADS_POSSIBLE) break; padPeakWindowMs[p] = constrain(v.as<int>(), 2, 30); p++; }
                } else if (doc["peakWindowMs"]) {
                    uint16_t old = doc["peakWindowMs"];
                    for (int p = 0; p < MAX_PADS_POSSIBLE; p++) padPeakWindowMs[p] = old;
                }
                if (doc["padCrosstalkLockMs"].is<JsonArray>()) {
                    JsonArray a = doc["padCrosstalkLockMs"]; int p = 0;
                    for (JsonVariant v : a) { if (p >= MAX_PADS_POSSIBLE) break; padCrosstalkLockMs[p] = constrain(v.as<int>(), 2, 150); p++; }
                } else if (doc["crosstalkLockMs"]) {
                    uint16_t old = doc["crosstalkLockMs"];
                    for (int p = 0; p < MAX_PADS_POSSIBLE; p++) padCrosstalkLockMs[p] = old;
                }
                if (doc["crosstalkMatrix"].is<JsonArray>()) {
                    JsonArray a = doc["crosstalkMatrix"]; int idx = 0;
                    for (JsonVariant v : a) {
                        if (idx >= MAX_PADS_POSSIBLE * MAX_PADS_POSSIBLE) break;
                        int i = idx / MAX_PADS_POSSIBLE, j = idx % MAX_PADS_POSSIBLE;
                        crosstalkMatrix[i][j] = (i == j) ? 0 : constrain(v.as<int>(), 0, 100);
                        idx++;
                    }
                }
                if (doc["masterVolume"]) globalMasterVolume = doc["masterVolume"];
                if (doc["systemMode"]) globalSystemMode = (SysMode)doc["systemMode"].as<int>();
                if (doc["padCount"]) globalPadCount = doc["padCount"];
                if (doc["fixedVelocity"]) globalFixedVelocity = doc["fixedVelocity"];
                if (doc["dynamicVelocity"].is<int>()) globalDynamicVelocity = (doc["dynamicVelocity"].as<int>() != 0);

                JsonArray kits = doc["kits"];
                for (JsonObject kitObj : kits) {
                    int k = kitObj["id"];
                    if (k >= 1 && k <= MAX_KITS) {
                        if (kitObj["name"].is<const char*>()) {
                            String nm = kitObj["name"].as<const char*>();
                            nm.trim();
                            if (nm.length() > KIT_NAME_MAXLEN) nm = nm.substring(0, KIT_NAME_MAXLEN);
                            strncpy(kitNames[k], nm.c_str(), KIT_NAME_MAXLEN);
                            kitNames[k][KIT_NAME_MAXLEN] = '\0';
                        }
                        JsonArray pads = kitObj["pads"];
                        int p = 0;
                        for (JsonObject padObj : pads) {
                            if (p < MAX_PADS_POSSIBLE) {
                                kitAssignments[k][p].layerA_ToneNum = padObj["la"];
                                kitAssignments[k][p].layerB_ToneNum = padObj["lb"];
                                kitAssignments[k][p].padVolumePercentA = padObj["va"];
                                kitAssignments[k][p].padVolumePercentB = padObj["vb"];
                                kitAssignments[k][p].padFilterA = padObj["fa"];
                                kitAssignments[k][p].padFilterB = padObj["fb"];
                                kitAssignments[k][p].padPitchA = padObj["pta"].is<int>() ? padObj["pta"].as<int>() : 0;
                                kitAssignments[k][p].padPitchB = padObj["ptb"].is<int>() ? padObj["ptb"].as<int>() : 0;
                                kitAssignments[k][p].padPanA = padObj["pna"].is<int>() ? padObj["pna"].as<int>() : 0;
                                kitAssignments[k][p].padPanB = padObj["pnb"].is<int>() ? padObj["pnb"].as<int>() : 0;
                                kitAssignments[k][p].roundRobinEnabled = padObj["rre"].is<int>() ? padObj["rre"].as<int>() : 0;
                                kitAssignments[k][p].rrToneNum[0] = padObj["rr1"].is<int>() ? padObj["rr1"].as<int>() : 0;
                                kitAssignments[k][p].rrToneNum[1] = padObj["rr2"].is<int>() ? padObj["rr2"].as<int>() : 0;
                                kitAssignments[k][p].rrToneNum[2] = padObj["rr3"].is<int>() ? padObj["rr3"].as<int>() : 0;
                            }
                            p++;
                        }
                    }
                }
                xSemaphoreGive(settingsMutex);
                saveKitMappingsToSD();
                saveKitNamesToSD();
            }
        }
    });

    server.on("/list-files", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray files = doc["files"].to<JsonArray>();
        constexpr int MAX_FILES_LISTED = 500;
        constexpr size_t MIN_FREE_HEAP_BYTES = 20000;
        bool truncated = false;
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            if (SD_MMC.exists("/SAMPLES")) {
                File dir = SD_MMC.open("/SAMPLES");
                File entry = dir.openNextFile();
                int count = 0;
                while (entry) {
                    if (count >= MAX_FILES_LISTED || ESP.getFreeHeap() < MIN_FREE_HEAP_BYTES) {
                        truncated = true;
                        entry.close();
                        break;
                    }
                    if (!entry.isDirectory()) {
                        JsonObject f = files.add<JsonObject>();
                        String nm = String(entry.name());
                        int slash = nm.lastIndexOf('/');
                        if (slash >= 0) nm = nm.substring(slash + 1);
                        f["name"] = nm;
                        f["size"] = entry.size();
                    }
                    entry.close();
                    entry = dir.openNextFile();
                }
                dir.close();
            }
            xSemaphoreGive(sdMutex);
        }
        doc["truncated"] = truncated;
        doc["shown"] = (int)files.size();
        String jsonResponse;
        serializeJson(doc, jsonResponse);
        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", jsonResponse);
        addCORSHeaders(response); request->send(response);
    });

    server.on("/delete-file", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("name")) { AsyncWebServerResponse *response = request->beginResponse(400, "text/plain", "Missing name"); addCORSHeaders(response); request->send(response); return; }
        String nm = request->getParam("name")->value();
        nm.replace("/", ""); nm.replace("\\", ""); nm.replace("..", "");
        String path = "/SAMPLES/" + nm;
        bool existed = false, removed = false;
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            existed = SD_MMC.exists(path);
            if (existed) removed = SD_MMC.remove(path);
            xSemaphoreGive(sdMutex);
        }
        AsyncWebServerResponse *response;
        if (existed) {
            response = request->beginResponse(200, "text/plain", removed ? "Deleted" : "Delete Failed");
        } else {
            response = request->beginResponse(404, "text/plain", "Not Found");
        }
        addCORSHeaders(response); request->send(response);
    });

    server.on("/get-wav", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("tone")) { AsyncWebServerResponse *response = request->beginResponse(400, "text/plain", "Missing tone"); addCORSHeaders(response); request->send(response); return; }
        uint16_t toneNum = request->getParam("tone")->value().toInt();
        char path[30]; snprintf(path, sizeof(path), "/SAMPLES/%04d.WAV", toneNum);
        if (!SD_MMC.exists(path)) { AsyncWebServerResponse *response = request->beginResponse(404, "text/plain", "Not Found"); addCORSHeaders(response); request->send(response); return; }
        AsyncWebServerResponse *response = request->beginResponse(SD_MMC, path, "audio/wav");
        addCORSHeaders(response); request->send(response);
    });

    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Upload Complete");
        addCORSHeaders(response); request->send(response);
    }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
        static File uploadFile;
        if (index == 0) {
            if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                if (!SD_MMC.exists("/SAMPLES")) SD_MMC.mkdir("/SAMPLES");
                String path = filename;
                if (!path.startsWith("/SAMPLES/") && !path.startsWith("SAMPLES/")) {
                    if (path.startsWith("/")) path = "/SAMPLES" + path;
                    else path = "/SAMPLES/" + path;
                }
                uploadFile = SD_MMC.open(path, FILE_WRITE);
                xSemaphoreGive(sdMutex);
            }
        }
        if (uploadFile) {
            if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                uploadFile.write(data, len);
                xSemaphoreGive(sdMutex);
            }
        }
        if (final) {
            if (uploadFile) {
                if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                    uploadFile.close();
                    xSemaphoreGive(sdMutex);
                }
            }
        }
    });

    server.begin();
}

void stopWirelessHost() {
    webServerActive = false;
    server.end();
    WiFi.softAPdisconnect(true);
}

void piezoTask(void* param) {
    piezo.begin();
    for (;;) {
        piezo.scan();
        uint32_t now = millis();
        xSemaphoreTake(settingsMutex, portMAX_DELAY); 
        SysMode currentMode = globalSystemMode; 
        uint8_t activePads = globalPadCount; 
        uint8_t currentVel = globalFixedVelocity; 
        bool dynamicVelOn = globalDynamicVelocity;
        xSemaphoreGive(settingsMutex);
        
        if (!webServerActive) {
            for (int i = 0; i < activePads; i++) {
                if (globalMidiNoteOffTime[i] > 0 && (now - globalMidiNoteOffTime[i] >= 15)) {
                    MIDI.noteOff(PAD_MIDI_NOTES[i], 0, MIDI_CHANNEL);
                    globalMidiNoteOffTime[i] = 0;
                }

                if (piezo.wasHit(i)) {
                    uint16_t peakValue = piezo.getPeak(i);
                    if (currentMode == MODE_SAMPLER || currentMode == MODE_HYBRID) audioEngine.triggerPad(i);
                    if (currentMode == MODE_MIDI_OUT || currentMode == MODE_HYBRID) {
                        uint8_t hitVel = dynamicVelOn ? calculateDynamicVelocity(peakValue, padThreshold[i]) : currentVel;
                        MIDI.noteOn(PAD_MIDI_NOTES[i], hitVel, MIDI_CHANNEL); 
                        globalMidiNoteOffTime[i] = now; 
                    } 
                    xSemaphoreTake(settingsMutex, portMAX_DELAY); 
                    globalPadLastVelocity[i] = peakValue; 
                    globalPadHitTime[i] = now; 
                    xSemaphoreGive(settingsMutex);
                    ui.setPadActive(i);

                    broadcastWsEvent("PAD_HIT", "\"pad\":" + String(i) + ",\"peak\":" + String(peakValue));
                }
            }
        } else {
            for (int i = 0; i < activePads; i++) {
                if (piezo.wasHit(i)) {
                    uint16_t peakValue = piezo.getPeak(i);
                    broadcastWsEvent("PAD_HIT", "\"pad\":" + String(i) + ",\"peak\":" + String(peakValue));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void uiTask(void* param) {
    for (;;) {
        int change = ui.update();
        if (ws.count() > 0) {
            ws.cleanupClients();
        }
        if (change != 0) {
            int liveKit = ui.getKit();
            if (change == 99) { ui.showLoading(liveKit); audioEngine.loadKit(liveKit); }
            else if (ui.getCurrentScreen() == SCREEN_MAIN) {
                int newKit;
                bool chainOn;
                xSemaphoreTake(settingsMutex, portMAX_DELAY); chainOn = globalChainModeActive; xSemaphoreGive(settingsMutex);
                if (chainOn && (change == 1 || change == -1)) {
                    int stepped = stepChain((int8_t)change);
                    newKit = (stepped > 0) ? stepped : liveKit;
                } else {
                    newKit = constrain(liveKit + change, MIN_KIT, MAX_KITS);
                }
                if (newKit != liveKit) { ui.showLoading(newKit); audioEngine.loadKit(newKit); ui.setKit(newKit); }
            }
        }
        static uint32_t lastRefresh = 0;
        if (millis() - lastRefresh > 30) { ui.refreshRender(); ui.clearPadActive(); lastRefresh = millis(); }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200); MIDI.begin(); USB.begin();
    settingsMutex = xSemaphoreCreateMutex(); if (!settingsMutex) while(1);
    sdMutex = xSemaphoreCreateMutex(); if (!sdMutex) while(1);
    if (!psramFound()) while (1) delay(500);

    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN); ui.begin(); ui.showSplash();
    SD_MMC.setPins(SD_MMC_CLK_PIN, SD_MMC_CMD_PIN, SD_MMC_DATA_PIN);
    if (!SD_MMC.begin("/sdcard", true)) { ui.showError("SD Card Error!"); while (1) delay(500); }

    loadKitMappingsFromSD();
    loadKitNamesFromSD();
    loadChainsFromSD();
    if (!audioEngine.begin()) { ui.showError("Audio Error!"); while (1) delay(500); }

    ui.showLoading(1); audioEngine.loadKit(1); ui.setKit(1); ui.showMain();
    xTaskCreatePinnedToCore(piezoTask, "PiezoTask", 4096, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(uiTask, "UITask", 8192, nullptr, 2, nullptr, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
