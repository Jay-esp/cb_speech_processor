// speech processor
// (C) Jef Collin
// 2025


// reminder: when the lvgl library is updated, edit the config file!!!

// update firmware version for automatic check and download
// make sure WP jumer is ON
// note that it takes a long time to download the new firmware
// there is no soft or hard reset function so power cycle after software update


// todo

// mutex for all tft. calls ?



// libraries
#include "FS.h"
#include <SD.h>
#include <SPI.h>
#include <Preferences.h>
#include <lvgl.h>
#include <Wire.h>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include <SigmaDSP.h>
#include <driver/i2s.h>
#include <vector>
#include <algorithm>
#include <nvs_flash.h>
#include <nvs.h>
#include <set>
#include "arduinoFFT.h"
#include <LovyanGFX.hpp>

// Include generated parameter file
#include "SigmaDSP_parameters.h"


// change when the dsp code changes, this will trigger an update of the eeprom
int8_t dsp_firmware_version = 20;

// note config shows 0X68 and 0XA0 while I2C scanner shows 0X34 and 0X50, this is explained as 7 bit and 8 bit values

// The first parameter is the Wire object we'll be using when communicating wth the DSP
// The second parameter is the DSP i2c address, which is defined in the parameter file
// The third parameter is the sample rate
// An optional fourth parameter is the pin to physically reset the DSP
SigmaDSP dsp(Wire, DSP_I2C_ADDRESS, 48000.00f);

// Write Protect pin for dsp eeprom
// if low on power on the dsp fails
#define EEPROM_WP_PIN 14

// Only needed if an external i2c EEPROM is present + the DSP is in selfboot mode
// The first parameter is the Wire object we'll be using when communicating wth the EEPROM
// The second parameter is the EEPROM i2c address, which is defined in the parameter file
// The third parameter is the EEPROM size in kilobits (kb)
// An optional fourth parameter is the pin to toggle while writing content to EEPROM (for led not wp pin)
DSPEEPROM ee(Wire, EEPROM_I2C_ADDRESS, 256, 0);

SemaphoreHandle_t i2c_mutex; // handle for the mutex i2c communication with dsp
SemaphoreHandle_t i2s_mutex; // handle for the mutex data stream to/from dsp
SemaphoreHandle_t spi_mutex_lcd_sd; // handle for the mutex lcd and sd

// update after change in sequence or new tab

#define TAB_MAIN_REF              0
#define TAB_MENU_REF              1
#define TAB_COMPRESSOR_REF        2
#define TAB_EQUALIZER_REF         3
#define TAB_PRESETS_REF           4
#define TAB_EFFECTS_REF           5
#define TAB_ENDOFTRANSMISSION_REF 6
#define TAB_AUTOKEY_REF           7
#define TAB_VOICE_REF             8
#define TAB_SCOPE_REF             9
#define TAB_RECORDER_REF         10
#define TAB_MORSE_REF            11
#define TAB_VOX_REF              12
#define TAB_TEST_REF             13
#define TAB_SETTINGS_REF         14
#define TAB_EDIT_REF             15
#define TAB_SPECTRUM_REF         16
#define TAB_FILETRANSFER_REF     17
#define TAB_DELETE_REF           18
#define TAB_PRESETSMAIN_REF      19
#define TAB_SETEOTLEVEL_REF      20
#define TAB_SETAMPLIFIER_REF     21
#define TAB_SETVURANGE_REF       22
#define TAB_SETMORSELEVEL_REF    23
#define TAB_SETAUTOKEYLEVEL_REF  24




// I2S configuration
#define I2S_NUM               I2S_NUM_0
#define SAMPLE_RATE           48000
#define I2S_BUFFER_SIZE       1024
#define CIRCULAR_BUFFER_SIZE  12288

#define PIN_BCK               38
#define PIN_WS                39
#define PIN_DATA_IN           47
#define PIN_DATA_OUT          21

// SD card chip select
#define SD_CS_PIN 41

// relay driver
#define TX_RELAY 1

// transmit button
#define PTT_IN 18

// number of eot roger beeps for dynamic creation
#define EOT_BEEP_COUNT       67

// lvgl
#define LV_USE_DEBUG = 0;

// 3.5" display
#define TFT_HOR_RES 480
#define TFT_VER_RES 320
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 5 * (LV_COLOR_DEPTH / 8))

// use alps or other decoder
#define Use_Alps_Encoder false

// rotary encode io pins
// swap S1 and S2 pins depending on type of encoder
#define Encoder_1_Pin1 6
#define Encoder_1_Pin2 7
#define Encoder_2_Pin1 16
#define Encoder_2_Pin2 17
#define Encoder_1_Key 5
#define Encoder_2_Key 15

// rotary encoder
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

#if (Use_Alps_Encoder)
// Alps EC11 encoder requires half step tables, others need full step
#define R_START 0x0
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5

const unsigned char ttable[6][4] = {
  // R_START (00)
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};

#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6
#define R_START 0x0

const unsigned char ttable[7][4] = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

unsigned char Encoder_1_State = R_START;
unsigned char Encoder_2_State = R_START;

// track rotary encoder changes
int EncoderCounter1 = 0;
int EncoderCounter2 = 0;

long unsigned timer_encoderbutton1;
long unsigned timer_encoderbutton2;

boolean Encoder_Key1_Long_Press = false;
boolean Encoder_Key2_Long_Press = false;

char printbuf[100];

// due to the lovyangxf lib the IDE enforces more rules hence we need the structure definitions in front of the gxf class

// compressor definition
typedef struct speech_compressor_t {
  float threshold = -60.0f;  // dB threshold
  float ratio     = 4.0f;    // compression ratio
  float rms_tc    = 120.0f;
  float hold      = 10.0f;
  float decay     = 10.0f;
  float postgain  = 0.0f;
  float knee      = 10.0f;    // dB width of the knee
};

// create compressor instance
speech_compressor_t speech_compressor_parameters;

// noise gate definition
typedef struct speech_noisegate_t {
  float gateThreshold = -60.0f; // dB threshold
  float gateKnee     = 4.0f;    // compression ratio
  float gateFloor    = 1.0f;
};

// create compressor instance
speech_noisegate_t speech_noisegate_parameters;

// graphics lib configuration
class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ILI9488     _panel_instance;
    lgfx::Touch_XPT2046     _touch_instance;
    lgfx::Bus_SPI           _bus_instance;
  public:
    LGFX(void)
    {
      {
        auto cfg = _bus_instance.config();
        // SPI
        cfg.spi_host = SPI2_HOST;
        cfg.spi_mode = 0;
        cfg.freq_write = 60000000;
        cfg.freq_read  = 16000000;
        cfg.spi_3wire  = true;
        cfg.use_lock   = true;
        cfg.dma_channel = SPI_DMA_CH_AUTO;
        cfg.pin_sclk = 12;
        cfg.pin_mosi = 11;
        cfg.pin_miso = 13;
        cfg.pin_dc   = 2;
        _bus_instance.config(cfg);
        _panel_instance.setBus(&_bus_instance);
      }
      {
        auto cfg = _panel_instance.config();
        cfg.pin_cs           =    10;
        cfg.pin_rst          =    -1;
        cfg.pin_busy         =    -1;
        cfg.panel_width      =   320;
        cfg.panel_height     =   480;
        cfg.offset_x         =     0;
        cfg.offset_y         =     0;
        cfg.offset_rotation  =     0;
        cfg.dummy_read_pixel =     8;
        cfg.dummy_read_bits  =     1;
        cfg.readable         =  true;
        cfg.invert           = false;
        cfg.rgb_order        = false;
        cfg.dlen_16bit       = false;
        cfg.bus_shared       =  true;
        _panel_instance.config(cfg);
      }
      {
        // must be raw data point but not used if calibration is used
        auto cfg = _touch_instance.config();
        cfg.x_min      = 0;
        cfg.x_max      = 4000;
        cfg.y_min      = 0;
        cfg.y_max      = 4000;
        cfg.pin_int    = -1;
        cfg.bus_shared = true;
        cfg.offset_rotation = 0;
        // SPI
        cfg.spi_host = SPI2_HOST;
        cfg.freq = 1000000;
        cfg.pin_sclk = 12;
        cfg.pin_mosi = 11;
        cfg.pin_miso = 13;
        cfg.pin_cs   =  42;
        _touch_instance.config(cfg);
        _panel_instance.setTouch(&_touch_instance);
      }
      setPanel(&_panel_instance);
    }
};

LGFX TFT;

// callback routine for display
void my_disp_flush(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  lv_draw_sw_rgb565_swap(px_map, w * h);
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  TFT.startWrite();
  TFT.pushImageDMA(area->x1, area->y1, w, h, (uint16_t*)px_map);
  TFT.endWrite();
  xSemaphoreGive(spi_mutex_lcd_sd);
  lv_disp_flush_ready(disp);
}

// two buffer system
void *draw_buf_1;
void *draw_buf_2;

unsigned long lastTickMillis = 0;

static lv_display_t *disp;

// current tab
uint32_t tab_c = 0;

Preferences preferences;

// touch screen calib data
uint16_t speech_setting_calibration_data[8];

// timers
unsigned long LVGL_Timer = 0;
unsigned long key1_timer = 0;
unsigned long key2_timer = 0;
unsigned long prior_tick_Millis = millis();
unsigned long speech_vu_update_timer = 0;
unsigned long ptt_stuck_timer = 0;

// state machine
uint8_t speech_eot_state = 0;
boolean speech_bypass = false;
boolean speech_transmitting = false;
boolean speech_ptt_active = false;
boolean speech_ptt_active_last = false;
uint16_t speech_ptt_debounce = 100;
volatile bool speech_ptt_event_flag = false;

// ptt mode
// 0 normal: ptt on -> tx on, ptt off -> tx off
// 1 delayed mode: ptt on -> tx on, ptt off -> wait until beep or message finished -> tx off
// 2 non tx mode: ptt on or off -> tx off
// 3 esp controlled mode
int8_t speech_ptt_mode = 0;
volatile unsigned long speech_ptt_debounce_timer = 0;
unsigned long speech_ptt_record_debounce_timer;

boolean speech_screen_update_done = false;

// 0 = nothing
// 1 = turn off
// 2 = turn on
uint8_t speech_relay_action = 0;

boolean speech_input_muted = false;

int8_t speech_vu_meter = 0;
float speech_vu_readback = 0;

int32_t speech_vu_scaling = 40;

int32_t speech_amplifier = 10;

int32_t speech_gain = 10;
int32_t speech_output = 10;
int32_t speech_headphone = 0;

boolean speech_rotary_selected_output = true;

//end of transmission mode on
boolean speech_eot_active = false;

int32_t speech_compressor_threshold = 10;
int32_t speech_compressor_ratio = 10;
int32_t speech_compressor_postgain = 0;
int32_t speech_compressor_noisegate = 10;

boolean speech_rotary_selected_threshold = true;
boolean speech_rotary_selected_postgain = true;

// Create an instance for each EQ block
secondOrderEQ eqBand0;
secondOrderEQ eqBand1;
secondOrderEQ eqBand2;
secondOrderEQ eqBand3;
secondOrderEQ eqBand4;
secondOrderEQ eqBand5;
secondOrderEQ eqBand6;
secondOrderEQ eqBand7;
secondOrderEQ eqBand8;
secondOrderEQ eqBand9;
secondOrderEQ eqBand10;
secondOrderEQ eqBand11;

uint8_t speech_selected_eq = 0;

int32_t speech_eq0 = 0;
int32_t speech_eq1 = 0;
int32_t speech_eq2 = 0;
int32_t speech_eq3 = 0;
int32_t speech_eq4 = 0;
int32_t speech_eq5 = 0;
int32_t speech_eq6 = 0;
int32_t speech_eq7 = 0;
int32_t speech_eq8 = 0;
int32_t speech_eq9 = 0;
int32_t speech_eq10 = 0;
int32_t speech_eq11 = 0;

String speech_presets_msg[] = {
  "Preset 1",
  "Preset 2",
  "Preset 3",
  "Preset 4",
  "Preset 5",
  "Preset 6"
};

uint8_t speech_presets_sel = 1;

int32_t speech_effect_echo_time = 5;
int32_t speech_effect_echo_fb = 15;
int32_t speech_effect_pitch = 0;
int32_t speech_effect_ring = 30;

boolean speech_effect_echo_sb_enabled = false;
boolean speech_effect_echo_cleared = true;

boolean speech_effect_echo_on = false;
boolean speech_effect_sb_on = false;

boolean speech_effect_pitch_on = false;
boolean speech_effect_ring_on = false;

uint8_t speech_effect_selected_slider = 0;

// selected roger beep
uint16_t speech_eot_beep_selected = 1;

// selected sound clip beep
uint16_t speech_eot_sound_selected = 1;

// select beep or sound
uint8_t speech_eot_selection = 0;

// count sound clip files
uint16_t speech_eot_sound_count = 0;

// block other processes if sound eot is activated
boolean speech_eot_block = false;

// lock activity during eot or...
boolean speech_lock = false;

long unsigned speech_scope_timer;
int8_t* speech_scope_previous;
float speech_scope_scale = 8000;
boolean speech_scope_autoscale = false;

// spectrum sampling and FFT
#define FFT_SIZE 2048
#define MAX_FREQ 4000
#define BIN_WIDTH (SAMPLE_RATE / FFT_SIZE)
#define NUM_BINS_TO_PROCESS (MAX_FREQ / BIN_WIDTH)
#define NUM_BANDS 119
#define BAND_WIDTH (NUM_BINS_TO_PROCESS / NUM_BANDS)

int speech_spectrum_band_values[NUM_BANDS] = {0};

static double speech_spectrum_vReal[FFT_SIZE];
static double speech_spectrum_vImag[FFT_SIZE];

ArduinoFFT<double> FFT = ArduinoFFT<double>(speech_spectrum_vReal, speech_spectrum_vImag, FFT_SIZE, SAMPLE_RATE);

int32_t speech_spectrum_buffer[2048];

uint8_t speech_spectrum_cursor = 0;
uint8_t speech_spectrum_cursor_last_pos = 255;
boolean speech_spectrum_peaktrigger = false;

int32_t speech_spectrum_low_freq = 100;
int32_t speech_spectrum_high_freq = 4000;

uint8_t speech_rec_pb_state = 0;
uint8_t speech_rec_pb_state_snap = 0;
uint16_t speech_rec_pb_progress = 0;
unsigned long speech_rec_pb_timer;
unsigned long speech_rec_pb_progress_timer;

uint8_t speech_selected_file = 0;

// flag for recording tasks
boolean speech_recording_playback_active = false;

boolean speech_task1_completed = false;
boolean speech_task2_completed = false;

char speech_filename[30];

const char* speech_voice_filenames[] = {
  "/recording_20s.bin",
  "/recording_1.bin",
  "/recording_2.bin",
  "/recording_3.bin",
  "/recording_4.bin",
  "/recording_5.bin"

};

String speech_voice_msg[] = {
  "Msg 1",
  "Msg 2",
  "Msg 3",
  "Msg 4",
  "Msg 5"
};

uint8_t speech_voice_msg_sel = 1;

uint16_t speech_voice_progress = 0;

unsigned long speech_voice_progress_timer;

int32_t speech_autokey_countdown_time = 0;
int32_t speech_autokey_countdown_interval = 0;
int32_t speech_autokey_countdown_interval_reload = 0;
int32_t speech_autokey_previous_countdown_time = 1;
int32_t speech_autokey_previous_countdown_interval = 1;
boolean speech_autokey_enabled = false;
boolean speech_autokey_trigger = false;
boolean speech_autokey_stopped = false;
// count sound clip files
uint16_t speech_autokey_sound_count = 0;

// Timer handle
esp_timer_handle_t speech_autokey_timer;

String speech_morse_msg[] = {
  "Morse 1",
  "Morse 2",
  "Morse 3",
  "Morse 4",
  "Morse 5"
};


int32_t speech_autokey_level = 20;


String speech_morse_play = "";
uint8_t speech_morse_state = 0;
uint8_t speech_morse_msg_sel = 1;

int32_t speech_morse_level = 20;

int32_t speech_vox_threshold_slider = 0;
int32_t speech_vox_hangtime_slider = 0;
// vox state machine
uint8_t speech_vox_state = 0;
long unsigned speech_vox_timer;
// time the loops to avoid frequent read activity
long unsigned speech_vox_looptimer;

int32_t speech_test_signal_gain = 0;
// signal mode
int8_t speech_test_mode = 0;
int32_t speech_test_frequency1 = 1000;
int32_t speech_test_frequency2 = 500;
uint8_t speech_test_enc2_mode = 0;
boolean speech_test_cursor_override = false;

uint8_t speech_filetransfer_state = 0;
boolean speech_filetransfer_statuschange = false;

// count files
uint16_t speech_filedelete_count = 0;
uint16_t speech_filedelete_selected = 0;

int32_t speech_eot_level = 20;

int32_t speech_eot_mp3_level = 20;

static int32_t i2s_buffer[1024];

int32_t *speech_circular_buffer = NULL;

size_t speech_bytes_read;
size_t speech_circular_buffer_index = 0;
size_t speech_playback_index = 5000;

static volatile size_t speech_delay_offset = 5000;

int32_t *rec_pb_buffer0 = NULL;
int32_t *rec_pb_buffer1 = NULL;

// flag which buffer is filled 0 = noting yet
volatile  uint8_t speech_buffer_done = 0;
volatile  boolean speech_buffer_next = false;

volatile boolean speech_record_complete = false;

uint32_t speech_file_blocks = 0;

size_t speech_bytes_written;

// Task handles
TaskHandle_t filetransferTaskHandle;

TaskHandle_t recordTaskHandle;
TaskHandle_t writeTaskHandle;
TaskHandle_t readTaskHandle;
TaskHandle_t playTaskHandle;

TaskHandle_t readEotSoundTaskHandle;
TaskHandle_t playEotSoundTaskHandle;

TaskHandle_t echo_sb_TaskHandle;

// for keyboard routine
uint8_t kb_caller = 0;
uint8_t kb_returntab = 0;
String kb_text = "";


// lvgl screen variables
static lv_obj_t *tabview;

static lv_obj_t *tab_main;
static lv_obj_t *tab_menu;
static lv_obj_t *tab_compressor;
static lv_obj_t *tab_equalizer;
static lv_obj_t *tab_presets;
static lv_obj_t *tab_effects;
static lv_obj_t *tab_endoftransmission;
static lv_obj_t *tab_autokey;
static lv_obj_t *tab_voice;
static lv_obj_t *tab_scope;
static lv_obj_t *tab_recorder;
static lv_obj_t *tab_morse;
static lv_obj_t *tab_vox;
static lv_obj_t *tab_test;
static lv_obj_t *tab_settings;
static lv_obj_t *tab_edit;
static lv_obj_t *tab_spectrum;
static lv_obj_t *tab_filetransfer;
static lv_obj_t *tab_filedelete;
static lv_obj_t *tab_presetsmain;
static lv_obj_t *tab_seteotlevel;
static lv_obj_t *tab_setamplifier;
static lv_obj_t *tab_setvurange;
static lv_obj_t *tab_setmorselevel;
static lv_obj_t *tab_setautokeylevel;

static lv_obj_t *slider_main_gain;
static lv_obj_t *slider_main_output;
static lv_obj_t *slider_main_headphone;

static lv_obj_t *lbl_main_gain;
static lv_obj_t *lbl_main_output;
static lv_obj_t *lbl_main_headphone;

static lv_obj_t *cb_main_eot_on;

static lv_obj_t *led_main_tx;
static lv_obj_t *lbl_main_tx;

static lv_obj_t *btn_main_preset;
static lv_obj_t *btn_main_menu;

static lv_obj_t *slider_compressor_threshold;
static lv_obj_t *slider_compressor_ratio;
static lv_obj_t *slider_compressor_postgain;
static lv_obj_t *slider_compressor_noisegate;

static lv_obj_t *lbl_compressor_threshold;
static lv_obj_t *lbl_compressor_ratio;
static lv_obj_t *lbl_compressor_postgain;
static lv_obj_t *lbl_compressor_noisegate;

static lv_obj_t *btn_compressor_reset;
static lv_obj_t *btn_compressor_back;

static lv_obj_t *slider_eq0;
static lv_obj_t *slider_eq1;
static lv_obj_t *slider_eq2;
static lv_obj_t *slider_eq3;
static lv_obj_t *slider_eq4;
static lv_obj_t *slider_eq5;
static lv_obj_t *slider_eq6;
static lv_obj_t *slider_eq7;
static lv_obj_t *slider_eq8;
static lv_obj_t *slider_eq9;
static lv_obj_t *slider_eq10;
static lv_obj_t *slider_eq11;

static lv_obj_t *lbl_eq0;
static lv_obj_t *lbl_eq1;
static lv_obj_t *lbl_eq2;
static lv_obj_t *lbl_eq3;
static lv_obj_t *lbl_eq4;
static lv_obj_t *lbl_eq5;
static lv_obj_t *lbl_eq6;
static lv_obj_t *lbl_eq7;
static lv_obj_t *lbl_eq8;
static lv_obj_t *lbl_eq9;
static lv_obj_t *lbl_eq10;
static lv_obj_t *lbl_eq11;

static lv_obj_t *cb_presets_selection0;
static lv_obj_t *cb_presets_selection1;
static lv_obj_t *cb_presets_selection2;
static lv_obj_t *cb_presets_selection3;
static lv_obj_t *cb_presets_selection4;
static lv_obj_t *cb_presets_selection5;

static lv_obj_t *preset_checkboxes[6];

static lv_obj_t *btn_presets_set;
static lv_obj_t *btn_presets_recall;

static lv_obj_t *btn_presets_back;

static lv_obj_t *btn_presetsmain_preset0;
static lv_obj_t *btn_presetsmain_preset1;
static lv_obj_t *btn_presetsmain_preset2;
static lv_obj_t *btn_presetsmain_preset3;
static lv_obj_t *btn_presetsmain_preset4;
static lv_obj_t *btn_presetsmain_preset5;

static lv_obj_t *lbl_presetsmain_preset0;
static lv_obj_t *lbl_presetsmain_preset1;
static lv_obj_t *lbl_presetsmain_preset2;
static lv_obj_t *lbl_presetsmain_preset3;
static lv_obj_t *lbl_presetsmain_preset4;
static lv_obj_t *lbl_presetsmain_preset5;

static lv_obj_t *preset_mainbuttons[6];

static lv_obj_t *slider_effect_echo_time;
static lv_obj_t *slider_effect_echo_fb;

static lv_obj_t *slider_effect_pitch;

static lv_obj_t *slider_effect_ring;

static lv_obj_t *cb_effect_selection1;
static lv_obj_t *cb_effect_selection2;
static lv_obj_t *cb_effect_selection3;
static lv_obj_t *cb_effect_selection4;

static lv_obj_t *lbl_effect_slider1;
static lv_obj_t *lbl_effect_slider2;
static lv_obj_t *lbl_effect_slider3;
static lv_obj_t *lbl_effect_slider4;

static lv_obj_t *btn_effect_back;

static lv_obj_t *btn_eot_play;
static lv_obj_t *btn_eot_back;
static lv_obj_t *cb_eot_on;
static lv_obj_t *cb_eot_tx_on;

static lv_obj_t *cb_eot_sel_beep;
static lv_obj_t *cb_eot_sel_sound;

static lv_obj_t *rol_eot_roller_beep;

static lv_obj_t *rol_eot_roller_sound;

static lv_obj_t *btn_scope_back;

static lv_obj_t *lbl_spectrum_cursor;
static lv_obj_t *btn_spectrum_back;

static lv_obj_t *btn_recorder_play;
static lv_obj_t *btn_recorder_erase;
static lv_obj_t *btn_recorder_cancel;
static lv_obj_t *btn_recorder_back;
static lv_obj_t *bar_recorder_progress;
static lv_obj_t *cb_recorder_tx_on;
static lv_obj_t *cb_recorder_bypass_on;

static lv_obj_t *led_recorder_recording;
static lv_obj_t *lbl_recorder_recording;

static lv_obj_t *btn_voice_play;
static lv_obj_t *btn_voice_cancel;
static lv_obj_t *btn_voice_back;
static lv_obj_t *bar_voice_progress;
static lv_obj_t *cb_voice_tx_on;

static lv_obj_t *led_voice_recording;
static lv_obj_t *lbl_voice_recording;

static lv_obj_t *cb_voice_selection1;
static lv_obj_t *cb_voice_selection2;
static lv_obj_t *cb_voice_selection3;
static lv_obj_t *cb_voice_selection4;
static lv_obj_t *cb_voice_selection5;

static lv_obj_t *rol_autokey_roller_sound;

static lv_obj_t *led_autokey_tx;
static lv_obj_t *lbl_autokey_tx;

static lv_obj_t *spinbox_autokey_txtime;
static lv_obj_t *spinbox_autokey_interval;

static lv_obj_t *lbl_autokey_txtime;
static lv_obj_t *lbl_autokey_interval;

static lv_obj_t *btn_autokey_txtime_inc;
static lv_obj_t *btn_autokey_txtime_dec;
static lv_obj_t *btn_autokey_interval_inc;
static lv_obj_t *btn_autokey_interval_dec;

static lv_obj_t *btn_autokey_start_stop;
static lv_obj_t *lbl_autokey_start_stop;

static lv_obj_t *btn_autokey_set_level;
static lv_obj_t *btn_autokey_back;

static lv_obj_t *btn_morse_play;
static lv_obj_t *btn_morse_cancel;
static lv_obj_t *btn_morse_back;
static lv_obj_t *cb_morse_tx_on;
static lv_obj_t *btn_morse_set_level;

static lv_obj_t *cb_morse_selection1;
static lv_obj_t *cb_morse_selection2;
static lv_obj_t *cb_morse_selection3;
static lv_obj_t *cb_morse_selection4;
static lv_obj_t *cb_morse_selection5;

static lv_obj_t *slider_vox_threshold;
static lv_obj_t *slider_vox_hangtime;
static lv_obj_t *led_vox_tx;
static lv_obj_t *lbl_vox_tx;
static lv_obj_t *cb_vox_on;
static lv_obj_t *btn_vox_back;

static lv_obj_t *slider_test_signal_gain;

static lv_obj_t *cb_test_source1;
static lv_obj_t *cb_test_source2;
static lv_obj_t *cb_test_source3;

static lv_obj_t *spinbox_test_frequency_1;
static lv_obj_t *spinbox_test_frequency_2;

static lv_obj_t *btn_test_frequency_1_inc;
static lv_obj_t *btn_test_frequency_1_dec;
static lv_obj_t *btn_test_frequency_2_inc;
static lv_obj_t *btn_test_frequency_2_dec;

static lv_obj_t *btn_test_tx;

static lv_obj_t *btn_filetransfer_start;
static lv_obj_t *btn_filetransfer_cancel;
static lv_obj_t *btn_filetransfer_back;

lv_obj_t *lbl_filetransfer_info;

static lv_obj_t *cb_bypass_on;


static lv_obj_t *btn_eot_set_level;
static lv_obj_t *btn_settings_back;

static lv_obj_t *edit_txtentry;
static lv_obj_t *edit_keyboard;

static lv_obj_t *rol_filedelete_file;
static lv_obj_t *btn_filedelete_delete;
static lv_obj_t *btn_filedelete_back;

static lv_obj_t *slider_set_eot_level;
static lv_obj_t *slider_set_eot_mp3_level;
static lv_obj_t *btn_seteotlevel_back;

static lv_obj_t *slider_set_amplifier;
static lv_obj_t *btn_setamplifier_back;

static lv_obj_t *slider_set_vu_range;
static lv_obj_t *btn_setvu_back;

static lv_obj_t *slider_set_morse_level;
static lv_obj_t *btn_set_morse_level_back;

static lv_obj_t *slider_set_autokey_level;
static lv_obj_t *btn_set_autokey_level_back;



// touch screen callback
void my_touchpad_read(lv_indev_t * indev, lv_indev_data_t * data) {
  uint16_t touchX, touchY;
  data->state = LV_INDEV_STATE_REL;
  // Attempt to take mutex with timeout (e.g., 10ms)
  if (xSemaphoreTake(spi_mutex_lcd_sd, pdMS_TO_TICKS(10)) == pdTRUE) {
    bool touched = TFT.getTouch(&touchX, &touchY);
    xSemaphoreGive(spi_mutex_lcd_sd); // Release mutex immediately after hardware access
    if (touched) {
      data->state = LV_INDEV_STATE_PR;
      // Validate coordinates
      if (touchX <= 479 && touchY <= 319) {
        data->point.x = touchX;
        data->point.y = touchY;
      }
    }
  }
}

// encoder interrupts
void IRAM_ATTR isr1() {
  unsigned char pinstate = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][pinstate];
  unsigned char result = Encoder_1_State & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter1 < 10) {
      EncoderCounter1++;
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter1 > -10) {
      EncoderCounter1--;
    }
  }
}

void IRAM_ATTR isr2() {
  unsigned char pinstate = (digitalRead(Encoder_2_Pin2) << 1) | digitalRead(Encoder_2_Pin1);
  Encoder_2_State = ttable[Encoder_2_State & 0xf][pinstate];
  unsigned char result = Encoder_2_State & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter2 < 10) {
      EncoderCounter2++;
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter2 > -10) {
      EncoderCounter2--;
    }
  }
}

void IRAM_ATTR ptt_interrupt() {
  unsigned long t_now = millis(); // not perfect in ISR, but usable just to detect edges quickly
  // Only register event if debounce period passed
  if (t_now - speech_ptt_debounce_timer > speech_ptt_debounce) {
    speech_ptt_debounce_timer = t_now;
    if (speech_morse_state == 0) {
      speech_ptt_active = !digitalRead(PTT_IN);
      if (speech_ptt_active != speech_ptt_active_last) {
        speech_ptt_active_last = speech_ptt_active;
        speech_ptt_event_flag = true; // signal event for main loop
      }
    }
    else {
      speech_ptt_active = false;
    }
  }
}

// use Arduinos millis() as tick source
static uint32_t my_tick(void)
{
  return millis();
}


// echo / sb I2S task
void echo_sb_task(void *parameter) {
  boolean was_transmitting = false;
  while (true) {
    if (speech_effect_echo_sb_enabled) {
      // echo feature is enabled globally
      if (speech_transmitting && !speech_eot_block) {
        // we are actively transmitting - process echo
        was_transmitting = true;
        speech_effect_echo_cleared = false;
        // read I2S data
        xSemaphoreTake(i2s_mutex, portMAX_DELAY);
        i2s_read(I2S_NUM, (void*)i2s_buffer, sizeof(i2s_buffer), &speech_bytes_read, portMAX_DELAY);
        xSemaphoreGive(i2s_mutex);
        // number of samples read
        size_t num_samples = speech_bytes_read / sizeof(int32_t);
        // store data in the circular buffer
        for (size_t i = 0; i < num_samples; i++) {
          speech_circular_buffer[speech_circular_buffer_index] = i2s_buffer[i];
          speech_circular_buffer_index = (speech_circular_buffer_index + 1) % CIRCULAR_BUFFER_SIZE; // Wrap around
        }
        speech_playback_index = (speech_circular_buffer_index + speech_delay_offset) % CIRCULAR_BUFFER_SIZE;
        // write data from the circular buffer to I2S
        for (size_t i = 0; i < num_samples; i++) {
          int32_t sample = speech_circular_buffer[speech_playback_index];
          speech_playback_index = (speech_playback_index + 1) % CIRCULAR_BUFFER_SIZE; // Wrap around
          // send sample to I2S
          size_t speech_bytes_written;
          xSemaphoreTake(i2s_mutex, portMAX_DELAY);
          i2s_write(I2S_NUM, &sample, sizeof(sample), &speech_bytes_written, portMAX_DELAY);
          xSemaphoreGive(i2s_mutex);
        }

      }
      else {
        // echo enabled but not currently transmitting
        if (was_transmitting) {
          // we just stopped transmitting - clear buffers
          for (size_t i = 0; i < CIRCULAR_BUFFER_SIZE; i++) {
            speech_circular_buffer[i] = 0;
          }
          speech_effect_echo_cleared = true;
          was_transmitting = false;
        }
        // wait for transmission to start or echo to be disabled
        vTaskDelay(10); // Small delay to prevent busy-waiting
      }
    }
    else {
      // echo feature is disabled globally
      if (!speech_effect_echo_cleared) {
        // clear buffers if they haven't been cleared yet
        for (size_t i = 0; i < CIRCULAR_BUFFER_SIZE; i++) {
          speech_circular_buffer[i] = 0;
        }
        speech_effect_echo_cleared = true;
        was_transmitting = false;
      }
      // wait longer when completely disabled to save CPU
      vTaskDelay(100);
    }
  }
}

// timer ISR
void IRAM_ATTR speech_autokey_timer_callback(void* arg)
{
  if (speech_autokey_enabled) {
    if (speech_autokey_countdown_time > 0) {
      speech_autokey_countdown_time--;
      if (speech_autokey_countdown_time == 0) {
        // flag end
        speech_autokey_stopped = true;
        // reset at the end since there is no more check
        speech_autokey_countdown_interval = 0;
      }
      else {
        if (speech_autokey_countdown_interval > 0) {
          speech_autokey_countdown_interval--;
          if (speech_autokey_countdown_interval == 0) {
            // flag start of tx
            speech_autokey_trigger = true;
            // reload interval timer
            speech_autokey_countdown_interval = speech_autokey_countdown_interval_reload;
          }
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  // relay driver
  pinMode(TX_RELAY, OUTPUT);
  digitalWrite(TX_RELAY, LOW);

  // transmit button
  pinMode(PTT_IN, INPUT);

  // write protect pin eeprom dsp
  pinMode(EEPROM_WP_PIN, OUTPUT);
  digitalWrite(EEPROM_WP_PIN, HIGH);

  pinMode(Encoder_1_Pin1, INPUT);
  pinMode(Encoder_1_Pin2, INPUT);
  pinMode(Encoder_2_Pin1, INPUT);
  pinMode(Encoder_2_Pin2, INPUT);
  pinMode(Encoder_1_Key, INPUT);
  pinMode(Encoder_2_Key, INPUT);

  // allocate big buffers
  speech_circular_buffer = (int32_t *)calloc(CIRCULAR_BUFFER_SIZE, sizeof(int32_t));
  rec_pb_buffer0  = (int32_t *)calloc(6144, sizeof(int32_t));
  rec_pb_buffer1  = (int32_t *)calloc(6144, sizeof(int32_t));

  speech_scope_previous = (int8_t*)ps_malloc(512);

  Wire.begin();  // Initialize I2C
  dsp.begin();
  ee.begin();

  //    Serial.println(F("Pinging i2c lines...\n0 -> device is present\n2 -> device is not present"));
  //    Serial.print(F("DSP response: "));
  //    Serial.println(dsp.ping());
  //    Serial.print(F("EEPROM ping: "));
  //    Serial.println(ee.ping());

  // get current state to start otherwise encoder might not react to first click
  unsigned char temppinstate1 = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][temppinstate1];

  unsigned char temppinstate2 = (digitalRead(Encoder_2_Pin2) << 1) | digitalRead(Encoder_2_Pin1);
  Encoder_2_State = ttable[Encoder_2_State & 0xf][temppinstate2];

  TFT.init();
  TFT.setRotation(3);
  TFT.fillScreen(TFT_BLACK);

  lv_init();

  // set a tick source so that LVGL will know how much time elapsed
  lv_tick_set_cb(my_tick);

  // setup the two buffers
  draw_buf_1 = heap_caps_malloc(DRAW_BUF_SIZE, MALLOC_CAP_SPIRAM);
  draw_buf_2 = heap_caps_malloc(DRAW_BUF_SIZE, MALLOC_CAP_SPIRAM);

  disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);

  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, draw_buf_1, draw_buf_2, DRAW_BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);

  // use a mutex for all dsp related communication since we have multiple tasks competing for I2C access
  i2c_mutex = xSemaphoreCreateMutex();
  // mutex for i2s
  i2s_mutex = xSemaphoreCreateMutex();

  // mutex for i2s display and sd
  spi_mutex_lcd_sd = xSemaphoreCreateMutex();

  // sometimes the mount fails, retry loop
  boolean checkmount = false;
  for (int i = 0; i < 10; i++) {
    if (SD.begin(SD_CS_PIN, SPI, 25000000)) {
      // Serial.println("SD Card Mounted Successfully");
      checkmount = true;
      break;
    }
    Serial.println("SD Mount Failed, retrying...");
    delay(500);
  }

  clear_settings();

  // check if EEPROM already contains the current firmware version
  uint8_t dsp_check_version = ee.getFirmwareVersion();
  if (dsp_check_version != dsp_firmware_version) {
    TFT.fillScreen(TFT_WHITE);
    TFT.setTextColor(TFT_RED, TFT_WHITE);
    TFT.setTextDatum(MC_DATUM);
    TFT.setTextSize(3);
    TFT.drawString("DSP firmware updating...", TFT.width() / 2, TFT.height() / 2);

    // disable write protect
    digitalWrite(EEPROM_WP_PIN, LOW);

    // the last parameter in writeFirmware is the FW version, and prevents the MCU from overwriting on every reboot

    ee.writeFirmware(DSP_eeprom_firmware, sizeof(DSP_eeprom_firmware), dsp_firmware_version);
    dsp.reset();
    delay(2000); // wait for the FW to load from the EEPROM
    digitalWrite(EEPROM_WP_PIN, HIGH);

    TFT.fillScreen(TFT_WHITE);
    TFT.setTextColor(TFT_GREEN, TFT_WHITE);
    TFT.drawString("Completed... cycle power.", TFT.width() / 2, TFT.height() / 2);

    while (1) {}
  }

  attachInterrupt(Encoder_1_Pin1, isr1, CHANGE);
  attachInterrupt(Encoder_1_Pin2, isr1, CHANGE);
  attachInterrupt(Encoder_2_Pin1, isr2, CHANGE);
  attachInterrupt(Encoder_2_Pin2, isr2, CHANGE);

  boolean forcecalibrate = false;
  // check if one of the buttons is pressed during boot
  if ((digitalRead(Encoder_1_Key) + digitalRead(Encoder_2_Key)) == 1) {
    // make sure its not a power on glitch due to capacitor charging on the encoder module
    delay(500);
    if ((digitalRead(Encoder_1_Key) + digitalRead(Encoder_2_Key)) == 1) {
      // call the calibration screen if one and only one of the encoder switches is pressed during power on
      forcecalibrate = true;
    }
  }
  if (!load_settings() or forcecalibrate) {
    calibratescreen();
  } else {
    TFT.setTouchCalibrate(speech_setting_calibration_data);
  }

  Display_Splash_Screen();

  delay(2000);

  Setup_Screens();

  // this has to be done after the setup since declaring the variables does not create a pointer to them

  preset_checkboxes[0] = cb_presets_selection0;
  preset_checkboxes[1] = cb_presets_selection1;
  preset_checkboxes[2] = cb_presets_selection2;
  preset_checkboxes[3] = cb_presets_selection3;
  preset_checkboxes[4] = cb_presets_selection4;
  preset_checkboxes[5] = cb_presets_selection5;

  preset_mainbuttons[0] = lbl_presetsmain_preset0;
  preset_mainbuttons[1] = lbl_presetsmain_preset1;
  preset_mainbuttons[2] = lbl_presetsmain_preset2;
  preset_mainbuttons[3] = lbl_presetsmain_preset3;
  preset_mainbuttons[4] = lbl_presetsmain_preset4;
  preset_mainbuttons[5] = lbl_presetsmain_preset5;

  // check sound recording files and create if missing
  for (uint8_t i = 0; i < 6; i++) {
    if (!SD.exists(speech_voice_filenames[i])) {
      clear_file(i);
    }
  }

  // setup default values before loading presets
  // all others are set in the define section

  //// 2nd order equalizer typedef
  //typedef struct secondOrderEQ_t
  //{
  //  float Q            = 1.41; // Parametric, Peaking, range 0-16
  //  float boost        = 0.0;  // Range +/-15 [dB]
  //  float freq;                // Range 20-20000 [Hz]
  //  uint8_t filterType = parameters::filterType::peaking; // parameters::filterType::[type]
  //  uint8_t phase      = parameters::phase::deg_0;        // parameters::phase::deg_0/deg_180
  //  uint8_t state      = parameters::state::on;           // parameters::state::on/off
  //} secondOrderEQ;

  // initialise EQ bands
  eqBand0.freq = 90;
  eqBand0.Q = 0.8;
  eqBand0.boost = 0;

  eqBand1.freq = 150;
  eqBand1.Q = 1.2;
  eqBand1.boost = 0;

  eqBand2.freq = 220;
  eqBand2.Q = 1.5;
  eqBand2.boost = 0;

  eqBand3.freq = 300;
  eqBand3.Q = 1.7;
  eqBand3.boost = 0;

  eqBand4.freq = 430;
  eqBand4.Q = 1.8;
  eqBand4.boost = 0;

  eqBand5.freq = 600;
  eqBand5.Q = 2.0;
  eqBand5.boost = 0;

  eqBand6.freq = 850;
  eqBand6.Q = 2.2;
  eqBand6.boost = 0;

  eqBand7.freq = 1200;
  eqBand7.Q = 2.5;
  eqBand7.boost = 0;

  eqBand8.freq = 1700;
  eqBand8.Q = 2.7;
  eqBand8.boost = 0;

  eqBand9.freq = 2400;
  eqBand9.Q = 2.8;
  eqBand9.boost = 0;

  eqBand10.freq = 3300;
  eqBand10.Q = 3.0;
  eqBand10.boost = 0;

  eqBand11.freq = 4700;
  eqBand11.Q = 3.2;
  eqBand11.boost = 0;

  // mute mic input
  mute_input(true);

  // get first preset
  load_presets(0);

  // set all defaults that are not set already

  set_main_labels();
  show_tx_led(false);

  // max is 16 per module, clipping above 4
  set_hp_out_gain(4);

  set_compressor_labels(0);
  set_compressor_labels(1);

  set_eq_labels(0);

  set_presets_selection_buttons_state();
  load_presets_names();
  set_presets_labels();
  set_presetsmain_labels();

  set_eot_beep_options();
  set_eot_soundclip_options();

  show_recorder_message(false);
  set_voice_selection_buttons_state();
  load_voice_names();
  set_voice_labels();
  show_voicerecorder_message(false);

  load_morse_names();
  set_morse_labels();
  set_morse_selection_buttons_state();

  update_test_signal_gain();
  lv_spinbox_set_value(spinbox_test_frequency_1, speech_test_frequency1);
  lv_spinbox_set_value(spinbox_test_frequency_2, speech_test_frequency2);
  set_test_mode_buttons_state();

  update_eot_level();

  update_eot_mp3_level();

  // set the gain to 1.2 since we are mixing for echo later on and that reduces the amplitude so this factor will compensate for the mixer
  // for non mix signals we can reduce the gain slider a bit to compensate for this increase or boost a little
  set_esp32_amp_gain(1.2);

  // place this here since set_test_mode_buttons_state() is changing the routing otherwise no sound on startup
  select_output_source(0);

  // I2S Configuration, some fixes required for S3 module
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX | I2S_MODE_TX), // RX and TX modes
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = I2S_BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = true, // Clear TX descriptor on underflow
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_BCK,
    .ws_io_num = PIN_WS,
    .data_out_num = PIN_DATA_OUT,
    .data_in_num = PIN_DATA_IN
  };

  esp_err_t error_code;

  error_code = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);

  if (error_code != 0) {
    Serial.println("i2s install error");
    Serial.println(error_code);
  }

  error_code = i2s_set_pin(I2S_NUM, &pin_config);

  if (error_code != 0) {
    Serial.println("i2s set error");
    Serial.println(error_code);
  }

  // eot allways on in the background waiting for a trigger
  xTaskCreate(eot_task, "RogerBeep", 2048, NULL, 0, NULL);

  xTaskCreate(morse_task, "MorseBeep", 2048, NULL, 0, NULL);

  BaseType_t xret;
  xret = xTaskCreatePinnedToCore(
           echo_sb_task,         // Task function
           "I2S Task",      // Task name
           4096,            // Stack size
           NULL,            // Parameters
           1,               // Priority
           &echo_sb_TaskHandle,  // Task handle
           0                // Core to pin the task
         );

  //  print_memory_info_to_serial();

  // Setup the autokey timer once, but don't start it yet
  const esp_timer_create_args_t timer_args = {
    .callback = &speech_autokey_timer_callback,
    .name = "speech_autokey_timer"
  };
  esp_timer_create(&timer_args, &speech_autokey_timer);

  // start all timers
  speech_ptt_record_debounce_timer = millis();

  speech_vu_update_timer = millis() - 200;

  speech_scope_timer = millis() - 200;

  speech_vox_looptimer = millis();

  key1_timer = millis();

  key2_timer = millis();

  LVGL_Timer = millis() - 200;

  lastTickMillis = millis();

  // ptt button
  attachInterrupt(PTT_IN, ptt_interrupt, CHANGE);

}

void loop() {

  // sometimes too much ptt key bounce causes the ptt isr to fail, this is a catch for stuck ptt
  if ((speech_ptt_mode == 0 or speech_ptt_mode == 1) and speech_ptt_active and speech_transmitting and digitalRead(PTT_IN)) {
    if (millis() - ptt_stuck_timer > 150) {
      speech_ptt_active = false;
      speech_ptt_active_last = speech_ptt_active;
      speech_ptt_event_flag = true;
      Serial.println("ptt catch");
    }
  }
  else {
    ptt_stuck_timer = millis();
  }

  // ptt logic
  if (speech_ptt_event_flag) {
    speech_ptt_event_flag = false;
    switch (speech_ptt_mode) {
      case 0:
        // normal mode
        if (speech_ptt_active and !speech_transmitting) {
          if (lv_obj_get_state(cb_vox_on) & LV_STATE_CHECKED) {
            // vox is activated so unmute input
            if (speech_input_muted) {
              mute_input(false);
            }
            speech_vox_state = 1;
          }
          else {
            speech_relay_action = 2;
          }
        }
        else {
          if (lv_obj_get_state(cb_vox_on) & LV_STATE_CHECKED) {
            if (!speech_ptt_active) {
              speech_vox_state = 3;
            }
          }
          else {
            if (!speech_ptt_active and speech_transmitting) {
              speech_relay_action = 1;
            }
          }
        }
        break;

      case 1:
        // delayed off mode
        if (speech_ptt_active and !speech_transmitting) {
          speech_relay_action = 2;
        }
        // handle off elsewhere
        else {
          if (!speech_ptt_active and speech_transmitting) {
            if (speech_eot_state == 0) {
              speech_eot_state = 1;
            }
          }
        }
        break;

      case 2:
        // no change but turn tx off if on
        if (speech_transmitting) {
          speech_relay_action = 1;
        }
        break;

      case 3:
        // esp controlled test signal
        if (speech_ptt_active) {
          speech_relay_action = 2;
        }
        else {
          speech_relay_action = 1;
        }
        break;

      case 4:
        // audio recording mode
        // 0 = no action
        // 1 = record activated
        // 2 = record active
        // 3 = record deactivated
        // 4 = playback activated
        // 5 = playback active
        // 6 = playback deactivated

        if (speech_ptt_active) {
          if (speech_rec_pb_state == 0) {
            // do not accept ptt within this time
            if (millis() - speech_ptt_record_debounce_timer > 1000) {
              speech_rec_pb_state = 1;
            }
          }
          else {
            // if we are not in playback mode
            if (speech_rec_pb_state < 4) {
              // if we are for whatever reason not in idle mode, cancel whatever is running
              speech_rec_pb_state = 3;
            }
          }
          speech_ptt_record_debounce_timer = millis();
        }
        else {
          // on ptt release, immediate action
          if (speech_rec_pb_state == 3) {
            // no action if already in terminating mode
          }
          else {
            if (speech_rec_pb_state == 1 or speech_rec_pb_state == 2) {
              speech_rec_pb_state = 3;
            }
          }
        }

        break;
    }
  }


  // autokey activity
  if (speech_autokey_enabled and lv_tabview_get_tab_active(tabview) == TAB_AUTOKEY_REF) {
    if (speech_autokey_trigger) {
      // start transmitting
      // check for overlap
      if (!speech_transmitting) {
        transmit(true);
        autokey_play();
      }
      // clear flag
      speech_autokey_trigger = false;
    }
    if (speech_autokey_countdown_time != speech_autokey_previous_countdown_time) {
      display_autokey_timers();
      speech_autokey_previous_countdown_time = speech_autokey_countdown_time;
    }
    // check if stop is flagged
    if (speech_autokey_stopped) {
      speech_autokey_stopped = false;
      autokey_stopped();
    }
  }

  // vox mode
  if (speech_vox_state != 0) {
    // frequent calls to vu_meter_read will cause a light wistle so we time them
    if (millis() - speech_vox_looptimer > 20) {
      if (speech_vox_state == 1) {
        // vox is on but not triggered
        // vu meter is 0-50, slider is 0-50
        vu_meter_read();
        if (speech_vu_meter >= speech_vox_threshold_slider) {
          // above level
          speech_vox_state = 2;
          // turn on relay
          speech_relay_action = 2;
          speech_vox_timer = millis();
        }
      }
      else {
        if (speech_vox_state == 2) {
          // vox is triggered so check timing
          vu_meter_read();
          if (speech_vu_meter >= speech_vox_threshold_slider) {
            // still above level
            speech_vox_timer = millis();
          }
          else {
            if (millis() - speech_vox_timer >= 200 * speech_vox_hangtime_slider) {
              // timer elapsed
              speech_relay_action = 1;
              speech_vox_state = 1;
            }
          }
        }
        if (speech_vox_state == 3) {
          speech_relay_action = 1;
          speech_vox_state = 0;
        }
      }
      speech_vox_looptimer = millis();
    }
  }

  if (speech_filetransfer_statuschange) {
    if (speech_filetransfer_state == 1) {
      lv_label_set_text(lbl_filetransfer_info, "Started..waiting for tx");
      filetransfer_set_buttons();
    }
    else {
      if (speech_filetransfer_state == 3) {
        speech_filetransfer_state = 0;
        // lv_label_set_text(lbl_filetransfer_info, "Completed");
        lv_label_set_text(lbl_filetransfer_info, printbuf);
        filetransfer_set_buttons();
        // refresh roller list
        set_eot_soundclip_options();
      }
    }
    speech_filetransfer_statuschange = false;
  }

  if (speech_filetransfer_state == 1 and strlen(printbuf) > 0) {
    lv_label_set_text(lbl_filetransfer_info, printbuf);
  }

  tab_c = lv_tabview_get_tab_active(tabview);
  if (tab_c == TAB_MAIN_REF or tab_c == TAB_COMPRESSOR_REF or tab_c == TAB_SETAMPLIFIER_REF or tab_c == TAB_SETVURANGE_REF) {
    if (speech_transmitting) {
      if (millis() - speech_vu_update_timer > 50) {
        // only update if changed
        int8_t speech_vu_meter_previous = speech_vu_meter;
        vu_meter_read();
        if (speech_vu_meter != speech_vu_meter_previous) {
          VU_meter_update();
        }
        speech_vu_update_timer = millis();
      }
    }
    else
    {
      // make sure vu meter is cleared if tx done
      if (speech_vu_meter > 0) {
        speech_vu_meter = 0;
        VU_meter_update();
      }
    }
  }

  if (speech_eot_state == 1) {
    if (lv_tabview_get_tab_active(tabview) == TAB_ENDOFTRANSMISSION_REF) {
      set_eot_selection_buttons_state();
    }
    eot_play();
  }

  // end of eot actions
  if (speech_eot_state == 3) {
    // reset state
    speech_eot_state = 0;
    set_eot_selection_buttons_state();
  }

  if (!speech_lock) {
    // relay action flag from ptt or other relay related actions
    if (speech_relay_action != 0) {
      // 1 = turn off
      if (speech_relay_action == 1) {
        // do not mute if in vox mode
        if (speech_vox_state == 0) {
          if (!speech_input_muted) {
            mute_input(true);
          }
        }
        transmit(false);
        if (lv_tabview_get_tab_active(tabview) == TAB_TEST_REF) {
          // do this here otherwise it crashes in the interrupt routine
          lv_obj_clear_state(btn_test_tx, LV_STATE_CHECKED);
          lv_obj_clear_state(btn_test_tx, LV_STATE_DISABLED);
        }
        // clear vu meter
        tab_c = lv_tabview_get_tab_active(tabview);
        if (tab_c == TAB_MAIN_REF or tab_c == TAB_COMPRESSOR_REF or tab_c == TAB_SETAMPLIFIER_REF or tab_c == TAB_SETVURANGE_REF) {
          speech_vu_meter = 0;
          VU_meter_update();
        }
        // clear scope
        if (lv_tabview_get_tab_active(tabview) == TAB_SCOPE_REF) {
          scope_clear();
        }
        // clear spectrum
        if (lv_tabview_get_tab_active(tabview) == TAB_SPECTRUM_REF) {
          spectrum_clear();
        }
        // reset eot lock for echo
        speech_eot_block = false;
        if (speech_vox_state > 1) {
          speech_vox_state = 1;
        }
      } else {
        // 2 = turn on
        if (speech_input_muted) {
          mute_input(false);
        }
        transmit(true);
        if (lv_tabview_get_tab_active(tabview) == TAB_TEST_REF and speech_ptt_active) {
          lv_obj_clear_state(btn_test_tx, LV_STATE_CHECKED);
          lv_obj_add_state(btn_test_tx, LV_STATE_DISABLED);
        }
      }
      // clear action
      speech_relay_action = 0;
    }

    // check for record/playback actions, make a copy since it can be modified by isr
    speech_rec_pb_state_snap = speech_rec_pb_state;
    if (speech_rec_pb_state_snap != 0) {
      // 0 = no action
      // 1 = record activated
      // 2 = record active
      // 3 = record deactivated
      // 4 = playback activated
      // 5 = playback active
      // 6 = playback deactivated

      // record/playback screen
      if (lv_tabview_get_tab_active(tabview) == TAB_RECORDER_REF) {
        switch (speech_rec_pb_state_snap) {
          case 1:
            // record mode is activated by ptt
            show_recorder_message(true);
            // disable buttons
            lv_obj_add_state(btn_recorder_play, LV_STATE_DISABLED);
            lv_obj_add_state(btn_recorder_erase, LV_STATE_DISABLED);
            lv_obj_add_state(btn_recorder_cancel, LV_STATE_DISABLED);
            lv_obj_add_state(btn_recorder_back, LV_STATE_DISABLED);
            lv_obj_add_state(cb_recorder_tx_on, LV_STATE_DISABLED);
            lv_obj_add_state(cb_recorder_bypass_on, LV_STATE_DISABLED);
            // reset progress bar
            speech_rec_pb_progress = 0;
            update_recorder_bar();
            speech_rec_pb_progress_timer = millis();
            speech_rec_pb_state = 2;
            speech_recording_playback_active = true;
            // start tasks and resume loop
            speech_selected_file = 0;
            if (speech_input_muted) {
              mute_input(false);
            }
            setup_recording();

            //   print_memory_info_to_serial();

            break;

          case 3:
            // record mode is deactivated
            // end only when tasks are complete, tasks will end by ptt or end of recording time
            if (speech_task1_completed and speech_task2_completed) {
              show_recorder_message(false);
              speech_rec_pb_state = 0;
              speech_recording_playback_active = false;
              // enable buttons
              lv_obj_clear_state(btn_recorder_play, LV_STATE_DISABLED);
              lv_obj_clear_state(btn_recorder_erase, LV_STATE_DISABLED);

              lv_obj_add_state(btn_recorder_cancel, LV_STATE_DISABLED);
              lv_obj_clear_state(btn_recorder_back, LV_STATE_DISABLED);
              lv_obj_clear_state(cb_recorder_tx_on, LV_STATE_DISABLED);
              lv_obj_clear_state(cb_recorder_bypass_on, LV_STATE_DISABLED);
              if (!speech_input_muted) {
                mute_input(true);
              }
            }
            break;

          case 4:
            // playback mode is activated by button
            //  show_recorder_message(true);
            // disable buttons
            lv_obj_add_state(btn_recorder_play, LV_STATE_DISABLED);
            lv_obj_add_state(btn_recorder_erase, LV_STATE_DISABLED);
            lv_obj_clear_state(btn_recorder_cancel, LV_STATE_DISABLED);
            lv_obj_add_state(btn_recorder_back, LV_STATE_DISABLED);
            lv_obj_add_state(cb_recorder_tx_on, LV_STATE_DISABLED);
            lv_obj_add_state(cb_recorder_bypass_on, LV_STATE_DISABLED);
            // reset progress bar
            speech_rec_pb_progress = 0;
            update_recorder_bar();
            speech_rec_pb_progress_timer = millis();
            speech_rec_pb_state = 5;
            speech_recording_playback_active = true;
            // start tasks and resume loop
            speech_selected_file = 0;
            setup_playback();
            break;

          case 6:
            // playback mode is deactivated
            // end only when tasks are complete, tasks will end by end of file
            if (speech_task1_completed and speech_task2_completed) {
              speech_rec_pb_state = 0;
              speech_recording_playback_active = false;
              // enable buttons
              lv_obj_clear_state(btn_recorder_play, LV_STATE_DISABLED);
              lv_obj_clear_state(btn_recorder_erase, LV_STATE_DISABLED);
              lv_obj_add_state(btn_recorder_cancel, LV_STATE_DISABLED);
              lv_obj_clear_state(btn_recorder_back, LV_STATE_DISABLED);
              lv_obj_clear_state(cb_recorder_tx_on, LV_STATE_DISABLED);
              lv_obj_clear_state(cb_recorder_bypass_on, LV_STATE_DISABLED);
            }
            break;
        }

        // recording or playback running, update progress bar
        if (speech_rec_pb_state_snap == 2 or speech_rec_pb_state_snap == 5) {
          // 20 seconds upto 100%, due to core timing issues update every 400ms iso 200
          if (millis() - speech_rec_pb_progress_timer >= 400) {
            speech_rec_pb_progress = (millis() - speech_rec_pb_timer) / 200;
            update_recorder_bar();
            // restart update timer
            speech_rec_pb_progress_timer = millis();
            if (speech_rec_pb_progress > 100) {
              // done recording
              if (speech_rec_pb_state_snap == 2) {
                speech_rec_pb_state = 3;
              }
              else {
                speech_rec_pb_state = 6;
              }
            }
          }
        }
      }
      else {
        if (lv_tabview_get_tab_active(tabview) == TAB_VOICE_REF) {
          switch (speech_rec_pb_state_snap) {
            case 1:
              // record mode is activated by ptt
              show_voicerecorder_message(true);
              // disable buttons
              lv_obj_add_state(btn_voice_play, LV_STATE_DISABLED);
              lv_obj_add_state(btn_voice_cancel, LV_STATE_DISABLED);
              lv_obj_add_state(btn_voice_back, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_tx_on, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_selection1, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_selection2, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_selection3, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_selection4, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_selection5, LV_STATE_DISABLED);
              // reset progress bar
              speech_voice_progress = 0;
              update_voice_bar();
              speech_voice_progress_timer = millis();
              speech_rec_pb_state = 2;
              speech_recording_playback_active = true;
              // start tasks and resume loop
              speech_selected_file = speech_voice_msg_sel;
              if (speech_input_muted) {
                mute_input(false);
              }
              setup_recording();
              break;

            case 3:
              // record mode is deactivated
              // end only when tasks are complete, tasks will end by ptt or end of recording time
              if (speech_task1_completed and speech_task2_completed) {
                show_voicerecorder_message(false);
                speech_rec_pb_state = 0;
                speech_recording_playback_active = false;
                // enable buttons
                lv_obj_clear_state(btn_voice_play, LV_STATE_DISABLED);
                //  lv_obj_clear_state(btn_voice_cancel, LV_STATE_DISABLED);
                lv_obj_add_state(btn_voice_cancel, LV_STATE_DISABLED);
                lv_obj_clear_state(btn_voice_back, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_tx_on, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_selection1, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_selection2, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_selection3, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_selection4, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_selection5, LV_STATE_DISABLED);
                if (!speech_input_muted) {
                  mute_input(true);
                }
              }
              break;

            case 4:
              // playback mode is activated by button
              //  show_recorder_message(true);
              // disable buttons
              lv_obj_add_state(btn_voice_play, LV_STATE_DISABLED);
              lv_obj_clear_state(btn_voice_cancel, LV_STATE_DISABLED);
              lv_obj_add_state(btn_voice_back, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_tx_on, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_selection1, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_selection2, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_selection3, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_selection4, LV_STATE_DISABLED);
              lv_obj_add_state(cb_voice_selection5, LV_STATE_DISABLED);
              // reset progress bar
              speech_voice_progress = 0;
              update_voice_bar();
              speech_voice_progress_timer = millis();
              speech_rec_pb_state = 5;
              speech_recording_playback_active = true;
              // start tasks and resume loop
              speech_selected_file = speech_voice_msg_sel;
              setup_playback();
              break;

            case 6:
              // playback mode is deactivated
              // end only when tasks are complete, tasks will end by end of file
              if (speech_task1_completed and speech_task2_completed) {
                //  show_recorder_message(false);
                speech_rec_pb_state = 0;
                speech_recording_playback_active = false;
                // enable buttons
                lv_obj_clear_state(btn_voice_play, LV_STATE_DISABLED);
                //lv_obj_clear_state(btn_voice_cancel, LV_STATE_DISABLED);
                lv_obj_add_state(btn_voice_cancel, LV_STATE_DISABLED);
                lv_obj_clear_state(btn_voice_back, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_tx_on, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_selection1, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_selection2, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_selection3, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_selection4, LV_STATE_DISABLED);
                lv_obj_clear_state(cb_voice_selection5, LV_STATE_DISABLED);
              }
              break;
          }

          // recording or playback running, update progress bar
          if (speech_rec_pb_state_snap == 2 or speech_rec_pb_state_snap == 5) {
            // 20 seconds upto 100%, due to core timing issues update every 400ms iso 200
            if (millis() - speech_voice_progress_timer >= 400) {
              speech_voice_progress = (millis() - speech_rec_pb_timer) / 200;
              update_voice_bar();
              // restart update timer
              speech_voice_progress_timer = millis();
              if (speech_voice_progress > 100) {
                // done recording
                if (speech_rec_pb_state_snap == 2) {
                  speech_rec_pb_state = 3;
                }
                else {
                  speech_rec_pb_state = 6;
                }
              }
            }
          }
        }
      }
    }

    // morse state machine
    // 11 is morse setup screen
    if (lv_tabview_get_tab_active(tabview) == TAB_MORSE_REF) {
      switch (speech_morse_state) {
        // speech_morse_state
        // 0 nothing
        // 1 playback start
        // 2 playback running
        // 3 playback completed

        case 0:
          break;

        case 1:
          lv_obj_add_state(btn_morse_set_level, LV_STATE_DISABLED);
          lv_obj_add_state(btn_morse_play, LV_STATE_DISABLED);
          lv_obj_add_state(btn_morse_back, LV_STATE_DISABLED);
          lv_obj_add_state(cb_morse_selection1, LV_STATE_DISABLED);
          lv_obj_add_state(cb_morse_selection2, LV_STATE_DISABLED);
          lv_obj_add_state(cb_morse_selection3, LV_STATE_DISABLED);
          lv_obj_add_state(cb_morse_selection4, LV_STATE_DISABLED);
          lv_obj_add_state(cb_morse_selection5, LV_STATE_DISABLED);
          lv_obj_add_state(cb_morse_tx_on, LV_STATE_DISABLED);
          if (lv_obj_get_state(cb_morse_tx_on) & LV_STATE_CHECKED) {
            transmit(true);
          }
          speech_morse_play = speech_morse_msg[speech_morse_msg_sel - 1];
          xTaskNotifyGive(xTaskGetHandle("MorseBeep")); // trigger morse code
          speech_morse_state = 2;
          break;

        case 2:
          break;

        case 3:
          lv_obj_clear_state(btn_morse_set_level, LV_STATE_DISABLED);
          lv_obj_clear_state(btn_morse_play, LV_STATE_DISABLED);
          lv_obj_clear_state(btn_morse_back, LV_STATE_DISABLED);
          lv_obj_clear_state(cb_morse_selection1, LV_STATE_DISABLED);
          lv_obj_clear_state(cb_morse_selection2, LV_STATE_DISABLED);
          lv_obj_clear_state(cb_morse_selection3, LV_STATE_DISABLED);
          lv_obj_clear_state(cb_morse_selection4, LV_STATE_DISABLED);
          lv_obj_clear_state(cb_morse_selection5, LV_STATE_DISABLED);
          lv_obj_clear_state(cb_morse_tx_on, LV_STATE_DISABLED);
          speech_morse_state = 0;
          break;
      }
    }

    // autokey screen
    if (lv_tabview_get_tab_active(tabview) == TAB_AUTOKEY_REF) {
      switch (speech_morse_state) {
        // speech_morse_state
        // 0 nothing
        // 1 playback start
        // 2 playback running
        // 3 playback completed

        case 0:
          break;

        case 1:
          xTaskNotifyGive(xTaskGetHandle("MorseBeep")); // Trigger morse code
          speech_morse_state = 2;
          break;

        case 2:
          break;

        case 3:
          speech_morse_state = 0;
          break;
      }
    }

    // check encoder buttons
    if (digitalRead(Encoder_1_Key) == 0) {
      timer_encoderbutton1 = millis();
      Encoder_Key1_Long_Press = false;
      // wait until key is no longer pressed or time expired
      while (digitalRead(Encoder_1_Key) == 0) {
        if (millis() - timer_encoderbutton1 > 1000) {
          Encoder_Key1_Long_Press = true;
          break;
        }
      }
      if (Encoder_Key1_Long_Press) {
        // long press
        while (digitalRead(Encoder_1_Key) == 0) {}
        //little debounce
        delay(200);
      }
      else {
        // short press
        switch (lv_tabview_get_tab_active(tabview)) {
          case TAB_MAIN_REF:
            checkbox_toggle(cb_main_eot_on);
            // force check since LV_EVENT_CLICKED does not pick it up and LV_EVENT_VALUE_CHANGED is triggered multiple times
            lv_obj_send_event(cb_main_eot_on, LV_EVENT_CLICKED, NULL);
            break;

          case TAB_COMPRESSOR_REF:
            // compressor screen
            speech_rotary_selected_threshold = !speech_rotary_selected_threshold;
            set_compressor_labels(0);
            break;

          case TAB_EFFECTS_REF:
            // effects screen

            break;

          case TAB_ENDOFTRANSMISSION_REF:
            if (speech_eot_state == 0) {
              if (speech_eot_selection == 0) {
                // lv_obj_clear_state(cb_eot_sel_beep, LV_STATE_CHECKED);
                // lv_obj_add_state(cb_eot_sel_sound, LV_STATE_CHECKED);
                speech_eot_selection = 1;
              }
              else {
                //lv_obj_clear_state(cb_eot_sel_sound, LV_STATE_CHECKED);
                // lv_obj_add_state(cb_eot_sel_beep, LV_STATE_CHECKED);
                speech_eot_selection = 0;
              }
              set_eot_selection_buttons_state();
            }
            break;

          case TAB_SCOPE_REF:
            // scope screen
            speech_scope_autoscale = true;
            break;

          case TAB_SPECTRUM_REF:
            // spectrum screen
            speech_spectrum_peaktrigger = true;
            break;

          case TAB_VOX_REF:
            checkbox_toggle(cb_vox_on);
            // force check since LV_EVENT_CLICKED does not pick it up and LV_EVENT_VALUE_CHANGED is triggered multiple times
            lv_obj_send_event(cb_vox_on, LV_EVENT_CLICKED, NULL);
            break;

          case TAB_TEST_REF:
            // effects screen
            if (speech_test_mode < 2) {
              speech_test_mode++;
            }
            else {
              speech_test_mode = 0;
            }
            set_test_mode_buttons_state();
            break;
        }
      }
    }

    if (digitalRead(Encoder_2_Key) == 0) {
      timer_encoderbutton2 = millis();
      Encoder_Key2_Long_Press = false;
      // wait until key is no longer pressed or time expired
      while (digitalRead(Encoder_2_Key) == 0) {
        if (millis() - timer_encoderbutton2 > 1000) {
          Encoder_Key2_Long_Press = true;
          break;
        }
      }
      if (Encoder_Key2_Long_Press) {
        // long press
        while (digitalRead(Encoder_2_Key) == 0) {}
        //little debounce
        delay(200);
      }
      else {
        // short press
        switch (lv_tabview_get_tab_active(tabview)) {
          case TAB_MAIN_REF:
            // main screen
            speech_rotary_selected_output = !speech_rotary_selected_output;
            set_main_labels();
            break;

          case TAB_COMPRESSOR_REF:
            // compressor screen
            speech_rotary_selected_postgain = !speech_rotary_selected_postgain;
            set_compressor_labels(1);
            break;

          case TAB_ENDOFTRANSMISSION_REF:
            checkbox_toggle(cb_eot_on);
            // force check since LV_EVENT_CLICKED does not pick it up and LV_EVENT_VALUE_CHANGED is triggered multiple times
            lv_obj_send_event(cb_eot_on, LV_EVENT_CLICKED, NULL);
            break;

          case TAB_TEST_REF:
            switch (speech_test_mode) {
              case 0:
                // nothing selected
                speech_test_enc2_mode = 0;
                break;

              case 1:
                // only f1
                speech_test_enc2_mode = 1;
                break;

              case 2:
                // can be both so toggle
                if (speech_test_enc2_mode == 1) {
                  speech_test_enc2_mode = 2;
                  lv_spinbox_show_cursor(spinbox_test_frequency_1, false);
                  lv_spinbox_show_cursor(spinbox_test_frequency_2, true);
                }
                else {
                  speech_test_enc2_mode = 1;
                  lv_spinbox_show_cursor(spinbox_test_frequency_1, true);
                  lv_spinbox_show_cursor(spinbox_test_frequency_2, false);
                }
                break;

              case 3:
                // nothing selected
                speech_test_enc2_mode = 0;
                break;
            }
            break;
        }
      }
    }

    // check rotary encoders
    if (EncoderCounter1 != 0) {
      switch (lv_tabview_get_tab_active(tabview)) {
        case TAB_MAIN_REF:
          // main screen
          if (EncoderCounter1 > 0) {
            if (speech_gain < lv_slider_get_max_value(slider_main_gain)) {
              speech_gain++;
              update_gain();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_gain > 0) {
                speech_gain--;
                update_gain();
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_COMPRESSOR_REF:
          // compressor screen
          if (speech_rotary_selected_threshold) {
            if (EncoderCounter1 > 0) {
              if (speech_compressor_threshold < lv_slider_get_max_value(slider_compressor_threshold)) {
                speech_compressor_threshold++;
                update_compressor_threshold();
                speech_compressor_update();
              }
            }
            else {
              if (EncoderCounter1 < 0) {
                if (speech_compressor_threshold > 0) {
                  speech_compressor_threshold--;
                  update_compressor_threshold();
                  speech_compressor_update();
                }
              }
            }
          }
          else {
            if (EncoderCounter1 > 0) {
              if (speech_compressor_ratio  < lv_slider_get_max_value(slider_compressor_ratio)) {
                speech_compressor_ratio ++;
                update_compressor_ratio();
                speech_compressor_update();
              }
            }
            else {
              if (EncoderCounter1 < 0) {
                if (speech_compressor_ratio  > 1) {
                  speech_compressor_ratio --;
                  update_compressor_ratio();
                  speech_compressor_update();
                }
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_EQUALIZER_REF:
          // eq screen
          if (EncoderCounter1 > 0) {
            inc_eq_slider(speech_selected_eq);
          }
          else {
            if (EncoderCounter1 < 0) {
              dec_eq_slider(speech_selected_eq);
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_EFFECTS_REF:
          // effects screen
          switch (speech_effect_selected_slider) {
            case 1:
              if (EncoderCounter1 > 0) {
                lv_slider_set_value(slider_effect_echo_time, lv_slider_get_value(slider_effect_echo_time) + 1, LV_ANIM_OFF);
              }
              else {
                if (EncoderCounter1 < 0) {
                  lv_slider_set_value(slider_effect_echo_time, lv_slider_get_value(slider_effect_echo_time) - 1, LV_ANIM_OFF);
                }
              }
              lv_obj_send_event(slider_effect_echo_time, LV_EVENT_VALUE_CHANGED, NULL);
              break;

            case 2:
              if (EncoderCounter1 > 0) {
                lv_slider_set_value(slider_effect_echo_fb, lv_slider_get_value(slider_effect_echo_fb) + 1, LV_ANIM_OFF);
              }
              else {
                if (EncoderCounter1 < 0) {
                  lv_slider_set_value(slider_effect_echo_fb, lv_slider_get_value(slider_effect_echo_fb) - 1, LV_ANIM_OFF);
                }
              }
              lv_obj_send_event(slider_effect_echo_fb, LV_EVENT_VALUE_CHANGED, NULL);
              break;

            case 3:
              if (EncoderCounter1 > 0) {
                lv_slider_set_value(slider_effect_pitch, lv_slider_get_value(slider_effect_pitch) + 1, LV_ANIM_OFF);
              }
              else {
                if (EncoderCounter1 < 0) {
                  lv_slider_set_value(slider_effect_pitch, lv_slider_get_value(slider_effect_pitch) - 1, LV_ANIM_OFF);
                }
              }
              lv_obj_send_event(slider_effect_pitch, LV_EVENT_VALUE_CHANGED, NULL);
              break;

            case 4:
              if (EncoderCounter1 > 0) {
                lv_slider_set_value(slider_effect_ring, lv_slider_get_value(slider_effect_ring) + 1, LV_ANIM_OFF);
              }
              else {
                if (EncoderCounter1 < 0) {
                  lv_slider_set_value(slider_effect_ring, lv_slider_get_value(slider_effect_ring) - 1, LV_ANIM_OFF);
                }
              }
              lv_obj_send_event(slider_effect_ring, LV_EVENT_VALUE_CHANGED, NULL);
              break;

          }
          EncoderCounter1 = 0;
          break;

        case TAB_ENDOFTRANSMISSION_REF:
          // eot screen
          if (speech_eot_state == 0) {
            if (speech_eot_selection == 0) {
              if (EncoderCounter1 > 0) {
                if (speech_eot_beep_selected < EOT_BEEP_COUNT) {
                  speech_eot_beep_selected++;
                }
                else {
                  speech_eot_beep_selected = 1;
                }
                set_eot_beep_roller();
              }
              else {
                if (EncoderCounter1 < 0) {
                  if (speech_eot_beep_selected > 1) {
                    speech_eot_beep_selected--;
                  }
                  else {
                    speech_eot_beep_selected = EOT_BEEP_COUNT;
                  }
                  set_eot_beep_roller();
                }
              }
            }
            else {
              if (EncoderCounter1 > 0) {
                if (speech_eot_sound_selected < speech_eot_sound_count) {
                  speech_eot_sound_selected++;
                }
                else {
                  speech_eot_sound_selected = 1;
                }
                set_eot_sound_roller();
              }
              else {
                if (EncoderCounter1 < 0) {
                  if (speech_eot_sound_selected > 1) {
                    speech_eot_sound_selected--;
                  }
                  else {
                    speech_eot_sound_selected = speech_eot_sound_count;
                  }
                  set_eot_sound_roller();
                }
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_AUTOKEY_REF:
          // autokey
          if (!speech_autokey_enabled) {
            if (EncoderCounter1 > 0) {
              lv_spinbox_increment(spinbox_autokey_txtime);
            }
            else {
              if (EncoderCounter1 < 0) {
                lv_spinbox_decrement(spinbox_autokey_txtime);
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_VOICE_REF:
          // voice message screen
          if (EncoderCounter1 > 0) {
            if (speech_voice_msg_sel < 5) {
              speech_voice_msg_sel++;
              set_voice_selection_buttons_state();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_voice_msg_sel > 1) {
                speech_voice_msg_sel--;
                set_voice_selection_buttons_state();
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_MORSE_REF:
          // morse message screen
          if (EncoderCounter1 > 0) {
            if (speech_morse_msg_sel < 5) {
              speech_morse_msg_sel++;
              set_morse_selection_buttons_state();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_morse_msg_sel > 1) {
                speech_morse_msg_sel--;
                set_morse_selection_buttons_state();
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_VOX_REF:
          // vox screen
          if (EncoderCounter1 > 0) {
            if (speech_vox_threshold_slider  < lv_slider_get_max_value(slider_vox_threshold)) {
              speech_vox_threshold_slider ++;
              vox_set_threshold();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_vox_threshold_slider  > 0) {
                speech_vox_threshold_slider --;
                vox_set_threshold();
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_SCOPE_REF:
          // scope screen
          if (EncoderCounter1 > 0) {
            if (speech_scope_scale < 70000 - (speech_scope_scale / 10)) {
              speech_scope_scale += speech_scope_scale / 10;
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_scope_scale > 120 + (speech_scope_scale / 10)) {
                speech_scope_scale -= (speech_scope_scale / 10);
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_SPECTRUM_REF:
          // spectrum screen
          if (EncoderCounter1 > 0) {
            if (speech_spectrum_cursor < 118) {
              speech_spectrum_cursor++;
              spectrum_update_cursor(speech_spectrum_cursor);
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_spectrum_cursor > 0) {
                speech_spectrum_cursor--;
                spectrum_update_cursor(speech_spectrum_cursor);
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_TEST_REF:
          // test signal screen
          if (EncoderCounter1 > 0) {
            if (speech_test_signal_gain < lv_slider_get_max_value(slider_test_signal_gain)) {
              speech_test_signal_gain++;
              update_test_signal_gain();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_test_signal_gain > 0) {
                speech_test_signal_gain--;
                update_test_signal_gain();
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_DELETE_REF:
          // file delete screen
          if (EncoderCounter1 > 0) {
            if (speech_filedelete_selected < speech_filedelete_count - 1) {
              speech_filedelete_selected++;
              set_filedelete_selected();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_filedelete_selected > 0) {
                speech_filedelete_selected--;
                set_filedelete_selected();
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_SETEOTLEVEL_REF:
          // eot level adjust
          if (EncoderCounter1 > 0) {
            if (speech_eot_level < lv_slider_get_max_value(slider_set_eot_level)) {
              speech_eot_level++;
              update_eot_level();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_eot_level > 0) {
                speech_eot_level--;
                update_eot_level();
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_SETAMPLIFIER_REF:
          // amp factor adjust
          if (EncoderCounter1 > 0) {
            if (speech_amplifier < lv_slider_get_max_value(slider_set_amplifier)) {
              speech_amplifier++;
              update_amp_level();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_amplifier > 1) {
                speech_amplifier--;
                update_amp_level();
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_SETVURANGE_REF:
          // vu range factor adjust
          if (EncoderCounter1 > 0) {
            if (speech_vu_scaling < lv_slider_get_max_value(slider_set_vu_range)) {
              speech_vu_scaling++;
              update_vu_level();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_vu_scaling > 0) {
                speech_vu_scaling--;
                update_vu_level();
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_SETMORSELEVEL_REF:
          // morse level adjust
          if (EncoderCounter1 > 0) {
            if (speech_morse_level < lv_slider_get_max_value(slider_set_morse_level)) {
              speech_morse_level++;
              update_morse_level();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_morse_level > 0) {
                speech_morse_level--;
                update_morse_level();
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        case TAB_SETAUTOKEYLEVEL_REF:
          // morse level adjust
          if (EncoderCounter1 > 0) {
            if (speech_autokey_level < lv_slider_get_max_value(slider_set_autokey_level)) {
              speech_autokey_level++;
              update_autokey_level();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (speech_autokey_level > 0) {
                speech_autokey_level--;
                update_autokey_level();
              }
            }
          }
          EncoderCounter1 = 0;
          break;

        default:
          // clear if other screens
          EncoderCounter1 = 0;
          break;
      }
    }

    if (EncoderCounter2 != 0) {
      switch (lv_tabview_get_tab_active(tabview)) {
        case TAB_MAIN_REF:
          // main screen
          if (speech_rotary_selected_output) {
            if (EncoderCounter2 > 0) {
              if (speech_output < lv_slider_get_max_value(slider_main_output)) {
                speech_output++;
                update_output();
              }
            }
            else {
              if (EncoderCounter2 < 0) {
                if (speech_output > 0) {
                  speech_output--;
                  update_output();
                }
              }
            }
          }
          else {
            if (EncoderCounter2 > 0) {
              if (speech_headphone < lv_slider_get_max_value(slider_main_headphone)) {
                speech_headphone++;
                update_headphone();
              }
            }
            else {
              if (EncoderCounter2 < 0) {
                if (speech_headphone > 0) {
                  speech_headphone--;
                  update_headphone();
                }
              }
            }
          }
          EncoderCounter2 = 0;
          break;

        case TAB_COMPRESSOR_REF:
          // compressor screen
          if (speech_rotary_selected_postgain) {
            if (EncoderCounter2 > 0) {
              if (speech_compressor_postgain  < lv_slider_get_max_value(slider_compressor_postgain)) {
                speech_compressor_postgain ++;
                update_compressor_postgain();
                speech_compressor_update();
              }
            }
            else {
              if (EncoderCounter2 < 0) {
                if (speech_compressor_postgain  > 1) {
                  speech_compressor_postgain --;
                  update_compressor_postgain();
                  speech_compressor_update();
                }
              }
            }
          }
          else {
            if (EncoderCounter2 > 0) {
              if (speech_compressor_noisegate  < lv_slider_get_max_value(slider_compressor_noisegate)) {
                speech_compressor_noisegate ++;
                update_compressor_noisegate();
                speech_noisegate_update();
              }
            }
            else {
              if (EncoderCounter2 < 0) {
                if (speech_compressor_noisegate  > 1) {
                  speech_compressor_noisegate --;
                  update_compressor_noisegate();
                  speech_noisegate_update();
                }
              }
            }
          }
          EncoderCounter2 = 0;
          break;

        case TAB_EQUALIZER_REF:
          // eq screen
          if (EncoderCounter2 > 0) {
            if (speech_selected_eq < 11) {
              speech_selected_eq++;
              set_eq_labels(speech_selected_eq);
            }
          }
          else {
            if (EncoderCounter2 < 0) {
              if (speech_selected_eq > 0) {
                speech_selected_eq--;
                set_eq_labels(speech_selected_eq);
              }
            }
          }
          EncoderCounter2 = 0;
          break;

        case TAB_EFFECTS_REF:
          // effects screen
          if (EncoderCounter2 > 0) {
            // Positive encoder movement - move to next enabled slider
            int start = speech_effect_selected_slider;
            int current = start;
            do {
              current++;

              // Check if current position is valid and enabled
              if (current == 0) {
                // Skip 0 when moving positive if we're already on a slider
                continue;
              }
              else if (current == 1 || current == 2) {
                // Sliders 1-2 are for echo/sb
                if (speech_effect_echo_on || speech_effect_sb_on) {
                  speech_effect_selected_slider = current;
                  break;
                }
              }
              else if (current == 3) {
                // Slider 3 is for pitch
                if (speech_effect_pitch_on) {
                  speech_effect_selected_slider = current;
                  break;
                }
              }
              else if (current == 4) {
                // Slider 4 is for ring
                if (speech_effect_ring_on) {
                  speech_effect_selected_slider = current;
                  break;
                }
              }

              // If we reach 5, we've wrapped around without finding anything
              if (current >= 5) {
                // Stay at current position if no higher enabled sliders
                break;
              }
            } while (current <= 4);
          }
          else if (EncoderCounter2 < 0) {
            // Negative encoder movement - move to previous enabled slider
            int start = speech_effect_selected_slider;
            int current = start;

            do {
              current--;

              // Check if current position is valid and enabled
              if (current == 0) {
                // Only go to 0 if NO sliders are enabled
                bool anyEnabled = speech_effect_echo_on || speech_effect_sb_on ||
                                  speech_effect_pitch_on || speech_effect_ring_on;
                if (!anyEnabled) {
                  speech_effect_selected_slider = 0;
                  break;
                }
                // If sliders are enabled, skip 0 and continue to negative numbers
                continue;
              }
              else if (current == 1 || current == 2) {
                // Sliders 1-2 are for echo/sb
                if (speech_effect_echo_on || speech_effect_sb_on) {
                  speech_effect_selected_slider = current;
                  break;
                }
              }
              else if (current == 3) {
                // Slider 3 is for pitch
                if (speech_effect_pitch_on) {
                  speech_effect_selected_slider = current;
                  break;
                }
              }
              else if (current == 4) {
                // Slider 4 is for ring
                if (speech_effect_ring_on) {
                  speech_effect_selected_slider = current;
                  break;
                }
              }

              // If we reach -1, we've wrapped around without finding anything
              if (current <= -1) {
                // Stay at current position if no lower enabled sliders
                break;
              }
            } while (current >= 0);
          }

          set_effect_sliders_labels();

          // Reset encoder counter after processing
          EncoderCounter2 = 0;

          break;

        case TAB_AUTOKEY_REF:
          // autokey
          if (!speech_autokey_enabled) {
            if (EncoderCounter2 > 0) {
              lv_spinbox_increment(spinbox_autokey_interval);
            }
            else {
              if (EncoderCounter2 < 0) {
                lv_spinbox_decrement(spinbox_autokey_interval);
              }
            }
          }
          EncoderCounter2 = 0;
          break;

        case TAB_VOX_REF:
          // vox screen
          if (EncoderCounter2 > 0) {
            if (speech_vox_hangtime_slider  < lv_slider_get_max_value(slider_vox_hangtime)) {
              speech_vox_hangtime_slider ++;
              vox_set_hangtime();
            }
          }
          else {
            if (EncoderCounter2 < 0) {
              if (speech_vox_hangtime_slider  > 0) {
                speech_vox_hangtime_slider --;
                vox_set_hangtime();
              }
            }
          }
          EncoderCounter2 = 0;
          break;

        case TAB_TEST_REF:
          // test screen
          switch (speech_test_enc2_mode) {
            case 1:
              if (EncoderCounter2 > 0) {
                if (speech_test_frequency1 <= 20000 - lv_spinbox_get_step(spinbox_test_frequency_1)) {
                  speech_test_frequency1 = speech_test_frequency1 + lv_spinbox_get_step(spinbox_test_frequency_1);
                  lv_spinbox_set_value(spinbox_test_frequency_1, speech_test_frequency1);
                  set_sine_frequency1(speech_test_frequency1);
                }
              }
              else {
                if (EncoderCounter2 < 0) {
                  if (speech_test_frequency1 >= 20 + lv_spinbox_get_step(spinbox_test_frequency_1)) {
                    speech_test_frequency1 = speech_test_frequency1 - lv_spinbox_get_step(spinbox_test_frequency_1);
                    lv_spinbox_set_value(spinbox_test_frequency_1, speech_test_frequency1);
                    set_sine_frequency1(speech_test_frequency1);
                  }
                }
              }
              if (speech_test_cursor_override) {
                lv_spinbox_show_cursor(spinbox_test_frequency_1, true);
                lv_spinbox_show_cursor(spinbox_test_frequency_2, false);
                speech_test_cursor_override = false;
              }
              break;

            case 2:
              if (EncoderCounter2 > 0) {
                if (speech_test_frequency2 <= 20000 - lv_spinbox_get_step(spinbox_test_frequency_2)) {
                  speech_test_frequency2 = speech_test_frequency2 + lv_spinbox_get_step(spinbox_test_frequency_2);
                  lv_spinbox_set_value(spinbox_test_frequency_2, speech_test_frequency2);
                  set_sine_frequency2(speech_test_frequency2);

                }
              }
              else {
                if (EncoderCounter2 < 0) {
                  if (speech_test_frequency2 >= 20 + lv_spinbox_get_step(spinbox_test_frequency_2)) {
                    speech_test_frequency2 = speech_test_frequency2 - lv_spinbox_get_step(spinbox_test_frequency_2);
                    lv_spinbox_set_value(spinbox_test_frequency_2, speech_test_frequency2);
                    set_sine_frequency2(speech_test_frequency2);

                  }
                }
              }
              if (speech_test_cursor_override) {
                lv_spinbox_show_cursor(spinbox_test_frequency_1, false);
                lv_spinbox_show_cursor(spinbox_test_frequency_2, true);
                speech_test_cursor_override = false;
              }
              break;
          }
          EncoderCounter2 = 0;
          break;

        case TAB_SETEOTLEVEL_REF:
          // eot mp3 level adjust
          if (EncoderCounter2 > 0) {
            if (speech_eot_mp3_level < lv_slider_get_max_value(slider_set_eot_mp3_level)) {
              speech_eot_mp3_level++;
              update_eot_mp3_level();
            }
          }
          else {
            if (EncoderCounter2 < 0) {
              if (speech_eot_mp3_level > 0) {
                speech_eot_mp3_level--;
                update_eot_mp3_level();
              }
            }
          }
          EncoderCounter2 = 0;
          break;

        default:
          // clear if other screens
          EncoderCounter2 = 0;
          break;
      }
    }

    // other screen related activity
    switch (lv_tabview_get_tab_active(tabview)) {
      case TAB_SCOPE_REF:
        // scope screen
        if (speech_transmitting and speech_screen_update_done and millis() - speech_scope_timer > 20 ) {
          scope_sample_plot();
          speech_scope_timer = millis();
        }
        break;

      case TAB_SPECTRUM_REF:
        // spectrum screen
        if (speech_transmitting and speech_screen_update_done and millis() - speech_scope_timer > 20 ) {
          spectrum_sample();
          spectrum_show();
          speech_scope_timer = millis();
        }
        break;

    }

  }

  // update screen, let lvgl do its thing
  if (millis() >= LVGL_Timer + 5) {
    lv_timer_handler();
    LVGL_Timer = millis();
    // direct draw activity to ensure screen is build before we draw
    // draw after screen is first drawn, scope
    if (!speech_screen_update_done and lv_tabview_get_tab_active(tabview) == TAB_SCOPE_REF) {
      scope_draw_screen(true);
    }
    // draw after screen is first drawn, spectrum
    if (!speech_screen_update_done and lv_tabview_get_tab_active(tabview) == TAB_SPECTRUM_REF) {
      spectrum_draw_screen(true);
      spectrum_update_cursor(speech_spectrum_cursor);
    }
    speech_screen_update_done = true;
  }
  else {
    delay(1);
  }
}

// splash screen
void Display_Splash_Screen(void) {
  lv_obj_set_style_bg_color (lv_scr_act(), lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );

  static lv_style_t label_style1;
  lv_style_set_text_color(&label_style1, lv_color_white());
  lv_style_set_text_font(&label_style1, &lv_font_montserrat_40);
  lv_style_set_text_font(&label_style1, &lv_font_montserrat_24);

  lv_obj_t *labelintro1 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro1, &label_style1, LV_PART_MAIN);
  lv_label_set_text(labelintro1, "DSP Speech Processor");
  lv_obj_align(labelintro1, LV_ALIGN_TOP_MID, 0, 10);

  static lv_style_t label_style2;
  lv_style_set_text_color(&label_style2, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_text_font(&label_style2, &lv_font_montserrat_18);

  lv_obj_t *labelintro2 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro2, &label_style2, LV_PART_MAIN);
  lv_label_set_text(labelintro2, "At boot time press:");
  lv_obj_align(labelintro2, LV_ALIGN_BOTTOM_MID, 0, -130);

  lv_obj_t *labelintro3 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro3, &label_style2, LV_PART_MAIN);
  lv_label_set_text(labelintro3, "Any encoder button to calibrate LCD");
  lv_obj_align(labelintro3, LV_ALIGN_BOTTOM_MID, 0, -100);

  static lv_style_t label_style3;
  lv_style_set_text_color(&label_style3, lv_palette_main(LV_PALETTE_ORANGE));
  lv_style_set_text_font(&label_style3, &lv_font_montserrat_24);
  lv_obj_t *labelintro10 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro10, &label_style3, LV_PART_MAIN);
  lv_label_set_text(labelintro10, "(C) Jef Collin 2025");
  lv_obj_align(labelintro10, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_refr_now(NULL);
}

void Setup_Screens(void) {
  // general use non adressable objects
  lv_obj_t *label;
  lv_obj_t *btn;

  int32_t obj_height;

  // tabs
  tabview = lv_tabview_create(lv_scr_act());
  lv_tabview_set_tab_bar_size(tabview, 0);

  tab_main = lv_tabview_add_tab(tabview, "");
  tab_menu = lv_tabview_add_tab(tabview, "");
  tab_compressor = lv_tabview_add_tab(tabview, "");
  tab_equalizer = lv_tabview_add_tab(tabview, "");
  tab_presets = lv_tabview_add_tab(tabview, "");
  tab_effects = lv_tabview_add_tab(tabview, "");
  tab_endoftransmission = lv_tabview_add_tab(tabview, "");
  tab_autokey = lv_tabview_add_tab(tabview, "");
  tab_voice = lv_tabview_add_tab(tabview, "");
  tab_scope = lv_tabview_add_tab(tabview, "");
  tab_recorder = lv_tabview_add_tab(tabview, "");
  tab_morse = lv_tabview_add_tab(tabview, "");
  tab_vox = lv_tabview_add_tab(tabview, "");
  tab_test = lv_tabview_add_tab(tabview, "");
  tab_settings = lv_tabview_add_tab(tabview, "");
  tab_edit = lv_tabview_add_tab(tabview, "");
  tab_spectrum = lv_tabview_add_tab(tabview, "");
  tab_filetransfer = lv_tabview_add_tab(tabview, "");
  tab_filedelete = lv_tabview_add_tab(tabview, "");
  tab_presetsmain = lv_tabview_add_tab(tabview, "");
  tab_seteotlevel = lv_tabview_add_tab(tabview, "");
  tab_setamplifier = lv_tabview_add_tab(tabview, "");
  tab_setvurange = lv_tabview_add_tab(tabview, "");
  tab_setmorselevel = lv_tabview_add_tab(tabview, "");
  tab_setautokeylevel = lv_tabview_add_tab(tabview, "");

  lv_obj_clear_flag(tabview, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_main, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_menu, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_compressor, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_equalizer, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_presets, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_effects, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_endoftransmission, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_autokey, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_voice, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_scope, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_recorder, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_morse, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_vox, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_test, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_settings, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_edit, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_spectrum, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_filetransfer, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_filedelete, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_presetsmain, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_seteotlevel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_setamplifier, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_setvurange, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_setmorselevel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_setautokeylevel, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_clear_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_style_bg_color (tab_main, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_menu, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_compressor, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_equalizer, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_presets, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_effects, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_endoftransmission, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_autokey, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_voice, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_scope, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_recorder, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_morse, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_vox, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_test, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_settings, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_edit, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_spectrum, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_filetransfer, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_filedelete, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_presetsmain, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_seteotlevel, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_setamplifier, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_setvurange, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_setmorselevel, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_setautokeylevel, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );

  lv_obj_set_style_bg_opa(tab_main, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_menu, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_compressor, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_equalizer, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_presets, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_effects, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_endoftransmission, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_autokey, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_voice, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_scope, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_recorder, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_morse, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_vox, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_test, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_settings, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_edit, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_spectrum, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_filetransfer, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_filedelete, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_presetsmain, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_seteotlevel, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_setamplifier, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_setvurange, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_setmorselevel, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_setautokeylevel, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);

  // remove 16 pix padding
  lv_obj_set_style_pad_top(tab_menu, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_menu, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_menu, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_menu, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_main, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_main, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_main, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_main, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_compressor, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_compressor, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_compressor, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_compressor, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_equalizer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_equalizer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_equalizer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_equalizer, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_presets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_presets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_presets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_presets, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_effects, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_effects, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_effects, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_effects, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_endoftransmission, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_endoftransmission, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_endoftransmission, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_endoftransmission, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_autokey, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_autokey, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_autokey, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_autokey, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_voice, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_voice, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_voice, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_voice, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_scope, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_scope, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_scope, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_scope, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_recorder, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_recorder, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_recorder, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_recorder, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_morse, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_morse, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_morse, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_morse, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_vox, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_vox, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_vox, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_vox, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_test, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_test, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_test, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_test, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_settings, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_settings, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_settings, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_settings, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_edit, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_edit, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_edit, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_edit, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_spectrum, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_spectrum, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_spectrum, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_spectrum, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_filetransfer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_filetransfer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_filetransfer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_filetransfer, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_filedelete, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_filedelete, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_filedelete, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_filedelete, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_presetsmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_presetsmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_presetsmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_presetsmain, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_seteotlevel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_seteotlevel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_seteotlevel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_seteotlevel, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_setamplifier, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_setamplifier, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_setamplifier, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_setamplifier, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_setvurange, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_setvurange, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_setvurange, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_setvurange, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_setmorselevel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_setmorselevel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_setmorselevel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_setmorselevel, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_setautokeylevel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_setautokeylevel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_setautokeylevel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_setautokeylevel, 0, LV_PART_MAIN);


  // object styles

  // label styles

  static lv_style_t label_style1;
  lv_style_set_text_color(&label_style1, lv_color_black());
  lv_style_set_text_font(&label_style1, &lv_font_montserrat_24);
  lv_style_set_text_align(&label_style1, LV_TEXT_ALIGN_LEFT);

  static lv_style_t label_style2;
  lv_style_set_text_color(&label_style2, lv_color_white());
  lv_style_set_text_font(&label_style2, &lv_font_montserrat_24);

  static lv_style_t label_style3;
  lv_style_set_text_color(&label_style3, lv_color_white());
  lv_style_set_text_font(&label_style3, &lv_font_montserrat_12);

  static lv_style_t label_style4;
  lv_style_set_text_color(&label_style4, lv_color_white());
  lv_style_set_text_font(&label_style4, &lv_font_montserrat_18);

  static lv_style_t label_style5;
  lv_style_set_text_color(&label_style5, lv_palette_main(LV_PALETTE_RED));
  lv_style_set_text_font(&label_style5, &lv_font_montserrat_24);

  static lv_style_t label_style6;
  lv_style_set_text_color(&label_style6, lv_color_white());
  lv_style_set_text_font(&label_style6, &lv_font_montserrat_24);
  lv_style_set_text_align(&label_style6, LV_TEXT_ALIGN_RIGHT);

  static lv_style_t label_style7;
  lv_style_set_text_color(&label_style7, lv_color_white());
  lv_style_set_text_font(&label_style7, &lv_font_montserrat_20);
  // button styles

  static lv_style_t btn_style1;
  lv_style_set_radius(&btn_style1, 3);
  lv_style_set_bg_color(&btn_style1, lv_color_white());
  lv_style_set_text_font(&btn_style1, &lv_font_montserrat_24);
  lv_style_set_text_color(&btn_style1, lv_color_black());
  lv_style_set_size(&btn_style1, 149, 50);

  static lv_style_t btn_style1_checked;
  lv_style_set_bg_color(&btn_style1_checked, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_text_font(&btn_style1_checked, &lv_font_montserrat_24);
  lv_style_set_text_color(&btn_style1_checked, lv_color_black());

  static lv_style_t btn_style2;
  lv_style_init(&btn_style2);
  lv_style_set_radius(&btn_style2, 0);
  lv_style_set_text_font(&btn_style2, &lv_font_montserrat_22);
  lv_style_set_bg_color(&btn_style2, lv_color_white());
  lv_style_set_size(&btn_style2, 480, 51);

  static lv_style_t btn_label_style2;
  lv_style_init(&btn_label_style2);
  lv_style_set_text_font(&btn_label_style2, &lv_font_montserrat_20);
  lv_style_set_text_color(&btn_label_style2, lv_color_black());
  lv_style_set_align(&btn_label_style2, LV_ALIGN_LEFT_MID);
  lv_style_set_pad_left(&btn_label_style2, 10);

  // radio buttons style

  static lv_style_t radiobtn_style1;
  lv_style_init(&radiobtn_style1);
  lv_style_set_radius(&radiobtn_style1, LV_RADIUS_CIRCLE);
  lv_style_set_bg_color(&radiobtn_style1, lv_color_black());
  lv_style_set_text_font(&radiobtn_style1, &lv_font_montserrat_18);
  lv_style_set_text_color(&radiobtn_style1, lv_color_white());
  lv_style_set_text_opa(&radiobtn_style1, LV_OPA_COVER);

  lv_style_set_border_color(&radiobtn_style1, lv_color_white());

  static lv_style_t radiobtn_checked_style1;
  lv_style_init(&radiobtn_checked_style1);
  lv_style_set_bg_image_src(&radiobtn_checked_style1, NULL);
  lv_style_set_bg_color(&radiobtn_checked_style1, lv_color_white());

  static lv_style_t radiobtn_style2;
  lv_style_init(&radiobtn_style2);
  //  lv_style_set_radius(&radiobtn_style2, LV_RADIUS_CIRCLE);
  lv_style_set_bg_color(&radiobtn_style2, lv_color_black());
  lv_style_set_text_font(&radiobtn_style2, &lv_font_montserrat_18);
  lv_style_set_text_color(&radiobtn_style2, lv_color_white());
  lv_style_set_text_opa(&radiobtn_style2, LV_OPA_COVER);

  lv_style_set_border_color(&radiobtn_style2, lv_color_white());

  static lv_style_t radiobtn_checked_style2;
  lv_style_init(&radiobtn_checked_style2);
  lv_style_set_bg_image_src(&radiobtn_checked_style2, NULL);
  lv_style_set_bg_color(&radiobtn_checked_style2, lv_color_white());


  // slider styles

  static const lv_style_prop_t props[] = {LV_STYLE_BG_COLOR, 0};
  static lv_style_transition_dsc_t transition_dsc;
  lv_style_transition_dsc_init(&transition_dsc, props, lv_anim_path_linear, 300, 0, NULL);

  static lv_style_t slider_style1_main;
  static lv_style_t slider_style1_indicator;
  static lv_style_t slider_style1_knob;
  static lv_style_t slider_style1_pressed_color;
  lv_style_init(&slider_style1_main);
  lv_style_set_bg_opa(&slider_style1_main, LV_OPA_COVER);
  lv_style_set_bg_color(&slider_style1_main, lv_color_hex3(0xbbb));
  lv_style_set_radius(&slider_style1_main, LV_RADIUS_CIRCLE);
  lv_style_set_pad_ver(&slider_style1_main, -2); /*Makes the indicator larger*/

  lv_style_init(&slider_style1_indicator);
  lv_style_set_bg_opa(&slider_style1_indicator, LV_OPA_COVER);
  lv_style_set_bg_color(&slider_style1_indicator, lv_palette_main(LV_PALETTE_LIGHT_BLUE));
  lv_style_set_radius(&slider_style1_indicator, LV_RADIUS_CIRCLE);
  lv_style_set_transition(&slider_style1_indicator, &transition_dsc);

  lv_style_init(&slider_style1_knob);
  lv_style_set_bg_opa(&slider_style1_knob, LV_OPA_COVER);
  lv_style_set_bg_color(&slider_style1_knob, lv_palette_main(LV_PALETTE_LIGHT_BLUE));

  lv_style_set_border_color(&slider_style1_knob, lv_palette_main(LV_PALETTE_LIGHT_BLUE));

  lv_style_set_border_width(&slider_style1_knob, 2);
  lv_style_set_radius(&slider_style1_knob, LV_RADIUS_CIRCLE);
  lv_style_set_pad_all(&slider_style1_knob, 6); /*Makes the knob larger*/
  lv_style_set_transition(&slider_style1_knob, &transition_dsc);

  lv_style_init(&slider_style1_pressed_color);
  //  lv_style_set_bg_color(&slider_style1_pressed_color, lv_palette_darken(LV_PALETTE_CYAN, 2));
  lv_style_set_bg_color(&slider_style1_pressed_color, lv_palette_main(LV_PALETTE_LIGHT_BLUE));

  // spinbox styles

  static lv_style_t spinbox_style1;

  lv_style_set_bg_color(&spinbox_style1, lv_color_white());
  lv_style_set_text_font(&spinbox_style1, &lv_font_montserrat_20);
  lv_style_set_text_color(&spinbox_style1, lv_color_black());

  // checkbox style

  static lv_style_t cb_style1;
  lv_style_init(&cb_style1);
  // lv_style_set_radius(&cb_style1, LV_RADIUS_CIRCLE);
  lv_style_set_bg_color(&cb_style1, lv_color_black());
  lv_style_set_text_font(&cb_style1, &lv_font_montserrat_24);
  lv_style_set_text_color(&cb_style1, lv_color_white());
  lv_style_set_text_opa(&cb_style1, LV_OPA_COVER);

  // main screen

  lv_obj_t *vuframe1 = lv_obj_create(tab_main);
  lv_obj_set_size(vuframe1, 29, 304);
  lv_obj_align(vuframe1, LV_ALIGN_TOP_LEFT, 0, 8);
  lv_obj_set_style_bg_opa(vuframe1, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_radius(vuframe1, 0, LV_PART_MAIN);
  lv_obj_clear_flag(vuframe1, LV_OBJ_FLAG_SCROLLABLE);

  slider_main_gain = lv_slider_create(tab_main);
  lv_obj_remove_style_all(slider_main_gain);
  lv_obj_add_style(slider_main_gain, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_main_gain, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_main_gain, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_main_gain, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_main_gain, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_main_gain, 15, 200);
  lv_slider_set_range(slider_main_gain, 0, 50);
  lv_obj_align(slider_main_gain, LV_ALIGN_TOP_LEFT, 90, 60);
  lv_obj_add_event_cb(slider_main_gain, event_gain_change, LV_EVENT_ALL, NULL);

  lbl_main_gain = lv_label_create(tab_main);
  lv_label_set_text(lbl_main_gain, "Gain");
  lv_obj_add_style(lbl_main_gain, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_main_gain, slider_main_gain, LV_ALIGN_OUT_TOP_MID, 0, -25);

  slider_main_output = lv_slider_create(tab_main);
  lv_obj_remove_style_all(slider_main_output);
  lv_obj_add_style(slider_main_output, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_main_output, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_main_output, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_main_output, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_main_output, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_main_output, 15, 200);
  lv_slider_set_range(slider_main_output, 0, 50);
  lv_obj_align(slider_main_output, LV_ALIGN_TOP_LEFT, 176, 60);
  lv_obj_add_event_cb(slider_main_output, event_output_change, LV_EVENT_ALL, NULL);

  lbl_main_output = lv_label_create(tab_main);
  lv_label_set_text(lbl_main_output, "Output");
  lv_obj_add_style(lbl_main_output, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_main_output, slider_main_output, LV_ALIGN_OUT_TOP_MID, 0, -25);

  slider_main_headphone = lv_slider_create(tab_main);
  lv_obj_remove_style_all(slider_main_headphone);
  lv_obj_add_style(slider_main_headphone, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_main_headphone, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_main_headphone, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_main_headphone, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_main_headphone, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_main_headphone, 15, 200);
  lv_slider_set_range(slider_main_headphone, 0, 50);
  lv_obj_align(slider_main_headphone, LV_ALIGN_TOP_LEFT, 262, 60);
  lv_obj_add_event_cb(slider_main_headphone, event_headphone_change, LV_EVENT_ALL, NULL);

  lbl_main_headphone = lv_label_create(tab_main);
  lv_label_set_text(lbl_main_headphone, "HP");
  lv_obj_add_style(lbl_main_headphone, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_main_headphone, slider_main_headphone, LV_ALIGN_OUT_TOP_MID, 0, -25);

  led_main_tx = lv_led_create(tab_main);
  lv_obj_align(led_main_tx, LV_ALIGN_TOP_LEFT, 331, 50);
  lv_led_set_brightness(led_main_tx, 150);
  lv_led_set_color(led_main_tx, lv_palette_main(LV_PALETTE_RED));
  lv_led_on(led_main_tx);

  lbl_main_tx = lv_label_create(tab_main);
  lv_label_set_text(lbl_main_tx, "TX");
  lv_obj_add_style(lbl_main_tx, &label_style5, LV_PART_MAIN);
  lv_obj_align_to(lbl_main_tx, led_main_tx, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

  cb_main_eot_on = lv_checkbox_create(tab_main);
  lv_checkbox_set_text(cb_main_eot_on, "EOT on");
  lv_obj_align(cb_main_eot_on, LV_ALIGN_TOP_LEFT, 331, 144);
  lv_obj_add_style(cb_main_eot_on, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_main_eot_on, event_eot_switch, LV_EVENT_CLICKED, NULL);

  btn_main_preset = lv_btn_create(tab_main);
  lv_obj_add_style(btn_main_preset, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_main_preset, event_presetsmainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_main_preset);
  lv_label_set_text(label, "Presets");
  lv_obj_center(label);
  lv_obj_align(btn_main_preset, LV_ALIGN_TOP_RIGHT, 0, 201);

  btn_main_menu = lv_btn_create(tab_main);
  lv_obj_add_style(btn_main_menu, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_main_menu, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_main_menu);
  lv_label_set_text(label, "Menu");
  lv_obj_center(label);
  lv_obj_align(btn_main_menu, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab menu

  lv_obj_t *btn50 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn50, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn50, event_compressorscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn50);
  lv_label_set_text(label, "Compressor");
  lv_obj_center(label);
  lv_obj_align(btn50, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *btn51 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn51, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn51, event_eqscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn51);
  lv_label_set_text(label, "Equalizer");
  lv_obj_center(label);
  lv_obj_align(btn51, LV_ALIGN_TOP_LEFT, 0, 67);

  lv_obj_t *btn52 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn52, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn52, event_presetsscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn52);
  lv_label_set_text(label, "Presets");
  lv_obj_center(label);
  lv_obj_align(btn52, LV_ALIGN_TOP_LEFT, 0, 134);

  lv_obj_t *btn53 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn53, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn53, event_effectsscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn53);
  lv_label_set_text(label, "Effects");
  lv_obj_center(label);
  lv_obj_align(btn53, LV_ALIGN_TOP_LEFT, 0, 201);

  lv_obj_t *btn54 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn54, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn54, event_eotscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn54);
  lv_label_set_text(label, "E.O.T.");
  lv_obj_center(label);
  lv_obj_align(btn54, LV_ALIGN_TOP_LEFT, 0, 268);

  lv_obj_t *btn55 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn55, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn55, event_scopescr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn55);
  lv_label_set_text(label, "Scope");
  lv_obj_center(label);
  lv_obj_align(btn55, LV_ALIGN_TOP_MID, 0, 0);

  lv_obj_t *btn56 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn56, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn56, event_spectrumscreen, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn56);
  lv_label_set_text(label, "Spectrum");
  lv_obj_center(label);
  lv_obj_align(btn56, LV_ALIGN_TOP_MID, 0, 67);

  lv_obj_t *btn57 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn57, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn57, event_recordscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn57);
  lv_label_set_text(label, "Recorder");
  lv_obj_center(label);
  lv_obj_align(btn57, LV_ALIGN_TOP_MID, 0, 134);

  lv_obj_t *btn58 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn58, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn58, event_voicescr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn58);
  lv_label_set_text(label, "Voice msg");
  lv_obj_center(label);
  lv_obj_align(btn58, LV_ALIGN_TOP_MID, 0, 201);

  lv_obj_t *btn59 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn59, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn59, event_autokeyscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn59);
  lv_label_set_text(label, "Autokey");
  lv_obj_center(label);
  lv_obj_align(btn59, LV_ALIGN_TOP_MID, 0, 268);

  lv_obj_t *btn60 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn60, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn60, event_morsescr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn60);
  lv_label_set_text(label, "Morse");
  lv_obj_center(label);
  lv_obj_align(btn60, LV_ALIGN_TOP_RIGHT, 0, 0);

  lv_obj_t *btn61 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn61, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn61, event_voxscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn61);
  lv_label_set_text(label, "VOX");
  lv_obj_center(label);
  lv_obj_align(btn61, LV_ALIGN_TOP_RIGHT, 0, 67);

  lv_obj_t *btn62 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn62, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn62, event_testscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn62);
  lv_label_set_text(label, "Test");
  lv_obj_center(label);
  lv_obj_align(btn62, LV_ALIGN_TOP_RIGHT, 0, 134);

  lv_obj_t *btn63 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn63, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn63, event_settingsscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn63);
  lv_label_set_text(label, "Settings");
  lv_obj_center(label);
  lv_obj_align(btn63, LV_ALIGN_TOP_RIGHT, 0, 201);

  lv_obj_t *btn64 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn64, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn64, event_mainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn64);
  lv_label_set_text(label, LV_SYMBOL_HOME);
  lv_obj_center(label);
  lv_obj_align(btn64, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab compressor

  lv_obj_t *vuframe2 = lv_obj_create(tab_compressor);
  lv_obj_set_size(vuframe2, 29, 304);
  lv_obj_align(vuframe2, LV_ALIGN_TOP_LEFT, 0, 8);
  lv_obj_set_style_bg_opa(vuframe2, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_radius(vuframe2, 0, LV_PART_MAIN);
  lv_obj_clear_flag(vuframe2, LV_OBJ_FLAG_SCROLLABLE);

  slider_compressor_threshold = lv_slider_create(tab_compressor);
  lv_obj_remove_style_all(slider_compressor_threshold);
  lv_obj_add_style(slider_compressor_threshold, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_compressor_threshold, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_compressor_threshold, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_compressor_threshold, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_compressor_threshold, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_compressor_threshold, 15, 185);
  lv_slider_set_range(slider_compressor_threshold, 0, 50);
  lv_obj_align(slider_compressor_threshold, LV_ALIGN_TOP_LEFT, 90, 55);
  lv_obj_add_event_cb(slider_compressor_threshold, event_compressor_threshold_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_compressor_threshold = lv_label_create(tab_compressor);
  lv_label_set_text(lbl_compressor_threshold, "Threshold");
  lv_obj_add_style(lbl_compressor_threshold, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_compressor_threshold, slider_compressor_threshold, LV_ALIGN_OUT_TOP_MID, 0, -25);

  slider_compressor_ratio = lv_slider_create(tab_compressor);
  lv_obj_remove_style_all(slider_compressor_ratio);
  lv_obj_add_style(slider_compressor_ratio, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_compressor_ratio, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_compressor_ratio, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_compressor_ratio, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_compressor_ratio, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_compressor_ratio, 15, 185);
  lv_slider_set_range(slider_compressor_ratio, 1, 10);
  lv_obj_align(slider_compressor_ratio, LV_ALIGN_TOP_LEFT, 190, 55);
  lv_obj_add_event_cb(slider_compressor_ratio, event_compressor_ratio_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_compressor_ratio = lv_label_create(tab_compressor);
  lv_label_set_text(lbl_compressor_ratio, "Ratio");
  lv_obj_add_style(lbl_compressor_ratio, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_compressor_ratio, slider_compressor_ratio, LV_ALIGN_OUT_TOP_MID, 0, -25);

  slider_compressor_postgain = lv_slider_create(tab_compressor);
  lv_obj_remove_style_all(slider_compressor_postgain);
  lv_obj_add_style(slider_compressor_postgain, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_compressor_postgain, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_compressor_postgain, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_compressor_postgain, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_compressor_postgain, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_compressor_postgain, 15, 185);
  lv_slider_set_range(slider_compressor_postgain, 0, 20);
  lv_obj_align(slider_compressor_postgain, LV_ALIGN_TOP_LEFT, 290, 55);
  lv_obj_add_event_cb(slider_compressor_postgain, event_compressor_postgain_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_compressor_postgain = lv_label_create(tab_compressor);
  lv_label_set_text(lbl_compressor_postgain, "PostGain");
  lv_obj_add_style(lbl_compressor_postgain, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_compressor_postgain, slider_compressor_postgain, LV_ALIGN_OUT_TOP_MID, 0, -25);

  slider_compressor_noisegate = lv_slider_create(tab_compressor);
  lv_obj_remove_style_all(slider_compressor_noisegate);
  lv_obj_add_style(slider_compressor_noisegate, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_compressor_noisegate, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_compressor_noisegate, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_compressor_noisegate, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_compressor_noisegate, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_compressor_noisegate, 15, 185);
  lv_slider_set_range(slider_compressor_noisegate, 0, 35);
  lv_obj_align(slider_compressor_noisegate, LV_ALIGN_TOP_LEFT, 390, 55);
  lv_obj_add_event_cb(slider_compressor_noisegate, event_compressor_noisegate_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_compressor_noisegate = lv_label_create(tab_compressor);
  lv_label_set_text(lbl_compressor_noisegate, "Gate");
  lv_obj_add_style(lbl_compressor_noisegate, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_compressor_noisegate, slider_compressor_noisegate, LV_ALIGN_OUT_TOP_MID, 0, -25);

  btn_compressor_reset = lv_btn_create(tab_compressor);
  lv_obj_add_style(btn_compressor_reset, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_compressor_reset, event_compressor_reset, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_compressor_reset);
  lv_label_set_text(label, "Reset");
  lv_obj_center(label);
  lv_obj_align(btn_compressor_reset, LV_ALIGN_TOP_MID, 0, 268);

  btn_compressor_back = lv_btn_create(tab_compressor);
  lv_obj_add_style(btn_compressor_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_compressor_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_compressor_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_compressor_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab equalizer

  slider_eq0 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq0);
  lv_obj_add_style(slider_eq0, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq0, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq0, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq0, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq0, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq0, 15, 200);
  lv_slider_set_range(slider_eq0, -15, +15);
  lv_obj_align(slider_eq0, LV_ALIGN_TOP_LEFT, 12, 30);
  lv_obj_add_event_cb(slider_eq0, event_eq0_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq0 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq0, "90");
  lv_obj_add_style(lbl_eq0, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq0, slider_eq0, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq1 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq1);
  lv_obj_add_style(slider_eq1, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq1, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq1, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq1, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq1, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq1, 15, 200);
  lv_slider_set_range(slider_eq1, -15, +15);
  lv_obj_align(slider_eq1, LV_ALIGN_TOP_LEFT, 52, 30);
  lv_obj_add_event_cb(slider_eq1, event_eq1_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq1 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq1, "150");
  lv_obj_add_style(lbl_eq1, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq1, slider_eq1, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq2 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq2);
  lv_obj_add_style(slider_eq2, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq2, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq2, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq2, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq2, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq2, 15, 200);
  lv_slider_set_range(slider_eq2, -15, +15);
  lv_obj_align(slider_eq2, LV_ALIGN_TOP_LEFT, 92, 30);
  lv_obj_add_event_cb(slider_eq2, event_eq2_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq2 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq2, "220");
  lv_obj_add_style(lbl_eq2, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq2, slider_eq2, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq3 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq3);
  lv_obj_add_style(slider_eq3, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq3, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq3, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq3, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq3, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq3, 15, 200);
  lv_slider_set_range(slider_eq3, -15, +15);
  lv_obj_align(slider_eq3, LV_ALIGN_TOP_LEFT, 132, 30);
  lv_obj_add_event_cb(slider_eq3, event_eq3_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq3 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq3, "300");
  lv_obj_add_style(lbl_eq3, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq3, slider_eq3, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq4 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq4);
  lv_obj_add_style(slider_eq4, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq4, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq4, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq4, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq4, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq4, 15, 200);
  lv_slider_set_range(slider_eq4, -15, +15);
  lv_obj_align(slider_eq4, LV_ALIGN_TOP_LEFT, 172, 30);
  lv_obj_add_event_cb(slider_eq4, event_eq4_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq4 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq4, "430");
  lv_obj_add_style(lbl_eq4, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq4, slider_eq4, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq5 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq5);
  lv_obj_add_style(slider_eq5, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq5, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq5, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq5, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq5, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq5, 15, 200);
  lv_slider_set_range(slider_eq5, -15, +15);
  lv_obj_align(slider_eq5, LV_ALIGN_TOP_LEFT, 212, 30);
  lv_obj_add_event_cb(slider_eq5, event_eq5_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq5 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq5, "600");
  lv_obj_add_style(lbl_eq5, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq5, slider_eq5, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq6 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq6);
  lv_obj_add_style(slider_eq6, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq6, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq6, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq6, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq6, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq6, 15, 200);
  lv_slider_set_range(slider_eq6, -15, +15);
  lv_obj_align(slider_eq6, LV_ALIGN_TOP_LEFT, 252, 30);
  lv_obj_add_event_cb(slider_eq6, event_eq6_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq6 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq6, "850");
  lv_obj_add_style(lbl_eq6, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq6, slider_eq6, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq7 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq7);
  lv_obj_add_style(slider_eq7, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq7, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq7, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq7, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq7, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq7, 15, 200);
  lv_slider_set_range(slider_eq7, -15, +15);
  lv_obj_align(slider_eq7, LV_ALIGN_TOP_LEFT, 292, 30);
  lv_obj_add_event_cb(slider_eq7, event_eq7_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq7 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq7, "1.2K");
  lv_obj_add_style(lbl_eq7, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq7, slider_eq7, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq8 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq8);
  lv_obj_add_style(slider_eq8, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq8, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq8, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq8, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq8, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq8, 15, 200);
  lv_slider_set_range(slider_eq8, -15, +15);
  lv_obj_align(slider_eq8, LV_ALIGN_TOP_LEFT, 332, 30);
  lv_obj_add_event_cb(slider_eq8, event_eq8_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq8 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq8, "1.7K");
  lv_obj_add_style(lbl_eq8, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq8, slider_eq8, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq9 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq9);
  lv_obj_add_style(slider_eq9, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq9, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq9, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq9, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq9, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq9, 15, 200);
  lv_slider_set_range(slider_eq9, -15, +15);
  lv_obj_align(slider_eq9, LV_ALIGN_TOP_LEFT, 372, 30);
  lv_obj_add_event_cb(slider_eq9, event_eq9_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq9 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq9, "2.4K");
  lv_obj_add_style(lbl_eq9, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq9, slider_eq9, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq10 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq10);
  lv_obj_add_style(slider_eq10, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq10, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq10, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq10, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq10, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq10, 15, 200);
  lv_slider_set_range(slider_eq10, -15, +15);
  lv_obj_align(slider_eq10, LV_ALIGN_TOP_LEFT, 412, 30);
  lv_obj_add_event_cb(slider_eq10, event_eq10_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq10 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq10, "3.3K");
  lv_obj_add_style(lbl_eq10, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq10, slider_eq10, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq11 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq11);
  lv_obj_add_style(slider_eq11, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq11, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq11, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq11, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq11, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq11, 15, 200);
  lv_slider_set_range(slider_eq11, -15, +15);
  lv_obj_align(slider_eq11, LV_ALIGN_TOP_LEFT, 452, 30);
  lv_obj_add_event_cb(slider_eq11, event_eq11_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq11 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq11, "4.7K");
  lv_obj_add_style(lbl_eq11, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq11, slider_eq11, LV_ALIGN_OUT_TOP_MID, 0, -15);

  lv_obj_t *btn150 = lv_btn_create(tab_equalizer);
  lv_obj_add_style(btn150, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn150, event_eq_reset, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn150);
  lv_label_set_text(label, "Reset");
  lv_obj_center(label);
  lv_obj_align(btn150, LV_ALIGN_TOP_MID, 0, 268);

  lv_obj_t *btn151 = lv_btn_create(tab_equalizer);
  lv_obj_add_style(btn151, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn151, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn151);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn151, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab presets

  cb_presets_selection0 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection0, event_presets_sel0, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection0, LV_ALIGN_TOP_LEFT, 5, 10);

  cb_presets_selection1 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection1, event_presets_sel1, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection1, LV_ALIGN_TOP_LEFT, 5, 60);

  cb_presets_selection2 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection2, event_presets_sel2, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection2, LV_ALIGN_TOP_LEFT, 5, 110);

  cb_presets_selection3 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection3, event_presets_sel3, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection3, LV_ALIGN_TOP_LEFT, 5, 160);

  cb_presets_selection4 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection4, event_presets_sel4, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection4, LV_ALIGN_TOP_LEFT, 5, 210);

  cb_presets_selection5 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection5, event_presets_sel5, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection5, LV_ALIGN_TOP_LEFT, 5, 260);

  // separate or it does not work
  lv_obj_add_style(cb_presets_selection0, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection0, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection1, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection1, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection2, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection2, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection3, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection3, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection4, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection4, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection5, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection5, &radiobtn_style1, LV_PART_INDICATOR);

  lv_obj_add_style(cb_presets_selection0, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection1, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection2, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection3, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection4, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection5, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);

  btn_presets_recall = lv_btn_create(tab_presets);
  lv_obj_add_style(btn_presets_recall, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_recall, event_presets_recall, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_recall);
  lv_label_set_text(label, "Recall");
  lv_obj_center(label);
  lv_obj_align(btn_presets_recall, LV_ALIGN_TOP_RIGHT, 0, 134);

  btn_presets_set = lv_btn_create(tab_presets);
  lv_obj_add_style(btn_presets_set, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_set, event_presets_set, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_set);
  lv_label_set_text(label, "Set");
  lv_obj_center(label);
  lv_obj_align(btn_presets_set, LV_ALIGN_TOP_RIGHT, 0, 201);

  btn_presets_back = lv_btn_create(tab_presets);
  lv_obj_add_style(btn_presets_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_presets_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab effects

  slider_effect_echo_time = lv_slider_create(tab_effects);
  lv_obj_remove_style_all(slider_effect_echo_time);
  lv_obj_add_style(slider_effect_echo_time, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_effect_echo_time, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_effect_echo_time, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_effect_echo_time, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_effect_echo_time, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_effect_echo_time, 15, 180);
  lv_slider_set_range(slider_effect_echo_time, 0, 100);
  lv_obj_align(slider_effect_echo_time, LV_ALIGN_TOP_LEFT, 35, 60);
  lv_obj_add_event_cb(slider_effect_echo_time, event_effects_echo_time_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_effect_slider1 = lv_label_create(tab_effects);
  lv_label_set_text(lbl_effect_slider1, "Echo\n time");
  lv_obj_add_style(lbl_effect_slider1, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_effect_slider1, slider_effect_echo_time, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_effect_echo_fb = lv_slider_create(tab_effects);
  lv_obj_remove_style_all(slider_effect_echo_fb);
  lv_obj_add_style(slider_effect_echo_fb, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_effect_echo_fb, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_effect_echo_fb, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_effect_echo_fb, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_effect_echo_fb, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_effect_echo_fb, 15, 180);
  lv_slider_set_range(slider_effect_echo_fb, 0, 100);
  lv_obj_align(slider_effect_echo_fb, LV_ALIGN_TOP_LEFT, 132, 60);
  lv_obj_add_event_cb(slider_effect_echo_fb, event_effects_echo_fb_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_effect_slider2 = lv_label_create(tab_effects);
  lv_label_set_text(lbl_effect_slider2, "Echo\n fb");
  lv_obj_add_style(lbl_effect_slider2, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_effect_slider2, slider_effect_echo_fb, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_effect_pitch = lv_slider_create(tab_effects);
  lv_obj_remove_style_all(slider_effect_pitch);
  lv_obj_add_style(slider_effect_pitch, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_effect_pitch, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_effect_pitch, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_effect_pitch, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_effect_pitch, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_effect_pitch, 15, 180);
  lv_slider_set_range(slider_effect_pitch, -20, +20);
  lv_obj_align(slider_effect_pitch, LV_ALIGN_TOP_LEFT, 229, 60);
  lv_obj_add_event_cb(slider_effect_pitch, event_effects_pitch_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_effect_slider3 = lv_label_create(tab_effects);
  lv_label_set_text(lbl_effect_slider3, "Pitch");
  lv_obj_add_style(lbl_effect_slider3, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_effect_slider3, slider_effect_pitch, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_effect_ring = lv_slider_create(tab_effects);
  lv_obj_remove_style_all(slider_effect_ring);
  lv_obj_add_style(slider_effect_ring, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_effect_ring, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_effect_ring, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_effect_ring, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_effect_ring, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_effect_ring, 15, 180);
  lv_slider_set_range(slider_effect_ring, 20, 150);
  lv_obj_align(slider_effect_ring, LV_ALIGN_TOP_LEFT, 326, 60);
  lv_obj_add_event_cb(slider_effect_ring, event_effects_ring_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_effect_slider4 = lv_label_create(tab_effects);
  lv_label_set_text(lbl_effect_slider4, "Ring");
  lv_obj_add_style(lbl_effect_slider4, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_effect_slider4, slider_effect_ring, LV_ALIGN_OUT_TOP_MID, 0, -15);

  cb_effect_selection1 = lv_checkbox_create(tab_effects);
  lv_checkbox_set_text(cb_effect_selection1, "Echo");
  lv_obj_add_event_cb(cb_effect_selection1, event_effects_echo, LV_EVENT_CLICKED, NULL);
  lv_obj_align(cb_effect_selection1, LV_ALIGN_TOP_LEFT, 30, 288);

  cb_effect_selection2 = lv_checkbox_create(tab_effects);
  lv_checkbox_set_text(cb_effect_selection2, "SB");
  lv_obj_add_event_cb(cb_effect_selection2, event_effects_sb, LV_EVENT_CLICKED, NULL);
  lv_obj_align(cb_effect_selection2, LV_ALIGN_TOP_LEFT, 127, 288);

  cb_effect_selection3 = lv_checkbox_create(tab_effects);
  lv_checkbox_set_text(cb_effect_selection3, "Pitch");
  lv_obj_add_event_cb(cb_effect_selection3, event_effects_pitch, LV_EVENT_CLICKED, NULL);
  lv_obj_align(cb_effect_selection3, LV_ALIGN_TOP_LEFT, 224, 288);

  cb_effect_selection4 = lv_checkbox_create(tab_effects);
  lv_checkbox_set_text(cb_effect_selection4, "Ring");
  lv_obj_add_event_cb(cb_effect_selection4, event_effects_ring, LV_EVENT_CLICKED, NULL);
  lv_obj_align(cb_effect_selection4, LV_ALIGN_TOP_LEFT, 321, 288);

  // separate or it does not work
  lv_obj_add_style(cb_effect_selection1, &radiobtn_style2, LV_PART_MAIN);
  lv_obj_add_style(cb_effect_selection1, &radiobtn_style2, LV_PART_INDICATOR);
  lv_obj_add_style(cb_effect_selection2, &radiobtn_style2, LV_PART_MAIN);
  lv_obj_add_style(cb_effect_selection2, &radiobtn_style2, LV_PART_INDICATOR);
  lv_obj_add_style(cb_effect_selection3, &radiobtn_style2, LV_PART_MAIN);
  lv_obj_add_style(cb_effect_selection3, &radiobtn_style2, LV_PART_INDICATOR);
  lv_obj_add_style(cb_effect_selection4, &radiobtn_style2, LV_PART_MAIN);
  lv_obj_add_style(cb_effect_selection4, &radiobtn_style2, LV_PART_INDICATOR);

  lv_obj_add_style(cb_effect_selection1, &radiobtn_checked_style2, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_effect_selection2, &radiobtn_checked_style2, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_effect_selection3, &radiobtn_checked_style2, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_effect_selection4, &radiobtn_checked_style2, LV_PART_INDICATOR | LV_STATE_CHECKED);

  btn_effect_back = lv_btn_create(tab_effects);
  lv_obj_add_style(btn_effect_back, &btn_style1, LV_PART_MAIN);
  lv_obj_set_width(btn_effect_back, 50);
  lv_obj_add_event_cb(btn_effect_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_effect_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_effect_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab end of transmission

  cb_eot_sel_beep = lv_checkbox_create(tab_endoftransmission);
  lv_checkbox_set_text(cb_eot_sel_beep, "Beep");
  lv_obj_align(cb_eot_sel_beep, LV_ALIGN_TOP_LEFT, 0, 15);
  lv_obj_add_style(cb_eot_sel_beep, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_eot_sel_beep, event_eot_select0, LV_EVENT_CLICKED, NULL);

  cb_eot_sel_sound = lv_checkbox_create(tab_endoftransmission);
  lv_checkbox_set_text(cb_eot_sel_sound, "Clip");
  lv_obj_align(cb_eot_sel_sound, LV_ALIGN_TOP_LEFT, 160, 15);
  lv_obj_add_style(cb_eot_sel_sound, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_eot_sel_sound, event_eot_select1, LV_EVENT_CLICKED, NULL);

  rol_eot_roller_beep = lv_roller_create(tab_endoftransmission);
  lv_roller_set_options(rol_eot_roller_beep,
                        "-\n"
                        "-\n"
                        "-",
                        LV_ROLLER_MODE_INFINITE);

  lv_obj_set_style_text_font(rol_eot_roller_beep, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_font(rol_eot_roller_beep, &lv_font_montserrat_24, LV_PART_SELECTED);
  lv_roller_set_visible_row_count(rol_eot_roller_beep, 5);
  lv_obj_set_width(rol_eot_roller_beep, 140);
  lv_obj_align(rol_eot_roller_beep, LV_ALIGN_LEFT_MID, 0, 20);

  lv_obj_add_event_cb(rol_eot_roller_beep, event_eot_beep_roller, LV_EVENT_ALL, NULL);

  rol_eot_roller_sound = lv_roller_create(tab_endoftransmission);
  lv_roller_set_options(rol_eot_roller_sound,
                        "-\n"
                        "-\n"
                        "-",
                        LV_ROLLER_MODE_INFINITE);

  lv_obj_set_style_text_font(rol_eot_roller_sound, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_font(rol_eot_roller_sound, &lv_font_montserrat_24, LV_PART_SELECTED);
  lv_roller_set_visible_row_count(rol_eot_roller_sound, 5);
  lv_obj_set_width(rol_eot_roller_sound, 140);
  lv_obj_align(rol_eot_roller_sound, LV_ALIGN_LEFT_MID, 160, 20);

  lv_obj_add_event_cb(rol_eot_roller_sound, event_eot_sound_roller, LV_EVENT_ALL, NULL);

  cb_eot_on = lv_checkbox_create(tab_endoftransmission);
  lv_checkbox_set_text(cb_eot_on, "EOT on");
  lv_obj_align(cb_eot_on, LV_ALIGN_TOP_LEFT, 331, 10);
  lv_obj_add_style(cb_eot_on, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_eot_on, event_eot_switch, LV_EVENT_CLICKED, NULL);

  cb_eot_tx_on = lv_checkbox_create(tab_endoftransmission);
  lv_checkbox_set_text(cb_eot_tx_on, "TX on");
  lv_obj_align(cb_eot_tx_on, LV_ALIGN_TOP_LEFT, 331, 77);
  lv_obj_add_style(cb_eot_tx_on, &cb_style1, LV_PART_MAIN);

  btn_eot_play = lv_btn_create(tab_endoftransmission);
  lv_obj_add_style(btn_eot_play, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_eot_play, event_eot_play, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_eot_play);
  lv_label_set_text(label, LV_SYMBOL_PLAY);
  lv_obj_center(label);
  lv_obj_align(btn_eot_play, LV_ALIGN_TOP_RIGHT, 0, 134);

  btn_eot_set_level = lv_btn_create(tab_endoftransmission);
  lv_obj_add_style(btn_eot_set_level, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_eot_set_level, event_eotsetlevelscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_eot_set_level);
  lv_label_set_text(label, LV_SYMBOL_SETTINGS);
  lv_obj_center(label);
  lv_obj_align(btn_eot_set_level, LV_ALIGN_TOP_RIGHT, 0, 201);


  btn_eot_back = lv_btn_create(tab_endoftransmission);
  lv_obj_add_style(btn_eot_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_eot_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_eot_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_eot_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab scope

  btn_scope_back = lv_btn_create(tab_scope);
  lv_obj_add_style(btn_scope_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_scope_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_scope_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_scope_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // spectrum

  lbl_spectrum_cursor = lv_label_create(tab_spectrum);
  lv_label_set_text(lbl_spectrum_cursor, "");
  lv_obj_add_style(lbl_spectrum_cursor, &label_style7, LV_PART_MAIN);
  lv_obj_align(lbl_spectrum_cursor, LV_ALIGN_TOP_LEFT, 5, 250);

  btn_spectrum_back = lv_btn_create(tab_spectrum);
  lv_obj_add_style(btn_spectrum_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_spectrum_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_spectrum_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_spectrum_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab recorder

  bar_recorder_progress = lv_bar_create(tab_recorder);
  lv_obj_set_size(bar_recorder_progress, 400, 24);
  lv_obj_align(bar_recorder_progress, LV_ALIGN_TOP_MID, 0, 20);
  lv_bar_set_value(bar_recorder_progress, 0, LV_ANIM_ON);

  add_ruler_to_bar(bar_recorder_progress, 400, 24, 0, 20, 5, 10, 2, 5);

  led_recorder_recording = lv_led_create(tab_recorder);
  lv_obj_align(led_recorder_recording, LV_ALIGN_TOP_LEFT, 4, 144);
  lv_led_set_brightness(led_recorder_recording, 150);
  lv_led_set_color(led_recorder_recording, lv_palette_main(LV_PALETTE_RED));
  lv_led_on(led_recorder_recording);

  lbl_recorder_recording = lv_label_create(tab_recorder);
  lv_label_set_text(lbl_recorder_recording, "Recording");
  lv_obj_add_style(lbl_recorder_recording, &label_style5, LV_PART_MAIN);

  lv_obj_align_to(lbl_recorder_recording, led_recorder_recording, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

  btn_recorder_play = lv_btn_create(tab_recorder);
  lv_obj_add_style(btn_recorder_play, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_recorder_play, event_playback, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_recorder_play);
  lv_label_set_text(label, LV_SYMBOL_PLAY);
  lv_obj_center(label);
  lv_obj_align(btn_recorder_play, LV_ALIGN_TOP_MID, 0, 134);

  btn_recorder_erase = lv_btn_create(tab_recorder);
  lv_obj_add_style(btn_recorder_erase, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_recorder_erase, event_eraserecording, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_recorder_erase);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_align(btn_recorder_erase, LV_ALIGN_TOP_MID, 0, 201);

  btn_recorder_cancel = lv_btn_create(tab_recorder);
  lv_obj_add_style(btn_recorder_cancel, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_recorder_cancel, event_cancelplayback, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_recorder_cancel);
  lv_label_set_text(label, LV_SYMBOL_STOP);
  lv_obj_center(label);
  lv_obj_align(btn_recorder_cancel, LV_ALIGN_TOP_MID, 0, 268);
  lv_obj_add_state(btn_recorder_cancel, LV_STATE_DISABLED);

  cb_recorder_tx_on = lv_checkbox_create(tab_recorder);
  lv_checkbox_set_text(cb_recorder_tx_on, "TX on");
  lv_obj_align(cb_recorder_tx_on, LV_ALIGN_TOP_LEFT, 331, 144);
  lv_obj_add_style(cb_recorder_tx_on, &cb_style1, LV_PART_MAIN);

  cb_recorder_bypass_on = lv_checkbox_create(tab_recorder);
  lv_checkbox_set_text(cb_recorder_bypass_on, "Bypass");
  lv_obj_align(cb_recorder_bypass_on, LV_ALIGN_TOP_LEFT, 331, 211);
  lv_obj_add_style(cb_recorder_bypass_on, &cb_style1, LV_PART_MAIN);

  btn_recorder_back = lv_btn_create(tab_recorder);
  lv_obj_add_style(btn_recorder_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_recorder_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_recorder_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_recorder_back, LV_ALIGN_TOP_RIGHT, 0, 268);


  // tab msg

  bar_voice_progress = lv_bar_create(tab_voice);
  lv_obj_set_size(bar_voice_progress, 400, 24);
  lv_obj_align(bar_voice_progress, LV_ALIGN_TOP_MID, 0, 20);
  lv_bar_set_value(bar_voice_progress, 0, LV_ANIM_ON);

  add_ruler_to_bar(bar_voice_progress, 400, 24, 0, 20, 5, 10, 2, 5);

  cb_voice_selection1 = lv_checkbox_create(tab_voice);
  lv_obj_add_event_cb(cb_voice_selection1, event_voice_sel1, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_voice_selection1, LV_ALIGN_TOP_LEFT, 5, 115);

  cb_voice_selection2 = lv_checkbox_create(tab_voice);
  lv_obj_add_event_cb(cb_voice_selection2, event_voice_sel2, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_voice_selection2, LV_ALIGN_TOP_LEFT, 5, 155);

  cb_voice_selection3 = lv_checkbox_create(tab_voice);
  lv_obj_add_event_cb(cb_voice_selection3, event_voice_sel3, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_voice_selection3, LV_ALIGN_TOP_LEFT, 5, 195);

  cb_voice_selection4 = lv_checkbox_create(tab_voice);
  lv_obj_add_event_cb(cb_voice_selection4, event_voice_sel4, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_voice_selection4, LV_ALIGN_TOP_LEFT, 5, 235);

  cb_voice_selection5 = lv_checkbox_create(tab_voice);
  lv_obj_add_event_cb(cb_voice_selection5, event_voice_sel5, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_voice_selection5, LV_ALIGN_TOP_LEFT, 5, 275);

  // separate or it does not work
  lv_obj_add_style(cb_voice_selection1, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_voice_selection1, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_voice_selection2, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_voice_selection2, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_voice_selection3, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_voice_selection3, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_voice_selection4, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_voice_selection4, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_voice_selection5, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_voice_selection5, &radiobtn_style1, LV_PART_INDICATOR);

  lv_obj_add_style(cb_voice_selection1, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_voice_selection2, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_voice_selection3, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_voice_selection4, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_voice_selection5, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);

  led_voice_recording = lv_led_create(tab_voice);
  lv_obj_align(led_voice_recording, LV_ALIGN_TOP_MID, -65, 144);
  lv_led_set_brightness(led_voice_recording, 150);
  lv_led_set_color(led_voice_recording, lv_palette_main(LV_PALETTE_RED));
  lv_led_on(led_voice_recording);

  lbl_voice_recording = lv_label_create(tab_voice);
  lv_label_set_text(lbl_voice_recording, "Recording");
  lv_obj_add_style(lbl_voice_recording, &label_style5, LV_PART_MAIN);

  lv_obj_align_to(lbl_voice_recording, led_voice_recording, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

  btn_voice_play = lv_btn_create(tab_voice);
  lv_obj_add_style(btn_voice_play, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_voice_play, event_playback, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_voice_play);
  lv_label_set_text(label, LV_SYMBOL_PLAY);
  lv_obj_center(label);
  lv_obj_align(btn_voice_play, LV_ALIGN_TOP_MID, 0, 201);

  btn_voice_cancel = lv_btn_create(tab_voice);
  lv_obj_add_style(btn_voice_cancel, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_voice_cancel, event_cancelplayback, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_voice_cancel);
  lv_label_set_text(label, LV_SYMBOL_STOP);
  lv_obj_center(label);
  lv_obj_align(btn_voice_cancel, LV_ALIGN_TOP_MID, 0, 268);
  lv_obj_add_state(btn_voice_cancel, LV_STATE_DISABLED);

  cb_voice_tx_on = lv_checkbox_create(tab_voice);
  lv_checkbox_set_text(cb_voice_tx_on, "TX on");
  lv_obj_align(cb_voice_tx_on, LV_ALIGN_TOP_LEFT, 331, 211);
  lv_obj_add_style(cb_voice_tx_on, &cb_style1, LV_PART_MAIN);

  btn_voice_back = lv_btn_create(tab_voice);
  lv_obj_add_style(btn_voice_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_voice_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_voice_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_voice_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab autokey

  led_autokey_tx = lv_led_create(tab_autokey);
  lv_obj_align(led_autokey_tx, LV_ALIGN_TOP_LEFT, 5, 130);
  lv_led_set_brightness(led_autokey_tx, 150);
  lv_led_set_color(led_autokey_tx, lv_palette_main(LV_PALETTE_RED));
  lv_led_on(led_autokey_tx);

  lbl_autokey_tx = lv_label_create(tab_autokey);
  lv_label_set_text(lbl_autokey_tx, "TX");
  lv_obj_add_style(lbl_autokey_tx, &label_style5, LV_PART_MAIN);
  lv_obj_align_to(lbl_autokey_tx, led_autokey_tx, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

  rol_autokey_roller_sound = lv_roller_create(tab_autokey);
  lv_roller_set_options(rol_autokey_roller_sound,
                        "-loading-\n"
                        "-loading-\n"
                        "-loading-",
                        LV_ROLLER_MODE_INFINITE);

  lv_obj_set_style_text_font(rol_autokey_roller_sound, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_font(rol_autokey_roller_sound, &lv_font_montserrat_24, LV_PART_SELECTED);
  lv_obj_set_width(rol_autokey_roller_sound, 280);
  lv_roller_set_visible_row_count(rol_autokey_roller_sound, 3);

  lv_obj_align(rol_autokey_roller_sound, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  spinbox_autokey_txtime = lv_spinbox_create(tab_autokey);
  lv_spinbox_set_range(spinbox_autokey_txtime, 2, 480);
  lv_spinbox_set_digit_format(spinbox_autokey_txtime, 3, 0);
  lv_spinbox_set_step(spinbox_autokey_txtime, 1);
  lv_obj_set_width(spinbox_autokey_txtime, 65);
  lv_obj_align(spinbox_autokey_txtime, LV_ALIGN_TOP_LEFT, 171, 5);
  lv_obj_add_style(spinbox_autokey_txtime, &spinbox_style1, LV_PART_MAIN);

  obj_height = lv_obj_get_height(spinbox_autokey_txtime);

  btn_autokey_txtime_inc = lv_button_create(tab_autokey);
  lv_obj_set_size(btn_autokey_txtime_inc, obj_height, obj_height);
  lv_obj_align_to(btn_autokey_txtime_inc, spinbox_autokey_txtime, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_autokey_txtime_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_autokey_txtime_inc, increment_autokey_txtime_event_cb, LV_EVENT_ALL,  NULL);

  btn_autokey_txtime_dec = lv_button_create(tab_autokey);
  lv_obj_set_size(btn_autokey_txtime_dec, obj_height, obj_height);
  lv_obj_align_to(btn_autokey_txtime_dec, spinbox_autokey_txtime, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_autokey_txtime_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_autokey_txtime_dec, decrement_autokey_txtime_event_cb, LV_EVENT_ALL, NULL);

  label = lv_label_create(tab_autokey);
  lv_label_set_text(label, "Duration");
  lv_obj_add_style(label, &label_style2, LV_PART_MAIN);

  lv_obj_align_to(label, btn_autokey_txtime_dec, LV_ALIGN_OUT_LEFT_MID, -15, 0);

  spinbox_autokey_interval = lv_spinbox_create(tab_autokey);
  lv_spinbox_set_range(spinbox_autokey_interval, 1, 30);
  lv_spinbox_set_digit_format(spinbox_autokey_interval, 2, 0);
  lv_spinbox_set_step(spinbox_autokey_interval, 1);
  lv_obj_set_width(spinbox_autokey_interval, 65);
  lv_obj_align(spinbox_autokey_interval, LV_ALIGN_TOP_LEFT, 171, 72);
  lv_obj_add_style(spinbox_autokey_interval, &spinbox_style1, LV_PART_MAIN);

  obj_height = lv_obj_get_height(spinbox_autokey_interval);

  btn_autokey_interval_inc = lv_button_create(tab_autokey);
  lv_obj_set_size(btn_autokey_interval_inc, obj_height, obj_height);
  lv_obj_align_to(btn_autokey_interval_inc, spinbox_autokey_interval, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_autokey_interval_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_autokey_interval_inc, increment_autokey_interval_event_cb, LV_EVENT_ALL,  NULL);

  btn_autokey_interval_dec = lv_button_create(tab_autokey);
  lv_obj_set_size(btn_autokey_interval_dec, obj_height, obj_height);
  lv_obj_align_to(btn_autokey_interval_dec, spinbox_autokey_interval, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_autokey_interval_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_autokey_interval_dec, decrement_autokey_interval_event_cb, LV_EVENT_ALL, NULL);

  label = lv_label_create(tab_autokey);
  lv_label_set_text(label, "Interval");
  lv_obj_add_style(label, &label_style2, LV_PART_MAIN);

  lv_obj_align_to(label, btn_autokey_interval_dec, LV_ALIGN_OUT_LEFT_MID, -15, 0);

  lbl_autokey_txtime = lv_label_create(tab_autokey);
  lv_obj_set_width(lbl_autokey_txtime, 160);
  lv_label_set_text(lbl_autokey_txtime, "0");
  lv_obj_add_style(lbl_autokey_txtime, &label_style6, LV_PART_MAIN);
  lv_obj_align_to(lbl_autokey_txtime, btn_autokey_txtime_inc, LV_ALIGN_OUT_RIGHT_MID, 30, 0);

  lbl_autokey_interval = lv_label_create(tab_autokey);
  lv_obj_set_width(lbl_autokey_interval, 160);
  lv_label_set_text(lbl_autokey_interval, "0");
  lv_obj_add_style(lbl_autokey_interval, &label_style6, LV_PART_MAIN);
  lv_obj_align_to(lbl_autokey_interval, btn_autokey_interval_inc, LV_ALIGN_OUT_RIGHT_MID, 30, 0);

  btn_autokey_start_stop = lv_btn_create(tab_autokey);
  lv_obj_add_style(btn_autokey_start_stop, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_autokey_start_stop, event_autokey_start_stop, LV_EVENT_CLICKED, NULL);
  lbl_autokey_start_stop = lv_label_create(btn_autokey_start_stop);
  lv_label_set_text(lbl_autokey_start_stop, LV_SYMBOL_PLAY);
  lv_obj_center(lbl_autokey_start_stop);
  lv_obj_align(btn_autokey_start_stop, LV_ALIGN_TOP_RIGHT, 0, 134);

  btn_autokey_set_level = lv_btn_create(tab_autokey);
  lv_obj_add_style(btn_autokey_set_level, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_autokey_set_level, event_autokeysetlevelscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_autokey_set_level);
  lv_label_set_text(label, LV_SYMBOL_SETTINGS);
  lv_obj_center(label);
  lv_obj_align(btn_autokey_set_level, LV_ALIGN_TOP_RIGHT, 0, 201);

  btn_autokey_back = lv_btn_create(tab_autokey);
  lv_obj_add_style(btn_autokey_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_autokey_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_autokey_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_autokey_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab morse

  cb_morse_selection1 = lv_checkbox_create(tab_morse);
  lv_obj_add_event_cb(cb_morse_selection1, event_morse_sel1, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_morse_selection1, LV_ALIGN_TOP_LEFT, 5, 10);

  cb_morse_selection2 = lv_checkbox_create(tab_morse);
  lv_obj_add_event_cb(cb_morse_selection2, event_morse_sel2, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_morse_selection2, LV_ALIGN_TOP_LEFT, 5, 60);

  cb_morse_selection3 = lv_checkbox_create(tab_morse);
  lv_obj_add_event_cb(cb_morse_selection3, event_morse_sel3, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_morse_selection3, LV_ALIGN_TOP_LEFT, 5, 110);

  cb_morse_selection4 = lv_checkbox_create(tab_morse);
  lv_obj_add_event_cb(cb_morse_selection4, event_morse_sel4, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_morse_selection4, LV_ALIGN_TOP_LEFT, 5, 160);

  cb_morse_selection5 = lv_checkbox_create(tab_morse);
  lv_obj_add_event_cb(cb_morse_selection5, event_morse_sel5, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_morse_selection5, LV_ALIGN_TOP_LEFT, 5, 210);

  // separate or it does not work
  lv_obj_add_style(cb_morse_selection1, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_morse_selection1, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_morse_selection2, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_morse_selection2, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_morse_selection3, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_morse_selection3, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_morse_selection4, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_morse_selection4, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_morse_selection5, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_morse_selection5, &radiobtn_style1, LV_PART_INDICATOR);

  lv_obj_add_style(cb_morse_selection1, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_morse_selection2, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_morse_selection3, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_morse_selection4, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_morse_selection5, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);

  cb_morse_tx_on = lv_checkbox_create(tab_morse);
  lv_checkbox_set_text(cb_morse_tx_on, "TX on");
  lv_obj_align(cb_morse_tx_on, LV_ALIGN_TOP_LEFT, 331, 10);
  lv_obj_add_style(cb_morse_tx_on, &cb_style1, LV_PART_MAIN);

  btn_morse_play = lv_btn_create(tab_morse);
  lv_obj_add_style(btn_morse_play, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_morse_play, event_morseplay, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_morse_play);
  lv_label_set_text(label, LV_SYMBOL_PLAY);
  lv_obj_center(label);
  lv_obj_align(btn_morse_play, LV_ALIGN_TOP_RIGHT, 0, 134);

  btn_morse_set_level = lv_btn_create(tab_morse);
  lv_obj_add_style(btn_morse_set_level, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_morse_set_level, event_morsesetlevelscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_morse_set_level);
  lv_label_set_text(label, LV_SYMBOL_SETTINGS);
  lv_obj_center(label);
  lv_obj_align(btn_morse_set_level, LV_ALIGN_TOP_RIGHT, 0, 201);

  btn_morse_back = lv_btn_create(tab_morse);
  lv_obj_add_style(btn_morse_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_morse_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_morse_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_morse_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab vox

  slider_vox_threshold = lv_slider_create(tab_vox);

  lv_obj_remove_style_all(slider_vox_threshold);

  lv_obj_add_style(slider_vox_threshold, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_vox_threshold, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_vox_threshold, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_vox_threshold, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_vox_threshold, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_vox_threshold, 15, 200);
  lv_slider_set_range(slider_vox_threshold, 0, 50);
  lv_obj_align(slider_vox_threshold, LV_ALIGN_TOP_LEFT, 55, 60);

  label = lv_label_create(tab_vox);
  lv_label_set_text(label, "Threshold");
  lv_obj_add_style(label, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(label, slider_vox_threshold, LV_ALIGN_OUT_TOP_MID, 0, -25);

  lv_obj_add_event_cb(slider_vox_threshold, event_vox_threshold_change, LV_EVENT_VALUE_CHANGED, NULL);

  slider_vox_hangtime = lv_slider_create(tab_vox);

  lv_obj_remove_style_all(slider_vox_hangtime);

  lv_obj_add_style(slider_vox_hangtime, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_vox_hangtime, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_vox_hangtime, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_vox_hangtime, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_vox_hangtime, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_vox_hangtime, 15, 200);
  lv_slider_set_range(slider_vox_hangtime, 0, 50);

  lv_obj_align(slider_vox_hangtime, LV_ALIGN_TOP_LEFT, 165, 60);

  label = lv_label_create(tab_vox);
  lv_label_set_text(label, "Hang Time");
  lv_obj_add_style(label, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(label, slider_vox_hangtime, LV_ALIGN_OUT_TOP_MID, 0, -25);

  lv_obj_add_event_cb(slider_vox_hangtime, event_vox_hangtime_change, LV_EVENT_VALUE_CHANGED, NULL);

  led_vox_tx = lv_led_create(tab_vox);
  lv_obj_align(led_vox_tx, LV_ALIGN_TOP_LEFT, 331, 50);
  lv_led_set_brightness(led_vox_tx, 150);
  lv_led_set_color(led_vox_tx, lv_palette_main(LV_PALETTE_RED));
  lv_led_on(led_vox_tx);

  lbl_vox_tx = lv_label_create(tab_vox);
  lv_label_set_text(lbl_vox_tx, "TX");
  lv_obj_add_style(lbl_vox_tx, &label_style5, LV_PART_MAIN);
  lv_obj_align_to(lbl_vox_tx, led_vox_tx, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

  cb_vox_on = lv_checkbox_create(tab_vox);
  lv_checkbox_set_text(cb_vox_on, "VOX on");
  lv_obj_align(cb_vox_on, LV_ALIGN_TOP_LEFT, 331, 144);
  lv_obj_add_style(cb_vox_on, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_vox_on, event_vox_switch, LV_EVENT_CLICKED, NULL);

  btn_vox_back = lv_btn_create(tab_vox);
  lv_obj_add_style(btn_vox_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_vox_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_vox_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_vox_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab test

  slider_test_signal_gain = lv_slider_create(tab_test);

  lv_obj_remove_style_all(slider_test_signal_gain);

  lv_obj_add_style(slider_test_signal_gain, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_test_signal_gain, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_test_signal_gain, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_test_signal_gain, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_test_signal_gain, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_test_signal_gain, 15, 200);

  // calibrate range
  lv_slider_set_range(slider_test_signal_gain, 0, 37);

  lv_obj_align(slider_test_signal_gain, LV_ALIGN_TOP_LEFT, 25, 60);

  lv_obj_add_event_cb(slider_test_signal_gain, event_test_signal_gain_change, LV_EVENT_ALL, NULL);

  lv_obj_t *slider_label = lv_label_create(tab_test);
  lv_label_set_text(slider_label, "Gain");
  lv_obj_add_style(slider_label, &label_style4, LV_PART_MAIN);

  lv_obj_align_to(slider_label, slider_test_signal_gain, LV_ALIGN_OUT_TOP_MID, 0, -25);

  spinbox_test_frequency_1 = lv_spinbox_create(tab_test);
  lv_spinbox_set_range(spinbox_test_frequency_1, 20, 20000);
  lv_spinbox_set_digit_format(spinbox_test_frequency_1, 5, 0);
  //lv_spinbox_step_prev(spinbox_test_frequency_1);
  lv_spinbox_set_step(spinbox_test_frequency_1, 10);
  lv_obj_set_width(spinbox_test_frequency_1, 80);
  //  lv_obj_center(spinbox);
  lv_obj_align(spinbox_test_frequency_1, LV_ALIGN_TOP_LEFT, 180, 190);
  lv_obj_add_style(spinbox_test_frequency_1, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_test_frequency_1, frequency1_change_event_cb, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_test_frequency_1);

  btn_test_frequency_1_inc = lv_button_create(tab_test);
  lv_obj_set_size(btn_test_frequency_1_inc, obj_height, obj_height);
  lv_obj_align_to(btn_test_frequency_1_inc, spinbox_test_frequency_1, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_test_frequency_1_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_test_frequency_1_inc, increment_test_frequency1_event_cb, LV_EVENT_ALL,  NULL);

  btn_test_frequency_1_dec = lv_button_create(tab_test);
  lv_obj_set_size(btn_test_frequency_1_dec, obj_height, obj_height);
  lv_obj_align_to(btn_test_frequency_1_dec, spinbox_test_frequency_1, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_test_frequency_1_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_test_frequency_1_dec, decrement_test_frequency1_event_cb, LV_EVENT_ALL, NULL);

  spinbox_test_frequency_2 = lv_spinbox_create(tab_test);
  lv_spinbox_set_range(spinbox_test_frequency_2, 20, 20000);
  lv_spinbox_set_digit_format(spinbox_test_frequency_2, 5, 0);
  //lv_spinbox_step_prev(spinbox_test_frequency_2);
  lv_spinbox_set_step(spinbox_test_frequency_2, 10);
  lv_obj_set_width(spinbox_test_frequency_2, 80);
  //  lv_obj_center(spinbox);
  lv_obj_align(spinbox_test_frequency_2, LV_ALIGN_TOP_LEFT, 180, 260);
  lv_obj_add_style(spinbox_test_frequency_2, &spinbox_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(spinbox_test_frequency_2, frequency2_change_event_cb, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_test_frequency_2);

  btn_test_frequency_2_inc = lv_button_create(tab_test);
  lv_obj_set_size(btn_test_frequency_2_inc, obj_height, obj_height);
  lv_obj_align_to(btn_test_frequency_2_inc, spinbox_test_frequency_2, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_test_frequency_2_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_test_frequency_2_inc, increment_test_frequency2_event_cb, LV_EVENT_ALL,  NULL);

  btn_test_frequency_2_dec = lv_button_create(tab_test);
  lv_obj_set_size(btn_test_frequency_2_dec, obj_height, obj_height);
  lv_obj_align_to(btn_test_frequency_2_dec, spinbox_test_frequency_2, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_test_frequency_2_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_test_frequency_2_dec, decrement_test_frequency2_event_cb, LV_EVENT_ALL, NULL);

  cb_test_source1 = lv_checkbox_create(tab_test);
  lv_checkbox_set_text(cb_test_source1, "None");
  lv_obj_add_event_cb(cb_test_source1, event_test_mode0, LV_EVENT_CLICKED, NULL);
  lv_obj_align(cb_test_source1, LV_ALIGN_TOP_LEFT, 87, 5);

  cb_test_source2 = lv_checkbox_create(tab_test);
  lv_checkbox_set_text(cb_test_source2, "Single frequency sine");
  lv_obj_add_event_cb(cb_test_source2, event_test_mode1, LV_EVENT_CLICKED, NULL);
  lv_obj_align(cb_test_source2, LV_ALIGN_TOP_LEFT, 87, 50);

  cb_test_source3 = lv_checkbox_create(tab_test);
  lv_checkbox_set_text(cb_test_source3, "Dual frequency sine");
  lv_obj_add_event_cb(cb_test_source3, event_test_mode2, LV_EVENT_CLICKED, NULL);
  lv_obj_align(cb_test_source3, LV_ALIGN_TOP_LEFT, 87, 95);

  // separate or it does not work
  lv_obj_add_style(cb_test_source1, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_test_source1, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_test_source2, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_test_source2, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_test_source3, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_test_source3, &radiobtn_style1, LV_PART_INDICATOR);

  lv_obj_add_style(cb_test_source1, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_test_source2, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_test_source3, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);

  label = lv_label_create(tab_test);

  lv_label_set_text(label, LV_SYMBOL_AUDIO " 1");

  lv_obj_add_style(label, &label_style2, LV_PART_MAIN);

  lv_obj_align_to(label, spinbox_test_frequency_1, LV_ALIGN_OUT_LEFT_MID, -54, 0);

  label = lv_label_create(tab_test);

  lv_label_set_text(label, LV_SYMBOL_AUDIO " 2");

  lv_obj_add_style(label, &label_style2, LV_PART_MAIN);

  lv_obj_align_to(label, spinbox_test_frequency_2, LV_ALIGN_OUT_LEFT_MID, -54, 0);

  btn_test_tx = lv_btn_create(tab_test);
  lv_obj_add_style(btn_test_tx, &btn_style1, LV_PART_MAIN);
  lv_obj_add_style(btn_test_tx, &btn_style1_checked, LV_STATE_CHECKED);

  lv_obj_add_event_cb(btn_test_tx, event_test_transmit, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_test_tx);
  lv_label_set_text(label, "Transmit");
  lv_obj_center(label);
  lv_obj_align(btn_test_tx, LV_ALIGN_TOP_RIGHT, 0, 201);

  lv_obj_add_flag(btn_test_tx, LV_OBJ_FLAG_CHECKABLE);

  lv_obj_t *btn650 = lv_btn_create(tab_test);
  lv_obj_add_style(btn650, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn650, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn650);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn650, LV_ALIGN_TOP_RIGHT, 0, 268);


  // tab settings
  lv_obj_t *btn701 = lv_btn_create(tab_settings);
  lv_obj_add_style(btn701, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn701, event_filetransferscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn701);
  lv_label_set_text(label, "File " LV_SYMBOL_DOWNLOAD);
  lv_obj_center(label);
  lv_obj_align(btn701, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *btn702 = lv_btn_create(tab_settings);
  lv_obj_add_style(btn702, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn702, event_filedeletescreen, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn702);
  lv_label_set_text(label, "File " LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_align(btn702, LV_ALIGN_TOP_LEFT, 0, 67);

  lv_obj_t *btn703 = lv_btn_create(tab_settings);
  lv_obj_add_style(btn703, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn703, event_ampsetscreen, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn703);
  lv_label_set_text(label, "Amp " LV_SYMBOL_SETTINGS);
  lv_obj_center(label);
  lv_obj_align(btn703, LV_ALIGN_TOP_LEFT, 0, 134);

  lv_obj_t *btn704 = lv_btn_create(tab_settings);
  lv_obj_add_style(btn704, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn704, event_vusetscreen, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn704);
  lv_label_set_text(label, "VU " LV_SYMBOL_SETTINGS);
  lv_obj_center(label);
  lv_obj_align(btn704, LV_ALIGN_TOP_LEFT, 0, 201);

  cb_bypass_on = lv_checkbox_create(tab_settings);
  lv_checkbox_set_text(cb_bypass_on, "Bypass");
  lv_obj_align(cb_bypass_on, LV_ALIGN_TOP_LEFT, 331, 10);
  lv_obj_add_style(cb_bypass_on, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_bypass_on, event_bypass_switch, LV_EVENT_CLICKED, NULL);

  lv_obj_t *btn_settings_back = lv_btn_create(tab_settings);
  lv_obj_add_style(btn_settings_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_settings_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_settings_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_settings_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab edit

  edit_txtentry  = lv_textarea_create(tab_edit);
  lv_obj_add_style(edit_txtentry, &label_style1, LV_PART_MAIN);

  lv_textarea_set_one_line(edit_txtentry, true);
  lv_obj_set_size(edit_txtentry, 400, 45);
  lv_obj_align(edit_txtentry, LV_ALIGN_TOP_MID, 00, 10);
  edit_keyboard = lv_keyboard_create(tab_edit);
  lv_keyboard_set_textarea(edit_keyboard, edit_txtentry);
  lv_obj_align(edit_keyboard, LV_ALIGN_TOP_MID, 00, 100);
  lv_obj_add_event_cb(edit_keyboard,  event_keyboardhandler, LV_EVENT_ALL, NULL);

  // tab filetransfer

  btn_filetransfer_start = lv_btn_create(tab_filetransfer);
  lv_obj_add_style(btn_filetransfer_start, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_filetransfer_start, event_startfiletransfer, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_filetransfer_start);
  lv_label_set_text(label, "Start");
  lv_obj_center(label);
  lv_obj_align(btn_filetransfer_start, LV_ALIGN_TOP_LEFT, 0, 0);

  btn_filetransfer_cancel = lv_btn_create(tab_filetransfer);
  lv_obj_add_style(btn_filetransfer_cancel, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_filetransfer_cancel, event_cancelfiletransfer, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_filetransfer_cancel);
  lv_label_set_text(label, "Cancel");
  lv_obj_center(label);
  lv_obj_align(btn_filetransfer_cancel, LV_ALIGN_TOP_LEFT, 0, 67);

  lbl_filetransfer_info = lv_label_create(tab_filetransfer);
  lv_obj_add_style(lbl_filetransfer_info, &label_style2, LV_PART_MAIN);
  lv_label_set_text(lbl_filetransfer_info, "");
  lv_obj_align(lbl_filetransfer_info, LV_ALIGN_TOP_LEFT, 180, 13);

  btn_filetransfer_back = lv_btn_create(tab_filetransfer);
  lv_obj_add_style(btn_filetransfer_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_filetransfer_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_filetransfer_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_filetransfer_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab filedelete

  rol_filedelete_file = lv_roller_create(tab_filedelete);
  lv_roller_set_options(rol_filedelete_file,
                        "-\n"
                        "-\n"
                        "-",
                        LV_ROLLER_MODE_INFINITE);

  lv_obj_set_style_text_font(rol_filedelete_file, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_font(rol_filedelete_file, &lv_font_montserrat_24, LV_PART_SELECTED);
  lv_roller_set_visible_row_count(rol_filedelete_file, 5);
  lv_obj_set_width(rol_filedelete_file, 140);
  lv_obj_align(rol_filedelete_file, LV_ALIGN_LEFT_MID, 0, 20);

  lv_obj_add_event_cb(rol_filedelete_file, event_filedelete_select, LV_EVENT_ALL, NULL);

  label = lv_label_create(tab_filedelete);
  lv_label_set_text(label, "File");
  lv_obj_add_style(label, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(label, rol_filedelete_file, LV_ALIGN_OUT_TOP_MID, 0, -25);

  btn_filedelete_delete = lv_btn_create(tab_filedelete);
  lv_obj_add_style(btn_filedelete_delete, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_filedelete_delete, event_filedelete, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_filedelete_delete);
  lv_label_set_text(label, LV_SYMBOL_TRASH);
  lv_obj_center(label);
  lv_obj_align(btn_filedelete_delete, LV_ALIGN_TOP_RIGHT, 0, 201);

  btn_filedelete_back = lv_btn_create(tab_filedelete);
  lv_obj_add_style(btn_filedelete_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_filedelete_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_filedelete_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_filedelete_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab tab_presetsmain

  btn_presetsmain_preset0 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset0, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset0, event_mainpresets_sel0, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset0 = lv_label_create(btn_presetsmain_preset0);
  lv_obj_add_style(lbl_presetsmain_preset0, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset0, LV_ALIGN_TOP_LEFT, 0, 0);

  btn_presetsmain_preset1 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset1, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset1, event_mainpresets_sel1, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset1 = lv_label_create(btn_presetsmain_preset1);
  lv_obj_add_style(lbl_presetsmain_preset1, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset1, LV_ALIGN_TOP_LEFT, 0, 52);

  btn_presetsmain_preset2 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset2, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset2, event_mainpresets_sel2, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset2 = lv_label_create(btn_presetsmain_preset2);
  lv_obj_add_style(lbl_presetsmain_preset2, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset2, LV_ALIGN_TOP_LEFT, 0, 104);

  btn_presetsmain_preset3 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset3, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset3, event_mainpresets_sel3, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset3 = lv_label_create(btn_presetsmain_preset3);
  lv_obj_add_style(lbl_presetsmain_preset3, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset3, LV_ALIGN_TOP_LEFT, 0, 156);

  btn_presetsmain_preset4 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset4, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset4, event_mainpresets_sel4, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset4 = lv_label_create(btn_presetsmain_preset4);
  lv_obj_add_style(lbl_presetsmain_preset4, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset4, LV_ALIGN_TOP_LEFT, 0, 208);

  btn_presetsmain_preset5 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset5, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset5, event_mainpresets_sel5, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset5 = lv_label_create(btn_presetsmain_preset5);
  lv_obj_add_style(lbl_presetsmain_preset5, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset5, LV_ALIGN_TOP_LEFT, 0, 260);


  // eot level set screen

  slider_set_eot_level = lv_slider_create(tab_seteotlevel);

  lv_obj_remove_style_all(slider_set_eot_level);

  lv_obj_add_style(slider_set_eot_level, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_set_eot_level, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_set_eot_level, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_set_eot_level, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_set_eot_level, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_set_eot_level, 15, 200);

  // calibrate range
  lv_slider_set_range(slider_set_eot_level, 0, 37);

  lv_obj_align(slider_set_eot_level, LV_ALIGN_TOP_LEFT, 50, 60);

  lv_obj_add_event_cb(slider_set_eot_level, event_eot_level_change, LV_EVENT_ALL, NULL);

  lv_obj_t *slider_label2 = lv_label_create(tab_seteotlevel);
  lv_label_set_text(slider_label2, "Beep level");
  lv_obj_add_style(slider_label2, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(slider_label2, slider_set_eot_level, LV_ALIGN_OUT_TOP_MID, 0, -25);
  slider_set_eot_mp3_level = lv_slider_create(tab_seteotlevel);

  lv_obj_remove_style_all(slider_set_eot_mp3_level);

  lv_obj_add_style(slider_set_eot_mp3_level, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_set_eot_mp3_level, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_set_eot_mp3_level, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_set_eot_mp3_level, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_set_eot_mp3_level, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_set_eot_mp3_level, 15, 200);

  // calibrate range
  lv_slider_set_range(slider_set_eot_mp3_level, 0, 37);

  lv_obj_align(slider_set_eot_mp3_level, LV_ALIGN_TOP_LEFT, 170, 60);

  lv_obj_add_event_cb(slider_set_eot_mp3_level, event_eot_mp3_level_change, LV_EVENT_ALL, NULL);

  lv_obj_t *slider_label3 = lv_label_create(tab_seteotlevel);
  lv_label_set_text(slider_label3, "Clip level");
  lv_obj_add_style(slider_label3, &label_style4, LV_PART_MAIN);

  lv_obj_align_to(slider_label3, slider_set_eot_mp3_level, LV_ALIGN_OUT_TOP_MID, 0, -25);

  btn_seteotlevel_back = lv_btn_create(tab_seteotlevel);
  lv_obj_add_style(btn_seteotlevel_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_seteotlevel_back, event_eotscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_seteotlevel_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_seteotlevel_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // set amplifier

  lv_obj_t *vuframe3 = lv_obj_create(tab_setamplifier);
  lv_obj_set_size(vuframe3, 29, 304);
  lv_obj_align(vuframe3, LV_ALIGN_TOP_LEFT, 0, 8);
  lv_obj_set_style_bg_opa(vuframe3, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_radius(vuframe3, 0, LV_PART_MAIN);
  lv_obj_clear_flag(vuframe3, LV_OBJ_FLAG_SCROLLABLE);

  slider_set_amplifier = lv_slider_create(tab_setamplifier);
  lv_obj_remove_style_all(slider_set_amplifier);
  lv_obj_add_style(slider_set_amplifier, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_set_amplifier, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_set_amplifier, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_set_amplifier, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_set_amplifier, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_set_amplifier, 15, 200);
  lv_slider_set_range(slider_set_amplifier, 1, 10);
  lv_obj_align(slider_set_amplifier, LV_ALIGN_TOP_LEFT, 90, 60);
  lv_obj_add_event_cb(slider_set_amplifier, event_amp_level_change, LV_EVENT_ALL, NULL);
  lv_obj_t *slider_label4 = lv_label_create(tab_setamplifier);
  lv_label_set_text(slider_label4, "Amplifier");
  lv_obj_add_style(slider_label4, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(slider_label4, slider_set_amplifier, LV_ALIGN_OUT_TOP_MID, 0, -25);

  btn_setamplifier_back = lv_btn_create(tab_setamplifier);
  lv_obj_add_style(btn_setamplifier_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_setamplifier_back, event_settingsscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_setamplifier_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_setamplifier_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // vu meter settings

  lv_obj_t *vuframe4 = lv_obj_create(tab_setvurange);
  lv_obj_set_size(vuframe4, 29, 304);
  lv_obj_align(vuframe4, LV_ALIGN_TOP_LEFT, 0, 8);
  lv_obj_set_style_bg_opa(vuframe4, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_radius(vuframe4, 0, LV_PART_MAIN);
  lv_obj_clear_flag(vuframe4, LV_OBJ_FLAG_SCROLLABLE);

  slider_set_vu_range = lv_slider_create(tab_setvurange);
  lv_obj_remove_style_all(slider_set_vu_range);
  lv_obj_add_style(slider_set_vu_range, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_set_vu_range, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_set_vu_range, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_set_vu_range, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_set_vu_range, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_set_vu_range, 15, 200);
  lv_slider_set_range(slider_set_vu_range, 0, 80);
  lv_obj_align(slider_set_vu_range, LV_ALIGN_TOP_LEFT, 90, 60);
  lv_obj_add_event_cb(slider_set_vu_range, event_vu_level_change, LV_EVENT_ALL, NULL);
  lv_obj_t *slider_label5 = lv_label_create(tab_setvurange);
  lv_label_set_text(slider_label5, "VU range");
  lv_obj_add_style(slider_label5, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(slider_label5, slider_set_vu_range, LV_ALIGN_OUT_TOP_MID, 0, -25);

  btn_setvu_back = lv_btn_create(tab_setvurange);
  lv_obj_add_style(btn_setvu_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_setvu_back, event_settingsscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_setvu_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_setvu_back, LV_ALIGN_TOP_RIGHT, 0, 268);




  // morse level

  slider_set_morse_level = lv_slider_create(tab_setmorselevel);

  lv_obj_remove_style_all(slider_set_morse_level);

  lv_obj_add_style(slider_set_morse_level, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_set_morse_level, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_set_morse_level, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_set_morse_level, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_set_morse_level, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_set_morse_level, 15, 200);

  // calibrate range
  lv_slider_set_range(slider_set_morse_level, 0, 37);

  lv_obj_align(slider_set_morse_level, LV_ALIGN_TOP_LEFT, 50, 60);

  lv_obj_add_event_cb(slider_set_morse_level, event_morse_level_change, LV_EVENT_ALL, NULL);

  lv_obj_t *slider_label6 = lv_label_create(tab_setmorselevel);
  lv_label_set_text(slider_label6, "Morse level");
  lv_obj_add_style(slider_label6, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(slider_label6, slider_set_morse_level, LV_ALIGN_OUT_TOP_MID, 0, -25);

  btn_set_morse_level_back = lv_btn_create(tab_setmorselevel);
  lv_obj_add_style(btn_set_morse_level_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_set_morse_level_back, event_morsescr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_set_morse_level_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_set_morse_level_back, LV_ALIGN_TOP_RIGHT, 0, 268);


  // autokey level

  slider_set_autokey_level = lv_slider_create(tab_setautokeylevel);

  lv_obj_remove_style_all(slider_set_autokey_level);

  lv_obj_add_style(slider_set_autokey_level, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_set_autokey_level, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_set_autokey_level, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_set_autokey_level, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_set_autokey_level, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_set_autokey_level, 15, 200);

  lv_slider_set_range(slider_set_autokey_level, 0, 37);

  lv_obj_align(slider_set_autokey_level, LV_ALIGN_TOP_LEFT, 50, 60);

  lv_obj_add_event_cb(slider_set_autokey_level, event_autokey_level_change, LV_EVENT_ALL, NULL);

  lv_obj_t *slider_label7 = lv_label_create(tab_setautokeylevel);
  lv_label_set_text(slider_label7, "AK level");
  lv_obj_add_style(slider_label7, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(slider_label7, slider_set_autokey_level, LV_ALIGN_OUT_TOP_MID, 0, -25);

  btn_set_autokey_level_back = lv_btn_create(tab_setautokeylevel);
  lv_obj_add_style(btn_set_autokey_level_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_set_autokey_level_back, event_autokey_returnscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_set_autokey_level_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_set_autokey_level_back, LV_ALIGN_TOP_RIGHT, 0, 268);





}

// screen (tab) related

static void event_mainscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
    lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
  }
}

static void event_menuscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
    lv_tabview_set_act(tabview, TAB_MENU_REF, LV_ANIM_OFF);
  }
}

static void event_compressorscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_COMPRESSOR_REF, LV_ANIM_OFF);
  }
}

static void event_eqscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_EQUALIZER_REF, LV_ANIM_OFF);
  }
}

static void event_presetsscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_PRESETS_REF, LV_ANIM_OFF);
  }
}

static void event_effectsscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_EFFECTS_REF, LV_ANIM_OFF);
    speech_effect_selected_slider = 0;
    set_effect_sliders_labels();
  }
}

static void event_eotscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_ENDOFTRANSMISSION_REF, LV_ANIM_OFF);
  }
}

static void event_scopescr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    // turn off echo if its on
    effects_clear();
    lv_tabview_set_act(tabview, TAB_SCOPE_REF, LV_ANIM_OFF);
    // trigger frame draw
    speech_screen_update_done = false;
    // output to output
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.mux(MOD_NX1_3_2_MONOSWSLEW_ADDR, 3, 0);
    xSemaphoreGive(i2c_mutex);
  }
}

static void event_spectrumscreen(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    // turn off echo if its on
    effects_clear();
    lv_tabview_set_act(tabview, TAB_SPECTRUM_REF, LV_ANIM_OFF);
    // trigger frame draw
    speech_screen_update_done = false;
    speech_spectrum_peaktrigger = true;
    // output to output
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.mux(MOD_NX1_3_2_MONOSWSLEW_ADDR, 3, 0);
    xSemaphoreGive(i2c_mutex);
  }
}

static void event_recordscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    // turn off echo if its on
    effects_clear();

    lv_tabview_set_act(tabview, TAB_RECORDER_REF, LV_ANIM_OFF);

    speech_ptt_mode = 4;
    speech_ptt_active = false;
    speech_transmitting = false;

    speech_rec_pb_progress = 0;
    update_recorder_bar();

    // switch digital output to source for recording
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.mux(MOD_NX1_3_2_MONOSWSLEW_ADDR, 2, 0);

    // normal, switch later for playback mode
    dsp.mux(MOD_NX1_3_MONOSWSLEW_ADDR, 0, 0);
    xSemaphoreGive(i2c_mutex);
  }
}

static void event_voicescr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    // turn off echo if its on
    effects_clear();

    lv_tabview_set_act(tabview, TAB_VOICE_REF, LV_ANIM_OFF);

    speech_ptt_mode = 4;
    speech_ptt_active = false;
    speech_transmitting = false;

    speech_rec_pb_progress = 0;

    // switch digital output to source for recording
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.mux(MOD_NX1_3_2_MONOSWSLEW_ADDR, 2, 0);

    // normal, switch later for playback mode
    dsp.mux(MOD_NX1_3_MONOSWSLEW_ADDR, 0, 0);
    xSemaphoreGive(i2c_mutex);

  }
}

static void event_autokeyscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    display_autokey_timers();
    lv_tabview_set_act(tabview, TAB_AUTOKEY_REF, LV_ANIM_OFF);
    // do this because the roller build takes a while
    lv_obj_invalidate(lv_scr_act());
    lv_refr_now(NULL);
    // now build the roller
    set_autokey_soundclip_options();
  }
}


static void event_autokey_returnscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    display_autokey_timers();
    lv_tabview_set_act(tabview, TAB_AUTOKEY_REF, LV_ANIM_OFF);
  }
}

static void event_morsescr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    // turn off eot
    speech_eot_active = false;
    set_eot();
    lv_tabview_set_act(tabview, TAB_MORSE_REF, LV_ANIM_OFF);
  }
}

static void event_voxscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_VOX_REF, LV_ANIM_OFF);
  }
}

static void event_testscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    // turn off echo if its on
    effects_clear();

    // turn off eot
    speech_eot_active = false;
    set_eot();
    // ignore ptt and reset
    speech_ptt_mode = 3;
    speech_ptt_active = false;
    speech_transmitting = false;
    speech_relay_action = 1;
    lv_tabview_set_act(tabview, TAB_TEST_REF, LV_ANIM_OFF);
    // set to none to activate proper mux
    speech_test_mode = 0;
    set_test_mode_buttons_state();
  }
}

static void event_settingsscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_SETTINGS_REF, LV_ANIM_OFF);
  }
}

static void event_filetransferscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_FILETRANSFER_REF, LV_ANIM_OFF);
  }
}

static void event_presetsmainscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
    lv_tabview_set_act(tabview, TAB_PRESETSMAIN_REF, LV_ANIM_OFF);
  }
}

static void event_eotsetlevelscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_SETEOTLEVEL_REF, LV_ANIM_OFF);
  }
}


static void event_morsesetlevelscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_SETMORSELEVEL_REF, LV_ANIM_OFF);
  }
}

static void event_autokeysetlevelscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_SETAUTOKEYLEVEL_REF, LV_ANIM_OFF);
  }
}


// cleanup when going back to the menu screen
void cleanup(void) {
  // reset speech_ptt_mode to default based on operating mode
  if (speech_eot_active) {
    speech_ptt_mode = 1;
  }
  else {
    speech_ptt_mode = 0;
  }

  switch (lv_tabview_get_tab_active(tabview)) {
    case TAB_ENDOFTRANSMISSION_REF:
      // eot screen
      // cancel by flagging completion, this will exit the tasks
      speech_task1_completed = true;
      break;

    case TAB_AUTOKEY_REF:
      // clean up autokey
      //     lv_roller_set_options(rol_autokey_roller_sound, "", LV_ROLLER_MODE_NORMAL);
      lv_roller_set_options(rol_autokey_roller_sound,
                            "-loading-\n"
                            "-loading-\n"
                            "-loading-",
                            LV_ROLLER_MODE_INFINITE);

      autokey_stopped();
      break;

    case TAB_RECORDER_REF:
      // clean up pending recording
      speech_rec_pb_progress = 0;
      update_recorder_bar();
      break;

    case TAB_TEST_REF:
      // clear button
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      dsp.mux(MOD_NX1_4_2_MONOSWSLEW_ADDR, 0, 0);
      xSemaphoreGive(i2c_mutex);
      if (speech_transmitting) {
        lv_obj_clear_state(btn_test_tx, LV_STATE_CHECKED);
        lv_obj_clear_state(btn_test_tx, LV_STATE_DISABLED);
        speech_relay_action = 1;
      }
      // switch back to main signal
      select_output_source(0);
      set_sine1_on(false);
      set_sine2_on(false);
      break;

    case TAB_DELETE_REF:
      // file delete, clear list
      lv_roller_set_options(rol_filedelete_file, "", LV_ROLLER_MODE_NORMAL);
      break;

  }
}


// main screen

void vu_meter_read(void) {
  // readback 1 = full scale
  // = 2.7v pp
  // = 1.35v peak
  // = 0.95v rms
  // these values are direct out of the dsp chip, reduced by filters and coupler tx!

  // get hex value 3 bytes
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  int32_t readback = dsp.readBack(MOD_READBACK1_ALG0_VAL0_ADDR, MOD_READBACK1_ALG0_VAL0_VALUES, 3);
  xSemaphoreGive(i2c_mutex);

  if (readback & 0x800000) readback |= 0xFF000000; // sign extend
  // convert 5:19 float format
  speech_vu_readback = (float)readback / 524288.0f;

  // max vu meter value, adjust for output calibration
  // const float VU_READBACK_MAX = 0.28f;

  const float VU_READBACK_MAX = 0.007f * ((float) (82 - speech_vu_scaling));

  const int VU_LED_COUNT = 50;

  bool use_log_scale = true;
  // keep a copy because we will change it
  float readback_value = speech_vu_readback;

  // Clamp to avoid invalid values
  readback_value = fminf(fmaxf(readback_value, 0.000001f), VU_READBACK_MAX);

  if (use_log_scale) {
    // Normalize to full-scale range
    float normalized = readback_value / VU_READBACK_MAX;

    // Convert to dB relative to full-scale (now 0 dB at max)
    float vu_db = 20.0f * log10f(normalized);

    // Map from -40 dB to 0 dB → 0 to 1
    float vu_norm = (vu_db + 40.0f) / 40.0f;
    vu_norm = fminf(fmaxf(vu_norm, 0.0f), 1.0f);

    speech_vu_meter = (int)(vu_norm * VU_LED_COUNT + 0.5f);

  } else {
    // Linear scale
    float vu_norm = readback_value / VU_READBACK_MAX;
    speech_vu_meter = (int)(vu_norm * VU_LED_COUNT + 0.5f);
  }
}

// VU meter with parametric start positions and speech level
void VU_Meter_Show_Horizontal(byte speech_level, uint16_t start_posx, uint16_t start_posy) {
  uint16_t vu_posx = start_posx;
  uint16_t vu_posy = start_posy;
  for (byte vu_ledcount = 1; vu_ledcount <= 50; vu_ledcount++) {
    if (speech_level >= vu_ledcount) {
      if (vu_ledcount <= 35) {
        TFT.fillRect(vu_posx, vu_posy, 5, 25, TFT_GREEN);
      }
      else if (vu_ledcount > 35 && vu_ledcount <= 43) {
        TFT.fillRect(vu_posx, vu_posy, 5, 25, TFT_ORANGE);
      }
      else {
        TFT.fillRect(vu_posx, vu_posy, 5, 25, TFT_RED);
      }
    }
    else {
      // draw blank
      TFT.fillRect(vu_posx, vu_posy, 5, 25, TFT_BLACK);
    }
    vu_posx += 6; // Move to the next LED position
  }
}

void VU_meter_update(void) {
  VU_Meter_Show_Vertical(speech_vu_meter, 2, 310);
}

// Vertical VU meter (fills from bottom to top)
void VU_Meter_Show_Vertical(int8_t speech_level, uint16_t start_posx, uint16_t start_posy) {
  uint16_t vu_posx = start_posx;

  for (byte vu_ledcount = 1; vu_ledcount <= 50; vu_ledcount++) {
    // Adjust the first rectangle position (compensate for top-to-bottom drawing)
    uint16_t vu_posy = start_posy - (vu_ledcount * 6);  // Move up for each LED
    // Now draw the LEDs, starting at the corrected position
    if (speech_level >= vu_ledcount) {
      if (vu_ledcount <= 35) {
        TFT.fillRect(vu_posx, vu_posy, 25, 5, TFT_GREEN);
      }
      else if (vu_ledcount > 35 && vu_ledcount <= 43) {
        TFT.fillRect(vu_posx, vu_posy, 25, 5, TFT_ORANGE);
      }
      else {
        TFT.fillRect(vu_posx, vu_posy, 25, 5, TFT_RED);
      }
    }
    else {
      // Draw blank space if the level is less than the LED count
      TFT.fillRect(vu_posx, vu_posy, 25, 5, TFT_BLACK);
    }
  }
}

static void event_gain_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_gain_previous = speech_gain;
    speech_gain = lv_slider_get_value(obj);
    if (speech_gain != speech_gain_previous) {
      set_gain();
    }
  }
}

// update gain
void update_gain(void) {
  // update slider
  lv_slider_set_value(slider_main_gain, speech_gain, LV_ANIM_OFF);
  set_gain();
}

// update dsp gain potmeter
void set_gain(void) {
  // limit range
  // float db_value = -50.0f * (1.0f - ((float)speech_gain / (float)lv_slider_get_max_value(slider_main_gain)));
  float db_value = -50.0f + (float)speech_gain;
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.volume(MOD_SINGLE1_GAIN1940ALGNS1_ADDR, db_value);
  xSemaphoreGive(i2c_mutex);
}


// input amplifier gain
void set_input_amp_gain(int32_t in_gain) {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.gain(MOD_GAIN1_GAIN1940ALGNS2_ADDR, (int32_t) in_gain);
  xSemaphoreGive(i2c_mutex);
}

// esp32 return path input amplifier gain
void set_esp32_amp_gain(float in_gain) {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.gain(MOD_GAIN1_2_GAIN1940ALGNS3_ADDR, in_gain);
  xSemaphoreGive(i2c_mutex);
}

static void event_output_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_output_previous = speech_output;
    speech_output = lv_slider_get_value(obj);
    if (speech_output != speech_output_previous) {
      set_output();
    }
    if (!speech_rotary_selected_output) {
      speech_rotary_selected_output = true;
      set_main_labels();
    }
  }
}

// update output
void update_output(void) {
  lv_slider_set_value(slider_main_output, speech_output, LV_ANIM_OFF);
  set_output();
}

// update dsp gain potmeter
void set_output(void) {
  // limit range
  // float db_value = -30.0f * (1.0f - ((float)speech_output / (float)lv_slider_get_max_value(slider_main_output)));
  float db_value = -50.0f + (float)speech_output;
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.volume(MOD_SINGLE1_2_GAIN1940ALGNS5_ADDR, db_value);
  xSemaphoreGive(i2c_mutex);
}

static void event_headphone_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_headphone_previous = speech_headphone;
    speech_headphone = lv_slider_get_value(obj);
    if (speech_headphone != speech_headphone_previous) {
      set_headphone();
    }
    if (speech_rotary_selected_output) {
      speech_rotary_selected_output = false;
      set_main_labels();
    }
  }
}

// update headphone
void update_headphone(void) {
  lv_slider_set_value(slider_main_headphone, speech_headphone, LV_ANIM_OFF);
  set_headphone();
}

// headphone output stage amplifier gain
void set_hp_out_gain(int32_t out_gain) {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.gain(MOD_GAIN1_3_GAIN1940ALGNS4_ADDR, (int32_t) out_gain);
  xSemaphoreGive(i2c_mutex);
}

// update dsp gain potmeter
void set_headphone(void) {
  // limit range
  float db_value = -50.0f + (float)speech_headphone;
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.volume(MOD_SINGLE1_4_GAIN1940ALGNS8_ADDR, db_value);
  xSemaphoreGive(i2c_mutex);
}

void mute_input(boolean mute_on) {
  // bug in lib, needs 0 for mute
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.mute(MOD_MUTE4_ALG0_MUTEONOFF_ADDR, !mute_on);
  xSemaphoreGive(i2c_mutex);
  speech_input_muted = mute_on;

}

void mute_output(boolean mute_on) {
  // bug in lib, needs 0 for mute
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.mute(MOD_MUTE3_2_MUTENOSLEWALG1MUTE_ADDR, !mute_on);
  xSemaphoreGive(i2c_mutex);
}

void show_tx_led(boolean show_msg) {
  if (show_msg) {
    lv_obj_clear_flag(led_main_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_main_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(led_vox_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_vox_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(led_autokey_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_autokey_tx, LV_OBJ_FLAG_HIDDEN);
  }
  else {
    lv_obj_add_flag(led_main_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_main_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(led_vox_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_vox_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(led_autokey_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_autokey_tx, LV_OBJ_FLAG_HIDDEN);
  }
}

void set_main_labels(void) {
  lv_color_t selected_color = lv_palette_main(LV_PALETTE_GREEN);
  lv_color_t nonselected_color = lv_color_white();
  lv_obj_set_style_text_color(lbl_main_gain, selected_color, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_main_output, (speech_rotary_selected_output) ? selected_color : nonselected_color, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_main_headphone, (!speech_rotary_selected_output) ? selected_color : nonselected_color, LV_PART_MAIN);
}

// compressor screen

static void event_compressor_threshold_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_compressor_threshold_previous = speech_compressor_threshold;
    speech_compressor_threshold = lv_slider_get_value(obj);
    if (speech_compressor_threshold != speech_compressor_threshold_previous) {
      speech_compressor_update();
    }
    if (!speech_rotary_selected_threshold) {
      speech_rotary_selected_threshold = true;
      set_compressor_labels(0);
    }
  }
}

// update compressor threshold
void update_compressor_threshold(void) {
  lv_slider_set_value(slider_compressor_threshold, speech_compressor_threshold , LV_ANIM_OFF);
  speech_compressor_update();
}

static void event_compressor_ratio_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_compressor_ratio_previous = speech_compressor_ratio;
    speech_compressor_ratio = lv_slider_get_value(obj);
    if (speech_compressor_ratio != speech_compressor_ratio_previous) {
      speech_compressor_update();
    }
    if (speech_rotary_selected_threshold) {
      speech_rotary_selected_threshold = false;
      set_compressor_labels(0);
    }
  }
}

// update compressor ratio
void update_compressor_ratio(void) {
  lv_slider_set_value(slider_compressor_ratio, speech_compressor_ratio, LV_ANIM_OFF);
  speech_compressor_update();
}

static void event_compressor_postgain_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_compressor_postgain_previous = speech_compressor_postgain;
    speech_compressor_postgain = lv_slider_get_value(obj);
    if (speech_compressor_postgain != speech_compressor_postgain_previous) {
      speech_compressor_update();
    }
    if (!speech_rotary_selected_postgain) {
      speech_rotary_selected_postgain = true;
      set_compressor_labels(1);
    }
  }
}

// update compressor post gain
void update_compressor_postgain(void) {
  lv_slider_set_value(slider_compressor_postgain, speech_compressor_postgain, LV_ANIM_OFF);
  speech_compressor_update();
}

static void event_compressor_noisegate_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_compressor_noisegate_previous = speech_compressor_noisegate;
    speech_compressor_noisegate = lv_slider_get_value(obj);
    if (speech_compressor_noisegate != speech_compressor_noisegate_previous) {
      speech_noisegate_update();
    }
    if (speech_rotary_selected_postgain) {
      speech_rotary_selected_postgain = false;
      set_compressor_labels(1);
    }
  }
}

// update compressor noisegate
void update_compressor_noisegate(void) {
  lv_slider_set_value(slider_compressor_noisegate, speech_compressor_noisegate , LV_ANIM_OFF);
  speech_noisegate_update();
}

void set_compressor_labels(uint8_t labelset) {
  lv_color_t selected_color = lv_palette_main(LV_PALETTE_GREEN);
  lv_color_t nonselected_color = lv_color_white();
  if (labelset == 0) {
    lv_obj_set_style_text_color(lbl_compressor_threshold, (speech_rotary_selected_threshold) ? selected_color : nonselected_color, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_compressor_ratio, (!speech_rotary_selected_threshold) ? selected_color : nonselected_color, LV_PART_MAIN);
  }
  else {
    lv_obj_set_style_text_color(lbl_compressor_postgain, (speech_rotary_selected_postgain) ? selected_color : nonselected_color, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_compressor_noisegate, (!speech_rotary_selected_postgain) ? selected_color : nonselected_color, LV_PART_MAIN);
  }
}

// --- linspace for dB range: -90 to +6 ---
void linspace(float x1, float x2, float n, float * vect) {
  float v = (abs(x1) + abs(x2)) / (n - 1);
  for (int ctr = 0; ctr < n; ctr++) {
    vect[ctr] = x1 + (v * ctr);
  }
}

// --- 5-point smoothing filter ---
void curveSmooth5(float in[], float out[], int curveLength) {
  out[0] = in[0];
  out[1] = in[1];
  for (int j = 2; j < curveLength - 2; j++) {
    float sum = 0;
    for (int k = -2; k <= 2; k++) {
      sum += in[j + k];
    }
    out[j] = sum / 5.0f;
  }
  out[curveLength - 2] = in[curveLength - 2];
  out[curveLength - 1] = in[curveLength - 1];
}

// --- Generate Compression Curve ---
void MakeCompressCurve(speech_compressor_t &comp) {
  const int len = 34;
  float x[len];
  float y[len];
  float curve[len];

  linspace(-90.0f, 6.0f, len, x);

  float t = comp.threshold;
  float r = comp.ratio;
  float k = comp.knee;
  float halfK = k * 0.5f;

  for (int i = 0; i < len; i++) {
    float in = x[i];

    if (k > 0 && in > t - halfK && in < t + halfK) {
      // Soft knee (quadratic interpolation)
      float xk = in - (t - halfK);
      float yk = xk * xk / (2.0f * k);
      y[i] = in + (1.0f / r - 1.0f) * yk;
    } else if (in >= t + halfK) {
      // Above knee: compressed
      y[i] = t + (in - t) / r;
    } else {
      // Below knee: passthrough
      y[i] = in;
    }
  }

  // Apply optional smoothing
  curveSmooth5(y, curve, len);

  //  Serial.println("Input_dB\tOutput_dB\tfinal_dB");
  //  for (int i = 0; i < len; i++) {
  //    Serial.print(x[i], 2);
  //    Serial.print("\t");
  //    Serial.println(curve[i], 2);
  //  }

  for (int i = 0; i < len; i++) { // Coefficients of the curve calculation
    curve[i] = powf(10, (y[i] - x[i]) / 20); // Ratios of the linearized values of vect. y and x
  }

  uint16_t startMemoryAddress = MOD_COMPRESSOR1_ALG0_MONOALG10_ADDR;

  // create buffer to store converted data
  uint8_t storeData[5];
  uint8_t storeData1[5];
  uint8_t storeData2[5];
  uint8_t storeData3[5];
  uint8_t storeData4[5];

  float postgain_par = pow(10, comp.postgain / 20);
  dsp.floatToFixed(postgain_par, storeData1);
  const float FS = dsp.FS;
  // code is changed to reflect the actual values in sigmastudio, code in lib is incorrect
  float rms_tc = comp.rms_tc;  // in dB/second, e.g., 120
  float attack_par = fabsf(1.0f - powf(10.0f, -rms_tc / (10.0f * FS)));

  dsp.floatToFixed(attack_par, storeData2);

  // hold
  float hold_par = comp.hold * FS / 1000;
  dsp.floatToFixed(hold_par, storeData3);

  // decay (dB/s)
  float DecayUI = comp.decay;  // e.g. 10
  float raw = FS / 26373.0f * DecayUI;
  float decay_par = raw * 1.19209289550781e-7f;

  dsp.floatToFixed(decay_par, storeData4);

  // parameter load into Sigma DSP
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  for (int i = 0; i < 34; i++) {
    dsp.floatToFixed(curve[i], storeData);
    dsp.safeload_writeRegister(startMemoryAddress++, storeData, false);
  }
  dsp.safeload_writeRegister(MOD_COMPRESSOR1_ALG0_MONOALG1POSTGAIN_ADDR, storeData1, true);
  dsp.safeload_writeRegister(MOD_COMPRESSOR1_ALG0_MONOALG1RMS_ADDR, storeData2, true);
  dsp.safeload_writeRegister(MOD_COMPRESSOR1_ALG0_MONOALG1HOLD_ADDR, storeData3, true);
  dsp.safeload_writeRegister(MOD_COMPRESSOR1_ALG0_MONOALG1DECAY_ADDR, storeData4, true);
  xSemaphoreGive(i2c_mutex);

}

// update compressor parameters
void speech_compressor_update(void) {
  speech_compressor_parameters.threshold = -60 + speech_compressor_threshold;
  speech_compressor_parameters.ratio = speech_compressor_ratio;
  speech_compressor_parameters.postgain = speech_compressor_postgain;
  MakeCompressCurve(speech_compressor_parameters);
}

// --- Generate Noise Gate Curve ---
void MakeNoiseGateCurve(speech_noisegate_t &comp) {
  const int len = 34;
  float x[len];
  float y[len];
  float curve[len];

  linspace(-90.0f, 6.0f, len, x);

  // Noise gate parameters
  float t_gate = comp.gateThreshold;
  float k_gate = comp.gateKnee;
  float g_floor = comp.gateFloor;
  float halfK = k_gate * 0.5f;

  for (int i = 0; i < len; i++) {
    float in = x[i];
    float lowerBound = t_gate - halfK;
    float upperBound = t_gate + halfK;

    if (k_gate > 0 && in > lowerBound && in < upperBound) {
      float xk = in - lowerBound;
      float g_scale = (in - g_floor) / (2.0f * halfK * halfK);
      y[i] = g_floor + xk * xk * g_scale;
    }
    else if (in <= lowerBound) {
      y[i] = g_floor;
    }
    else {
      y[i] = in;
    }
  }

  // Apply smoothing
  curveSmooth5(y, curve, len);

  //  // Output to serial (for plotting)
  //  Serial.println("Input_dB\tOutput_dB");
  //  for (int i = 0; i < len; i++) {
  //    Serial.print(x[i], 2); Serial.print("\t");
  //    Serial.println(curve[i], 2);
  //  }

  // Convert to linear gain factors
  for (int i = 0; i < len; i++) {
    curve[i] = powf(10, (y[i] - x[i]) / 20.0f);
  }

  // DSP upload code remains unchanged
  uint16_t startMemoryAddress = MOD_COMPRESSOR2_ALG0_MONONOPOSTGAIN3DBFIX10_ADDR;
  uint8_t storeData[5];
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  for (int i = 0; i < 34; i++) {
    dsp.floatToFixed(curve[i], storeData);
    dsp.safeload_writeRegister(startMemoryAddress++, storeData, false);
  }
  xSemaphoreGive(i2c_mutex);
}

// update compressor parameters
void speech_noisegate_update(void) {
  speech_noisegate_parameters.gateThreshold =  -90 + speech_compressor_noisegate;
  speech_noisegate_parameters.gateKnee = -10;
  speech_noisegate_parameters.gateFloor = -110;
  MakeNoiseGateCurve(speech_noisegate_parameters);
}

static void event_compressor_reset(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mute_output(1);
    speech_compressor_threshold = 0;
    update_compressor_threshold();
    speech_compressor_ratio = 1;
    update_compressor_ratio();
    speech_compressor_postgain = 0;
    update_compressor_postgain();
    speech_compressor_update();
    speech_compressor_noisegate = 0;
    update_compressor_noisegate();
    speech_noisegate_update();
    mute_output(0);
  }
}

// equalizer screen

static void event_eq0_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq0_previous = speech_eq0;
    speech_eq0 = lv_slider_get_value(obj);
    if (speech_eq0 != speech_eq0_previous) {
      eqBand0.boost = speech_eq0;
      set_eq(0);
    }
    if (speech_selected_eq != 0) {
      speech_selected_eq = 0;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq1_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq1_previous = speech_eq1;
    speech_eq1 = lv_slider_get_value(obj);
    if (speech_eq1 != speech_eq1_previous) {
      eqBand1.boost = speech_eq1;
      set_eq(1);
    }
    if (speech_selected_eq != 1) {
      speech_selected_eq = 1;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq2_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq2_previous = speech_eq2;
    speech_eq2 = lv_slider_get_value(obj);
    if (speech_eq2 != speech_eq2_previous) {
      eqBand2.boost = speech_eq2;
      set_eq(2);
    }
    if (speech_selected_eq != 2) {
      speech_selected_eq = 2;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq3_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq3_previous = speech_eq3;
    speech_eq3 = lv_slider_get_value(obj);
    if (speech_eq3 != speech_eq3_previous) {
      eqBand3.boost = speech_eq3;
      set_eq(3);
    }
    if (speech_selected_eq != 3) {
      speech_selected_eq = 3;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq4_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq4_previous = speech_eq4;
    speech_eq4 = lv_slider_get_value(obj);
    if (speech_eq4 != speech_eq4_previous) {
      eqBand4.boost = speech_eq4;
      set_eq(4);
    }
    if (speech_selected_eq != 4) {
      speech_selected_eq = 4;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq5_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq5_previous = speech_eq5;
    speech_eq5 = lv_slider_get_value(obj);
    if (speech_eq5 != speech_eq5_previous) {
      eqBand5.boost = speech_eq5;
      set_eq(5);
    }
    if (speech_selected_eq != 5) {
      speech_selected_eq = 5;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq6_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq6_previous = speech_eq6;
    speech_eq6 = lv_slider_get_value(obj);
    if (speech_eq6 != speech_eq6_previous) {
      eqBand6.boost = speech_eq6;
      set_eq(6);
    }
    if (speech_selected_eq != 6) {
      speech_selected_eq = 6;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq7_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq7_previous = speech_eq7;
    speech_eq7 = lv_slider_get_value(obj);
    if (speech_eq7 != speech_eq7_previous) {
      eqBand7.boost = speech_eq7;
      set_eq(7);
    }
    if (speech_selected_eq != 7) {
      speech_selected_eq = 7;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq8_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq8_previous = speech_eq8;
    speech_eq8 = lv_slider_get_value(obj);
    if (speech_eq8 != speech_eq8_previous) {
      eqBand8.boost = speech_eq8;
      set_eq(8);
    }
    if (speech_selected_eq != 8) {
      speech_selected_eq = 8;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq9_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq9_previous = speech_eq9;
    speech_eq9 = lv_slider_get_value(obj);
    if (speech_eq9 != speech_eq9_previous) {
      eqBand9.boost = speech_eq9;
      set_eq(9);
    }
    if (speech_selected_eq != 9) {
      speech_selected_eq = 9;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq10_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq10_previous = speech_eq10;
    speech_eq10 = lv_slider_get_value(obj);
    if (speech_eq10 != speech_eq10_previous) {
      eqBand10.boost = speech_eq10;
      set_eq(10);
    }
    if (speech_selected_eq != 10) {
      speech_selected_eq = 10;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq11_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eq11_previous = speech_eq11;
    speech_eq11 = lv_slider_get_value(obj);
    if (speech_eq11 != speech_eq11_previous) {
      eqBand11.boost = speech_eq11;
      set_eq(11);
    }
    if (speech_selected_eq != 11) {
      speech_selected_eq = 11;
      set_eq_labels(speech_selected_eq);
    }
  }
}

static void event_eq_reset(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    speech_eq0 = 0;
    speech_eq1 = 0;
    speech_eq2 = 0;
    speech_eq3 = 0;
    speech_eq4 = 0;
    speech_eq5 = 0;
    speech_eq6 = 0;
    speech_eq7 = 0;
    speech_eq8 = 0;
    speech_eq9 = 0;
    speech_eq10 = 0;
    speech_eq11 = 0;
    update_eq_all_sliders();
  }
}

void set_eq_labels(uint8_t label_no) {
  lv_color_t selected_color = lv_palette_main(LV_PALETTE_GREEN);
  lv_color_t nonselected_color = lv_color_white();
  // Array of label objects
  lv_obj_t* labels[] = {lbl_eq0, lbl_eq1, lbl_eq2, lbl_eq3, lbl_eq4, lbl_eq5,
                        lbl_eq6, lbl_eq7, lbl_eq8, lbl_eq9, lbl_eq10, lbl_eq11
                       };
  // Ensure label_no is within range
  if (label_no >= sizeof(labels) / sizeof(labels[0])) {
    return; // Invalid index, do nothing
  }
  // Set colors for all labels
  for (uint8_t i = 0; i < sizeof(labels) / sizeof(labels[0]); i++) {
    lv_obj_set_style_text_color(labels[i], (i == label_no) ? selected_color : nonselected_color, LV_PART_MAIN);
  }
}

void inc_eq_slider(uint8_t slider_no) {
  // array of slider objects
  lv_obj_t* sliders[] = {
    slider_eq0, slider_eq1, slider_eq2, slider_eq3, slider_eq4, slider_eq5,
    slider_eq6, slider_eq7, slider_eq8, slider_eq9, slider_eq10, slider_eq11
  };
  // array of pointers tospeech_eq variables
  int32_t*speech_eq[] = {
    &speech_eq0, &speech_eq1, &speech_eq2, &speech_eq3, &speech_eq4, &speech_eq5,
    &speech_eq6, &speech_eq7, &speech_eq8, &speech_eq9, &speech_eq10, &speech_eq11
  };
  // ensure slider_no is within range
  if (slider_no >= sizeof(sliders) / sizeof(sliders[0])) {
    return; // Invalid index, do nothing
  }
  if (*speech_eq[slider_no] < 15) {
    // updating the slider will trigger the update of thespeech_eq value
    lv_slider_set_value(sliders[slider_no], lv_slider_get_value(sliders[slider_no]) + 1, LV_ANIM_OFF);
    // setting the value does not trigger the value change event, do it manually
    lv_obj_send_event(sliders[slider_no], LV_EVENT_VALUE_CHANGED, NULL);
  }
}

void dec_eq_slider(uint8_t slider_no) {
  // array of slider objects
  lv_obj_t* sliders[] = {
    slider_eq0, slider_eq1, slider_eq2, slider_eq3, slider_eq4, slider_eq5,
    slider_eq6, slider_eq7, slider_eq8, slider_eq9, slider_eq10, slider_eq11
  };
  // array of pointers tospeech_eq variables
  int32_t*speech_eq[] = {
    &speech_eq0, &speech_eq1, &speech_eq2, &speech_eq3, &speech_eq4, &speech_eq5,
    &speech_eq6, &speech_eq7, &speech_eq8, &speech_eq9, &speech_eq10, &speech_eq11
  };
  // ensure slider_no is within range
  if (slider_no >= sizeof(sliders) / sizeof(sliders[0])) {
    return; // Invalid index, do nothing
  }
  if (*speech_eq[slider_no] > -15) {
    // updating the slider will trigger the update of thespeech_eq value
    lv_slider_set_value(sliders[slider_no], lv_slider_get_value(sliders[slider_no]) - 1, LV_ANIM_OFF);
    // setting the value does not trigger the value change event, do it manually
    lv_obj_send_event(sliders[slider_no], LV_EVENT_VALUE_CHANGED, NULL);
  }
}

void update_eq_all_sliders(void) {
  eqBand0.boost = speech_eq0;
  lv_slider_set_value(slider_eq0, speech_eq0, LV_ANIM_OFF);
  set_eq(0);
  eqBand1.boost = speech_eq1;
  lv_slider_set_value(slider_eq1, speech_eq1, LV_ANIM_OFF);
  set_eq(1);
  eqBand2.boost = speech_eq2;
  lv_slider_set_value(slider_eq2, speech_eq2, LV_ANIM_OFF);
  set_eq(2);
  eqBand3.boost = speech_eq3;
  lv_slider_set_value(slider_eq3, speech_eq3, LV_ANIM_OFF);
  set_eq(3);
  eqBand4.boost = speech_eq4;
  lv_slider_set_value(slider_eq4, speech_eq4, LV_ANIM_OFF);
  set_eq(4);
  eqBand5.boost = speech_eq5;
  lv_slider_set_value(slider_eq5, speech_eq5, LV_ANIM_OFF);
  set_eq(5);
  eqBand6.boost = speech_eq6;
  lv_slider_set_value(slider_eq6, speech_eq6, LV_ANIM_OFF);
  set_eq(6);
  eqBand7.boost = speech_eq7;
  lv_slider_set_value(slider_eq7, speech_eq7, LV_ANIM_OFF);
  set_eq(7);
  eqBand8.boost = speech_eq8;
  lv_slider_set_value(slider_eq8, speech_eq8, LV_ANIM_OFF);
  set_eq(8);
  eqBand9.boost = speech_eq9;
  lv_slider_set_value(slider_eq9, speech_eq9, LV_ANIM_OFF);
  set_eq(9);
  eqBand10.boost = speech_eq10;
  lv_slider_set_value(slider_eq10, speech_eq10, LV_ANIM_OFF);
  set_eq(10);
  eqBand11.boost = speech_eq11;
  lv_slider_set_value(slider_eq11, speech_eq11, LV_ANIM_OFF);
  set_eq(11);
}

// update equalizer
void set_eq(uint8_t band) {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  switch (band) {
    case 0:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE0_B0_ADDR, eqBand0);
      break;

    case 1:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE1_B0_ADDR, eqBand1);
      break;

    case 2:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE2_B0_ADDR, eqBand2);
      break;

    case 3:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE3_B0_ADDR, eqBand3);
      break;

    case 4:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE4_B0_ADDR, eqBand4);
      break;

    case 5:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE5_B0_ADDR, eqBand5);
      break;

    case 6:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE6_B0_ADDR, eqBand6);
      break;

    case 7:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE7_B0_ADDR, eqBand7);
      break;

    case 8:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE8_B0_ADDR, eqBand8);
      break;

    case 9:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE9_B0_ADDR, eqBand9);
      break;

    case 10:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE10_B0_ADDR, eqBand10);
      break;

    case 11:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE11_B0_ADDR, eqBand11);
      break;

  }
  xSemaphoreGive(i2c_mutex);
}

// presets

static void event_presets_sel0(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_presets_sel = 0;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(11, TAB_PRESETS_REF,  cb_presets_selection0);
  }
}

static void event_presets_sel1(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_presets_sel = 1;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(12, TAB_PRESETS_REF,  cb_presets_selection1);
  }
}

static void event_presets_sel2(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_presets_sel = 2;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(13, TAB_PRESETS_REF,  cb_presets_selection2);
  }
}

static void event_presets_sel3(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_presets_sel = 3;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(14, TAB_PRESETS_REF,  cb_presets_selection3);
  }
}

static void event_presets_sel4(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_presets_sel = 4;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(15, TAB_PRESETS_REF,  cb_presets_selection4);
  }
}

static void event_presets_sel5(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_presets_sel = 5;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(16, TAB_PRESETS_REF,  cb_presets_selection5);
  }
}

void set_presets_selection_buttons_state(void) {
  for (int i = 0; i < sizeof(preset_checkboxes) / sizeof(preset_checkboxes[0]); i++) {
    bool should_be_checked = (i == speech_presets_sel);
    bool is_checked = (lv_obj_get_state(preset_checkboxes[i]) & LV_STATE_CHECKED);
    // Only change state if it doesn't match what we want
    if (should_be_checked && !is_checked) {
      lv_obj_add_state(preset_checkboxes[i], LV_STATE_CHECKED);
    } else if (!should_be_checked && is_checked) {
      lv_obj_clear_state(preset_checkboxes[i], LV_STATE_CHECKED);
    }
  }
}

void set_presets_labels(void) {
  for (int i = 0; i < sizeof(preset_checkboxes) / sizeof(preset_checkboxes[0]); i++) {
    // add one space for end sign otherswise trunctated
    speech_presets_msg[i].toCharArray(printbuf, 21);
    lv_checkbox_set_text(preset_checkboxes[i], printbuf);
  }
}

void set_presetsmain_labels(void) {
  for (int i = 0; i < sizeof(preset_mainbuttons) / sizeof(preset_mainbuttons[0]); i++) {
    // add one space for end sign otherswise trunctated
    speech_presets_msg[i].toCharArray(printbuf, 21);
    lv_label_set_text(preset_mainbuttons[i], printbuf);
  }
}

static void event_mainpresets_sel0(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(0);
  }
}

static void event_mainpresets_sel1(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(1);
  }
}

static void event_mainpresets_sel2(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(2);
  }
}

static void event_mainpresets_sel3(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(3);
  }
}

static void event_mainpresets_sel4(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(4);
  }
}

static void event_mainpresets_sel5(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(5);
  }
}

void mainpresets_load_return(uint8_t selected) {
  speech_presets_sel = selected;
  set_presets_selection_buttons_state();
  load_presets(speech_presets_sel);
  lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
}

static void event_presets_set(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  if (event != LV_EVENT_CLICKED) return;
  for (uint8_t i = 0; i < sizeof(preset_checkboxes) / sizeof(preset_checkboxes[0]); i++) {
    if (lv_obj_get_state(preset_checkboxes[i]) & LV_STATE_CHECKED) {
      save_presets(i);
      break; // assuming only one checkbox can be checked at a time
    }
  }
}

static void event_presets_recall(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  if (event != LV_EVENT_CLICKED) return;
  for (uint8_t i = 0; i < sizeof(preset_checkboxes) / sizeof(preset_checkboxes[0]); i++) {
    if (lv_obj_get_state(preset_checkboxes[i]) & LV_STATE_CHECKED) {
      load_presets(i);
      break; // assuming only one checkbox can be checked at a time
    }
  }
}

// effects screen

static void event_effects_echo(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_effect_echo_on = true;
      if (speech_effect_sb_on ) {
        speech_effect_sb_on = false;
        lv_obj_clear_state(cb_effect_selection2, LV_STATE_CHECKED);
      }
    }
    else {
      speech_effect_echo_on = false;
    }
    set_effects_echo_sb_slider();
    set_effect_echo_sb_mux(speech_effect_echo_on, speech_effect_sb_on);
    if (speech_effect_echo_on) {
      speech_effect_selected_slider = 1;
    }
    else {
      if (speech_effect_selected_slider == 1 or speech_effect_selected_slider == 2) {
        speech_effect_selected_slider = 0;
      }
    }
    set_effect_sliders_labels();
  }
}

static void event_effects_sb(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_effect_sb_on = true;
      if (speech_effect_echo_on) {
        speech_effect_echo_on = false;
        lv_obj_clear_state(cb_effect_selection1, LV_STATE_CHECKED);
      }
    }
    else {
      speech_effect_sb_on = false;
    }
    set_effects_echo_sb_slider();
    set_effect_echo_sb_mux(speech_effect_echo_on, speech_effect_sb_on);
    if (speech_effect_sb_on) {
      speech_effect_selected_slider = 1;
    }
    else {
      if (speech_effect_selected_slider == 1 or speech_effect_selected_slider == 2) {
        speech_effect_selected_slider = 0;
      }
    }
    set_effect_sliders_labels();
  }
}

void set_effect_echo_sb_mux(boolean effect_echo_active, boolean effect_sb_active) {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.mux(MOD_NX1_3_5_MONOSWSLEW_ADDR, ((effect_echo_active or effect_sb_active) ? 1 : 0), 0);
  xSemaphoreGive(i2c_mutex);

  if (effect_echo_active) {
    dsp.mux(MOD_NX1_3_2_MONOSWSLEW_ADDR, 1, 0);
  }
  else if (effect_sb_active) {
    dsp.mux(MOD_NX1_3_2_MONOSWSLEW_ADDR, 0, 0);
  }
  else {
    dsp.mux(MOD_NX1_3_2_MONOSWSLEW_ADDR, 3, 0);
  }
  if ((effect_echo_active or effect_sb_active)) {
    speech_effect_echo_sb_enabled = true;
  }
  else {
    speech_effect_echo_sb_enabled = false;
  }
}

static void event_effects_pitch(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    speech_effect_pitch_on = (lv_obj_get_state(obj) & LV_STATE_CHECKED);
    set_effects_pitch_slider();
    set_effect_pitch_mux(speech_effect_pitch_on);
    if (speech_effect_pitch_on) {
      speech_effect_selected_slider = 3;
    }
    else {
      if (speech_effect_selected_slider == 3) {
        speech_effect_selected_slider = 0;
      }
    }
    set_effect_sliders_labels();
  }
}

static void event_effects_ring(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    speech_effect_ring_on = (lv_obj_get_state(obj) & LV_STATE_CHECKED);
    set_effects_ring_slider();
    set_effect_ring_mux(speech_effect_ring_on);
    if (speech_effect_ring_on) {
      speech_effect_selected_slider = 4;

    }
    else {
      if (speech_effect_selected_slider == 4) {
        speech_effect_selected_slider = 0;
      }
    }
    set_effect_sliders_labels();
  }
}

void set_effects_pitch_slider(void) {
  if (speech_effect_pitch_on) {
    lv_obj_clear_state(slider_effect_pitch, LV_STATE_DISABLED);
  }
  else {
    lv_obj_add_state(slider_effect_pitch, LV_STATE_DISABLED);
  }
}

void set_effect_pitch_mux(boolean effect_active) {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.mux(MOD_NX1_3_3_MONOSWSLEW_ADDR, (effect_active ? 1 : 0), 0);
  xSemaphoreGive(i2c_mutex);
}

void set_effects_ring_slider(void) {
  if (speech_effect_ring_on) {
    lv_obj_clear_state(slider_effect_ring, LV_STATE_DISABLED);
  }
  else {
    lv_obj_add_state(slider_effect_ring, LV_STATE_DISABLED);
  }
}

void set_effect_ring_mux(boolean effect_active) {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.safeload_write(MOD_TONE1_3_ALG0_ON_ADDR, effect_active ? 0x00800000 : 0x00000000);
  dsp.mux(MOD_NX1_3_4_MONOSWSLEW_ADDR, (effect_active ? 1 : 0), 0);
  xSemaphoreGive(i2c_mutex);
}

void set_effects_echo_sb_slider(void) {
  if (speech_effect_echo_on or speech_effect_sb_on) {
    lv_obj_clear_state(slider_effect_echo_time, LV_STATE_DISABLED);
    lv_obj_clear_state(slider_effect_echo_fb, LV_STATE_DISABLED);
  }
  else {
    lv_obj_add_state(slider_effect_echo_time, LV_STATE_DISABLED);
    lv_obj_add_state(slider_effect_echo_fb, LV_STATE_DISABLED);
  }

  if (!speech_effect_sb_on) {
    lv_label_set_text(lbl_effect_slider1, "Echo\n time");
    lv_label_set_text(lbl_effect_slider2, "Echo\n feedb");
  }
  else {
    lv_label_set_text(lbl_effect_slider1, "SB\n time");
    lv_label_set_text(lbl_effect_slider2, "SB\n volume");
  }
}

// clear all effects
void effects_clear(void) {

  speech_effect_echo_on = false;
  speech_effect_sb_on = false;
  speech_effect_pitch_on = false;
  speech_effect_ring_on = false;

  speech_effect_echo_sb_enabled = false;
  speech_effect_echo_cleared = true;

  if (lv_obj_get_state(cb_effect_selection1) & LV_STATE_CHECKED) {
    lv_obj_clear_state(cb_effect_selection1, LV_STATE_CHECKED);
  }
  if (lv_obj_get_state(cb_effect_selection2) & LV_STATE_CHECKED) {
    lv_obj_clear_state(cb_effect_selection2, LV_STATE_CHECKED);
  }
  if (lv_obj_get_state(cb_effect_selection3) & LV_STATE_CHECKED) {
    lv_obj_clear_state(cb_effect_selection3, LV_STATE_CHECKED);
  }
  if (lv_obj_get_state(cb_effect_selection4) & LV_STATE_CHECKED) {
    lv_obj_clear_state(cb_effect_selection4, LV_STATE_CHECKED);
  }

  set_effect_pitch_mux(speech_effect_pitch_on);
  set_effect_ring_mux(speech_effect_ring_on);
  set_effect_echo_sb_mux(speech_effect_echo_on, speech_effect_sb_on);

}


void set_effect_sliders_labels(void) {
  lv_color_t active_color = lv_color_white();
  lv_color_t inactive_color = lv_palette_main(LV_PALETTE_GREY);
  lv_color_t selected_color = lv_palette_main(LV_PALETTE_GREEN);

  lv_obj_set_style_text_color(lbl_effect_slider1, (speech_effect_echo_on or speech_effect_sb_on) ? ((speech_effect_selected_slider == 1) ? selected_color : active_color) : inactive_color, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_effect_slider2, (speech_effect_echo_on or speech_effect_sb_on) ? ((speech_effect_selected_slider == 2) ? selected_color : active_color) : inactive_color, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_effect_slider3, (speech_effect_pitch_on) ? ((speech_effect_selected_slider == 3) ? selected_color : active_color) : inactive_color, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_effect_slider4, (speech_effect_ring_on) ? ((speech_effect_selected_slider == 4) ? selected_color : active_color) : inactive_color, LV_PART_MAIN);

}


void set_effect_checkboxes(void) {
  if (speech_effect_echo_on) {
    lv_obj_add_state(cb_effect_selection1, LV_STATE_CHECKED);
  }
  else {
    lv_obj_clear_state(cb_effect_selection1, LV_STATE_CHECKED);
  }

  if (speech_effect_sb_on) {
    lv_obj_add_state(cb_effect_selection2, LV_STATE_CHECKED);
  }
  else {
    lv_obj_clear_state(cb_effect_selection2, LV_STATE_CHECKED);
  }

  if (speech_effect_pitch_on) {
    lv_obj_add_state(cb_effect_selection3, LV_STATE_CHECKED);
  }
  else {
    lv_obj_clear_state(cb_effect_selection3, LV_STATE_CHECKED);
  }

  if (speech_effect_ring_on) {
    lv_obj_add_state(cb_effect_selection4, LV_STATE_CHECKED);
  }
  else {
    lv_obj_clear_state(cb_effect_selection4, LV_STATE_CHECKED);
  }
}

static void event_effects_echo_time_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_effect_echo_time_previous = speech_effect_echo_time;
    speech_effect_echo_time = lv_slider_get_value(obj);
    if (speech_effect_echo_time != speech_effect_echo_time_previous) {
      set_effects_echo_time();
    }
    if (speech_effect_selected_slider != 1 and !(lv_obj_get_state(obj) & LV_STATE_DISABLED)) {
      speech_effect_selected_slider = 1;
      set_effect_sliders_labels();
    }
  }
}

// update echo time
void update_effects_echo_time(void) {
  lv_slider_set_value(slider_effect_echo_time, speech_effect_echo_time, LV_ANIM_OFF);
  set_effects_echo_time();
}

// update
void set_effects_echo_time(void) {
  // difference between buffer pointers. 100 minimum (max delay) to xxx (min delay) to prevent overlap with buffer
  // buffer size - 1024, subtract 100 and divide by range (100) to get the multiplication factor
  speech_delay_offset = 100 + ((100 - speech_effect_echo_time) * 110);
}

static void event_effects_echo_fb_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t speech_effect_echo_fb_previous = speech_effect_echo_fb;
    speech_effect_echo_fb = lv_slider_get_value(obj);
    if (speech_effect_echo_fb != speech_effect_echo_fb_previous) {
      set_effects_echo_fb();
    }
    if (speech_effect_selected_slider != 2 and !(lv_obj_get_state(obj) & LV_STATE_DISABLED)) {
      speech_effect_selected_slider = 2;
      set_effect_sliders_labels();
    }
  }
}

// update gain
void update_effects_echo_fb(void) {
  lv_slider_set_value(slider_effect_echo_fb, speech_effect_echo_fb, LV_ANIM_OFF);
  set_effects_echo_fb();
}

// update
void set_effects_echo_fb(void) {
  float db_value = -15.0f * (1.0f - ((float)speech_effect_echo_fb / (float)lv_slider_get_max_value(slider_effect_echo_fb)));
  set_i2s_input_gain(db_value);
}

void set_i2s_input_gain(float db_in) {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.volume(MOD_SINGLE1_3_GAIN1940ALGNS6_ADDR, db_in);
  xSemaphoreGive(i2c_mutex);
}

static void event_effects_pitch_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  //  if ((event == LV_EVENT_CLICKED | LV_EVENT_VALUE_CHANGED)) {
  if ((event == LV_EVENT_VALUE_CHANGED)) {
    int32_t speech_effect_pitch_previous = speech_effect_pitch;
    speech_effect_pitch = lv_slider_get_value(obj);
    if (speech_effect_pitch != speech_effect_pitch_previous) {
      set_effects_pitch();
    }
    if (speech_effect_selected_slider != 3 and !(lv_obj_get_state(obj) & LV_STATE_DISABLED)) {
      speech_effect_selected_slider = 3;
      set_effect_sliders_labels();
    }
  }
}

// update pitch
void update_effects_pitch(void) {
  lv_slider_set_value(slider_effect_pitch, speech_effect_pitch, LV_ANIM_OFF);
  set_effects_pitch();
}

// update pitch transposer
void set_effects_pitch(void) {
  float param = (speech_effect_pitch / 20.0f) * 0.00250f;
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.safeload_write(MOD_PITCHTRANS1_ALG0_PITCHSHIFTSALG1FREQ_ADDR, param);
  xSemaphoreGive(i2c_mutex);
}

static void event_effects_ring_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  //  if ((event == LV_EVENT_CLICKED | LV_EVENT_VALUE_CHANGED)) {
  if ((event == LV_EVENT_VALUE_CHANGED)) {
    int32_t speech_effect_ring_previous = speech_effect_ring;
    speech_effect_ring = lv_slider_get_value(obj);
    if (speech_effect_ring != speech_effect_ring_previous) {
      set_effects_ring();
    }
    if (speech_effect_selected_slider != 4 and !(lv_obj_get_state(obj) & LV_STATE_DISABLED)) {
      speech_effect_selected_slider = 4;
      set_effect_sliders_labels();
    }
  }
}

// update ring
void update_effects_ring(void) {
  if (speech_effect_ring > 150) {
    speech_effect_ring = 150;
  }
  else if (speech_effect_ring < 20) {
    speech_effect_ring = 20;
  }

  lv_slider_set_value(slider_effect_ring, speech_effect_ring, LV_ANIM_OFF);
  set_effects_ring();
}

// update ring modulator
void set_effects_ring(void) {
  if (speech_effect_ring >= 20 and speech_effect_ring <= 150) {
    //  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.sineSource(MOD_STATIC_TONE1_3_ALG0_MASK_ADDR, speech_effect_ring);
    //  xSemaphoreGive(i2c_mutex);
  }
}

// eot screen

static void event_eot_select0(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_eot_selection = 0;
      set_eot_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
}

static void event_eot_select1(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_eot_selection = 1;
      set_eot_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
}

void set_eot_selection_buttons_state(void) {
  if (speech_eot_state != 0) {
    lv_obj_add_state(rol_eot_roller_beep, LV_STATE_DISABLED);
    lv_obj_add_state(rol_eot_roller_sound, LV_STATE_DISABLED);
    lv_obj_add_state(cb_eot_sel_beep, LV_STATE_DISABLED);
    lv_obj_add_state(cb_eot_sel_sound, LV_STATE_DISABLED);
    lv_obj_add_state(btn_eot_play, LV_STATE_DISABLED);
    lv_obj_add_state(cb_eot_on, LV_STATE_DISABLED);
    lv_obj_add_state(cb_eot_tx_on, LV_STATE_DISABLED);
    lv_obj_add_state(btn_eot_set_level, LV_STATE_DISABLED);
  }
  else {
    lv_obj_clear_state(cb_eot_sel_beep, LV_STATE_DISABLED);
    lv_obj_clear_state(cb_eot_sel_sound, LV_STATE_DISABLED);
    lv_obj_clear_state(btn_eot_play, LV_STATE_DISABLED);
    lv_obj_clear_state(cb_eot_on, LV_STATE_DISABLED);
    lv_obj_clear_state(cb_eot_tx_on, LV_STATE_DISABLED);
    lv_obj_clear_state(btn_eot_set_level, LV_STATE_DISABLED);

    switch (speech_eot_selection) {
      // beep selection
      case 0:
        lv_obj_add_state(cb_eot_sel_beep, LV_STATE_CHECKED);
        lv_obj_clear_state(cb_eot_sel_sound, LV_STATE_CHECKED);
        lv_obj_clear_state(rol_eot_roller_beep, LV_STATE_DISABLED);
        lv_obj_add_state(rol_eot_roller_sound, LV_STATE_DISABLED);
        break;

      // sound clip selection
      case 1:
        lv_obj_clear_state(cb_eot_sel_beep, LV_STATE_CHECKED);
        lv_obj_add_state(cb_eot_sel_sound, LV_STATE_CHECKED);
        lv_obj_add_state(rol_eot_roller_beep, LV_STATE_DISABLED);
        lv_obj_clear_state(rol_eot_roller_sound, LV_STATE_DISABLED);
        break;

    }
  }
}

static void event_eot_beep_roller(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target_obj(e);
  if (code == LV_EVENT_VALUE_CHANGED) {
    speech_eot_beep_selected = lv_roller_get_selected(obj) + 1;
  }
}

static void set_eot_beep_roller(void)
{
  lv_roller_set_selected(rol_eot_roller_beep, speech_eot_beep_selected - 1, LV_ANIM_OFF);
}

static void event_eot_sound_roller(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target_obj(e);
  if (code == LV_EVENT_VALUE_CHANGED) {
    speech_eot_sound_selected = lv_roller_get_selected(obj) + 1;
  }
}

static void set_eot_sound_roller(void)
{
  lv_roller_set_selected(rol_eot_roller_sound, speech_eot_sound_selected - 1, LV_ANIM_OFF);
}

static void event_eot_switch(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_eot_active = true;
      set_eot();
      // disable vox if on
      if (lv_obj_get_state(cb_vox_on) & LV_STATE_CHECKED) {
        lv_obj_clear_state(cb_vox_on, LV_STATE_CHECKED);
        if (!speech_input_muted) {
          mute_input(true);
        }
        speech_vox_state = 0;
      }
    }
    else {
      speech_eot_active = false;
      set_eot();
    }
  }
}

static void event_eot_play(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (speech_eot_selection != 0) {
      // do this here too to prevent feedback in this mode
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      dsp.mux(MOD_NX1_3_MONOSWSLEW_ADDR, 1, 0);
      xSemaphoreGive(i2c_mutex);
    }
    if (lv_obj_get_state(cb_eot_tx_on) & LV_STATE_CHECKED) {
      vTaskDelay(pdMS_TO_TICKS(150));
      transmit(true);
    }
    else {
      // headphone
    }
    speech_eot_state = 1;
    set_eot_selection_buttons_state();
    eot_play();
  }
}

void set_eot(void) {
  if (speech_eot_active) {
    speech_ptt_mode = 1;
    lv_obj_add_state(cb_eot_on, LV_STATE_CHECKED);
    lv_obj_add_state(cb_main_eot_on, LV_STATE_CHECKED);
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.mute(MOD_MUTE3_3_MUTENOSLEWALG2MUTE_ADDR, 1);
    xSemaphoreGive(i2c_mutex);
    set_eot_signal_gain(speech_eot_level);
  }
  else {
    speech_ptt_mode = 0;
    lv_obj_clear_state(cb_eot_on, LV_STATE_CHECKED);
    lv_obj_clear_state(cb_main_eot_on, LV_STATE_CHECKED);
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.mute(MOD_MUTE3_3_MUTENOSLEWALG2MUTE_ADDR, 0);
    xSemaphoreGive(i2c_mutex);
    set_eot_signal_gain(0);
  }
}

void set_eot_beep_options(void) {
  // make sure we have enough space for 999 selections
  char speech_eot_beep_options[EOT_BEEP_COUNT * 5];
  // make sure its an empty string to start
  speech_eot_beep_options[0] = '\0';
  for (int i = 1; i <= EOT_BEEP_COUNT; i++) {
    // temporary buffer for each number
    char buffer[8];
    if (i < EOT_BEEP_COUNT) {
      snprintf(buffer, sizeof(buffer), "%d\n", i);  // Append number with newline
    } else {
      snprintf(buffer, sizeof(buffer), "%d", i);    // Last number, no newline
    }
    // append number and newline
    strcat(speech_eot_beep_options, buffer);
  }
  lv_roller_set_options(rol_eot_roller_beep, speech_eot_beep_options, LV_ROLLER_MODE_INFINITE);
}

void set_eot_soundclip_options(void) {
  // Vector to store valid numeric filenames
  std::vector<int> fileNumbers;

  // Open the SD card root directory
  File root = SD.open("/");
  if (!root) {
    Serial.println("Failed to open SD card!");
    return;
  }

  // already add the prerecorded messages
  speech_eot_sound_count = 5;
  // Read files and collect valid numbers
  while (true) {
    File file = root.openNextFile();
    if (!file) break;  // No more files
    String filename = String(file.name());
    file.close();
    // must be 7 characters long, start with 3 digits, and end in ".bin"
    if (filename.length() == 7 && filename.endsWith(".bin") &&
        isDigit(filename[0]) && isDigit(filename[1]) &&
        isDigit(filename[2])) {
      // Convert numeric part to an integer and store in vector
      int fileNumber = filename.substring(0, 3).toInt();
      fileNumbers.push_back(fileNumber);
      speech_eot_sound_count++;
    }
  }
  root.close();

  // Sort the collected numbers
  std::sort(fileNumbers.begin(), fileNumbers.end());

  // **Calculate the required buffer size**
  size_t totalSize = 1;  // Start with 1 for the null terminator

  // Add sizes of pre-recorded messages
  for (uint8_t i = 0; i < 5; i++) {
    totalSize = totalSize + speech_voice_msg[i].length() + 2;
  }

  // Add sizes of file numbers (each 3-digit number + newline)
  totalSize += (fileNumbers.size() * 4);  // 3 digits + '\n'

  // char *options = (char *)malloc(totalSize);
  char *options = (char *)heap_caps_malloc(totalSize, MALLOC_CAP_SPIRAM);
  if (!options) {
    Serial.println("Memory allocation failed!");
    return;
  }
  options[0] = '\0';  // Ensure it's an empty string

  for (uint8_t i = 0; i < 5; i++) {
    // Truncate to 9 characters (or fewer if the string is shorter)
    String truncated = speech_voice_msg[i].substring(0, 9);
    strcat(options, truncated.c_str()); // Append truncated C-string
    strcat(options, "\n"); // Add newline
  }

  // Build the sorted options string
  for (size_t i = 0; i < fileNumbers.size(); i++) {
    char buffer[6];
    snprintf(buffer, sizeof(buffer), "%03d\n", fileNumbers[i]);  // Format as 3-digit
    strcat(options, buffer);
  }

  // Remove last newline (if any)
  if (strlen(options) > 0) {
    options[strlen(options) - 1] = '\0';
  }

  lv_roller_set_options(rol_eot_roller_sound, options, LV_ROLLER_MODE_INFINITE);

  // Free dynamically allocated memory
  free(options);

}

void eot_play(void) {
  // skip if already locked
  if (!speech_lock and speech_eot_state == 1 and !speech_eot_block) {
    // mute already otherwise the sound pops up again briefly at the end of the eot until the transmit relay is turned off in the main loop
    mute_input(true);
    speech_eot_state = 2;
    speech_lock = true;
    if (speech_eot_selection == 0) {
      xTaskNotifyGive(xTaskGetHandle("RogerBeep")); // Trigger beep
    }
    else {
      // sound rb
      speech_eot_block = true;
      if (speech_eot_sound_selected > 0 and speech_eot_sound_selected < 6) {
        // prerecorded messages
        strncpy(speech_filename, speech_voice_filenames[speech_eot_sound_selected], sizeof(speech_filename) - 1);
        speech_filename[sizeof(speech_filename) - 1] = '\0';  // Ensure null termination
      }
      else if (speech_eot_sound_selected > 5 and speech_eot_sound_selected <= speech_eot_sound_count) {
        char roller_text[5];  // Buffer to store roller selection (max 3 digits + null terminator)
        // Get the selected string (up to 3 digits) from the LVGL roller
        lv_roller_get_selected_str(rol_eot_roller_sound, roller_text, sizeof(roller_text));
        // Format the final string with prefix "/" and suffix ".bin"
        snprintf(speech_filename, sizeof(speech_filename), "/%s.bin", roller_text);
      }
      setup_sound_rb_playback();
    }
  }
  else {
    Serial.println("Already triggered");
  }
}

// update dsp test/eot gain potmeter
void set_eot_signal_gain(uint8_t setting) {
  // limit range
  //float db_value = -60.0f * (1.0f - (setting / 20));
  float db_value = -50.0f + (float)setting;
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.volume(MOD_SINGLE1_5_GAIN1940ALGNS9_ADDR, db_value);
  xSemaphoreGive(i2c_mutex);
}

// scope screen

void scope_draw_screen(boolean fullscreen) {
  if (fullscreen) {
    TFT.drawRect(0, 0, 480, 251, TFT_BLUE);
  }
  TFT.drawLine(0, 125, 479, 125, TFT_GREEN);
}

void scope_sample_plot(void) {
  // setup
  int32_t scope_index = 0;
  float dc_offset = 0.0f;

  // Read I2S data
  xSemaphoreTake(i2s_mutex, portMAX_DELAY);
  i2s_read(I2S_NUM, (void*)i2s_buffer, sizeof(i2s_buffer), &speech_bytes_read, portMAX_DELAY);
  xSemaphoreGive(i2s_mutex);
  // calculate DC offset (average of all samples)
  for (int i = 0; i < 1024; i++) {
    dc_offset += (i2s_buffer[i] >> 8) * (1.0f / 8388608.0f);
  }
  dc_offset /= 1024.0f;

  // autoscale calculation
  if (speech_scope_autoscale) {
    float peak = 0.0f;
    for (int i = 0; i < 1024; i++) {
      float sample_val = (i2s_buffer[i] >> 8) * (1.0f / 8388608.0f) - dc_offset;
      float abs_sample = fabsf(sample_val);
      if (abs_sample > peak) peak = abs_sample;
    }
    if (peak < 0.0001f) peak = 0.0001f;
    speech_scope_scale = 120.0f / peak;
    if (speech_scope_scale < 1) speech_scope_scale = 1;
    speech_scope_autoscale = false;
  }

  // scale and clamp samples (with DC offset removal)
  for (int i = 0; i < 1024; i++) {
    float sample_val = (i2s_buffer[i] >> 8) * (1.0f / 8388608.0f) - dc_offset;
    sample_val *= speech_scope_scale;
    // clamp to vertical bounds with proper rounding
    if (sample_val > 124.0f) sample_val = 124.0f;
    if (sample_val < -124.0f) sample_val = -124.0f;
    i2s_buffer[i] = (int32_t)(sample_val + (sample_val >= 0 ? 0.5f : -0.5f)); // Proper rounding
  }

  // trigger point detection at zero crossing
  for (int i = 1; i < 500; i++) {
    if (i2s_buffer[i] >= 0 && i2s_buffer[i - 1] < 0) {
      scope_index = i;
      break;
    }
  }

  // clear previous trace
  for (uint16_t p = 0; p < 476; p++) {
    TFT.drawLine(p + 1, 125 - speech_scope_previous[p],
                 p + 2, 125 - speech_scope_previous[p + 1], TFT_BLACK);
  }
  scope_draw_screen(false);
  // draw new trace
  for (uint16_t p = 0; p < 476; p++) {
    TFT.drawLine(p + 1, 125 - i2s_buffer[scope_index + p],
                 p + 2, 125 - i2s_buffer[scope_index + p + 1], TFT_WHITE);
  }

  // store current trace for next erase operation
  for (uint16_t pointer = 0; pointer < 478; pointer++) {
    speech_scope_previous[pointer] = i2s_buffer[scope_index + pointer];
  }
}

void scope_clear(void) {
  // blank out previous plot with background color
  for (uint16_t pointer = 0; pointer < 476 ; pointer++) {
    TFT.drawLine(pointer + 1, 125 - speech_scope_previous[pointer],
                 pointer + 2, 125 - speech_scope_previous[pointer + 1], TFT_BLACK);
  }
  scope_draw_screen(false);
  // keep all
  for (uint16_t pointer = 0; pointer < 478 ; pointer++) {
    speech_scope_previous[pointer] = 0;
  }
}

// spectrum screen

void spectrum_sample(void) {
  // Read I2S data
  xSemaphoreTake(i2s_mutex, portMAX_DELAY);
  // need 2048 samples
  i2s_read(I2S_NUM, (void*)i2s_buffer, sizeof(i2s_buffer), &speech_bytes_read, portMAX_DELAY);
  memcpy(&speech_spectrum_buffer[0], i2s_buffer, 1024 * sizeof(int32_t));
  i2s_read(I2S_NUM, (void*)i2s_buffer, sizeof(i2s_buffer), &speech_bytes_read, portMAX_DELAY);
  memcpy(&speech_spectrum_buffer[1024], i2s_buffer, 1024 * sizeof(int32_t));
  xSemaphoreGive(i2s_mutex);
  // convert to float with proper 24-bit scaling
  for (int i = 0; i < FFT_SIZE; i++) {
    speech_spectrum_vReal[i] = (float)speech_spectrum_buffer[i] * (1.0f / 8388608.0f);
    speech_spectrum_vImag[i] = 0.0f;
  }
  // run FFT
  FFT.dcRemoval();
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // improved normalization
  const float window_coherent_gain = 0.54f; // Hamming window
  const float fft_norm = 2.0f / (FFT_SIZE * window_coherent_gain);

  // adjust this to set 0dB reference
  const float reference_level = 30.0f;

  // dBFS conversion with calibration
  for (int i = 0; i < FFT_SIZE / 2; i++) {
    float magnitude = speech_spectrum_vReal[i] * fft_norm;
    magnitude = max(magnitude, 1e-12f); // Avoid log(0)
    speech_spectrum_vReal[i] = 20.0f * log10f(magnitude) - reference_level;
  }

  // compute band averages (in dBFS)
  for (int band = 0; band < NUM_BANDS; band++) {
    int startBin = (band * NUM_BINS_TO_PROCESS) / NUM_BANDS;
    int endBin   = ((band + 1) * NUM_BINS_TO_PROCESS) / NUM_BANDS;
    float sum = 0;
    int count = 0;
    for (int bin = startBin; bin < endBin; bin++) {
      if (speech_spectrum_vReal[bin] > -80.0f) { // ignore below -80 dBFS
        sum += speech_spectrum_vReal[bin];
        count++;
      }
    }
    float average = (count > 0) ? (sum / count) : -80.0f;
    speech_spectrum_band_values[band] = average;
  }

  // normalize to 0-127 for display with adjustable range
  const float DISPLAY_MIN_DB = -70.0f;
  const float DISPLAY_MAX_DB = 6.0f; // Increased headroom
  const float db_range = DISPLAY_MAX_DB - DISPLAY_MIN_DB;
  uint8_t peakvalue = 0;
  uint8_t peakpointer = 0;
  for (int band = 0; band < NUM_BANDS; band++) {
    float dbVal = speech_spectrum_band_values[band];
    // clamp to display range
    if (dbVal < DISPLAY_MIN_DB) dbVal = DISPLAY_MIN_DB;
    if (dbVal > DISPLAY_MAX_DB) dbVal = DISPLAY_MAX_DB;
    // map to 0-127
    uint8_t pixVal = (uint8_t)(127.0f * (dbVal - DISPLAY_MIN_DB) / db_range);
    speech_spectrum_band_values[band] = pixVal;
    if (speech_spectrum_peaktrigger) {
      if (pixVal > peakvalue) {
        peakvalue = pixVal;
        peakpointer = band;
      }
    }
  }
  // peak trigger handling
  if (speech_spectrum_peaktrigger) {
    speech_spectrum_cursor = peakpointer;
    speech_spectrum_peaktrigger = false;
    spectrum_update_cursor(speech_spectrum_cursor);
  }
}

void spectrum_show(void) {
  uint16_t startpos = 1;
  for (byte band = 0; band < NUM_BANDS; band++) {
    int height = speech_spectrum_band_values[band];
    if (height >= 0) {
      TFT.fillRect(startpos, 129 - height, 4, height, TFT_GREEN);
      TFT.fillRect(startpos, 1, 4, 128 - height, TFT_BLACK);
    }
    startpos += 4;
  }
}

void spectrum_clear(void) {
  uint16_t startpos = 1;
  for (byte band = 0; band < NUM_BANDS; band++) {
    TFT.fillRect(startpos, 1, 4, 128, TFT_BLACK);
    startpos += 4;
  }
}

void spectrum_draw_screen(boolean fullscreen) {
  if (fullscreen) {
    //  TFT.drawRect(0, 0, 480, 247, TFT_BLUE);
    TFT.drawRect(0, 0, 480, 130, TFT_BLUE);
  }
}

// update cursor and value
void spectrum_update_cursor(uint8_t s_cursor) {
  static const uint8_t startX = 1;
  static const uint8_t bandWidth = 4;
  static const uint8_t spacing = 0;
  static const uint8_t cursorY = 135;

  // calculate X-position
  uint16_t x = startX + s_cursor * (bandWidth + spacing);

  // erase previous cursor if any
  if (speech_spectrum_cursor_last_pos != 255) {
    uint16_t lastX = startX + speech_spectrum_cursor_last_pos * (bandWidth + spacing);
    TFT.fillTriangle(lastX + 0, cursorY, lastX + 2, cursorY - 4, lastX + 4, cursorY, TFT_BLACK);
  }
  // draw new cursor
  TFT.fillTriangle(x + 0, cursorY, x + 2, cursorY - 4, x + 4, cursorY, TFT_WHITE);
  // store current position for erase
  speech_spectrum_cursor_last_pos = s_cursor;
  const double binWidth = 48000.0 / 2048.0;
  const int maxBin = 170;
  const int bandCount = 119;
  double bandBinStart = (speech_spectrum_cursor * (double)maxBin) / bandCount;
  double bandBinMid = bandBinStart + ((double)maxBin / bandCount) / 2.0;
  double freq = bandBinMid * binWidth;
  static char buf[20];
  snprintf(buf, sizeof(buf), "f: %.0f Hz", freq);
  lv_label_set_text(lbl_spectrum_cursor, buf);
}

// recorder screen

void show_recorder_message(boolean show_msg) {
  if (show_msg) {
    lv_obj_clear_flag(led_recorder_recording, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_recorder_recording, LV_OBJ_FLAG_HIDDEN);
  }
  else {
    lv_obj_add_flag(led_recorder_recording, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_recorder_recording, LV_OBJ_FLAG_HIDDEN);
  }
}

void update_recorder_bar(void) {
  lv_bar_set_value(bar_recorder_progress, speech_rec_pb_progress, LV_ANIM_OFF);
}

static void event_playback(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED and speech_rec_pb_state == 0) {
    // this will trigger all further actions
    speech_rec_pb_state = 4;
  }
}

// clear the 20s recording
static void event_eraserecording(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    clear_file(0);
  }
}

// cancel the playback
static void event_cancelplayback(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    // cancel by flagging completion, this will exit the tasks
    speech_task1_completed = true;
    //  speech_task2_completed = true;
  }
}

// voice msg screen

void set_voice_labels(void) {
  speech_voice_msg[0].toCharArray(printbuf, 10);
  lv_checkbox_set_text(cb_voice_selection1, printbuf);
  speech_voice_msg[1].toCharArray(printbuf, 10);
  lv_checkbox_set_text(cb_voice_selection2, printbuf);
  speech_voice_msg[2].toCharArray(printbuf, 10);
  lv_checkbox_set_text(cb_voice_selection3, printbuf);
  speech_voice_msg[3].toCharArray(printbuf, 10);
  lv_checkbox_set_text(cb_voice_selection4, printbuf);
  speech_voice_msg[4].toCharArray(printbuf, 10);
  lv_checkbox_set_text(cb_voice_selection5, printbuf);
}

static void event_voice_sel1(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_voice_msg_sel = 1;
      set_voice_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(1, TAB_VOICE_REF,  cb_voice_selection1);
  }
}

static void event_voice_sel2(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_voice_msg_sel = 2;
      set_voice_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(2, TAB_VOICE_REF,  cb_voice_selection2);
  }
}

static void event_voice_sel3(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_voice_msg_sel = 3;
      set_voice_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(3, TAB_VOICE_REF,  cb_voice_selection3);
  }
}

static void event_voice_sel4(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_voice_msg_sel = 4;
      set_voice_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(4, TAB_VOICE_REF,  cb_voice_selection4);
  }
}

static void event_voice_sel5(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_voice_msg_sel = 5;
      set_voice_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(5, TAB_VOICE_REF,  cb_voice_selection5);
  }
}

void set_voice_selection_buttons_state(void) {
  switch (speech_voice_msg_sel) {
    case 1:
      lv_obj_add_state(cb_voice_selection1, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection2, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection3, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection4, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection5, LV_STATE_CHECKED);
      break;

    case 2:
      lv_obj_clear_state(cb_voice_selection1, LV_STATE_CHECKED);
      lv_obj_add_state(cb_voice_selection2, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection3, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection4, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection5, LV_STATE_CHECKED);
      break;

    case 3:
      lv_obj_clear_state(cb_voice_selection1, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection2, LV_STATE_CHECKED);
      lv_obj_add_state(cb_voice_selection3, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection4, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection5, LV_STATE_CHECKED);
      break;

    case 4:
      lv_obj_clear_state(cb_voice_selection1, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection2, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection3, LV_STATE_CHECKED);
      lv_obj_add_state(cb_voice_selection4, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection5, LV_STATE_CHECKED);
      break;

    case 5:
      lv_obj_clear_state(cb_voice_selection1, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection2, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection3, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_voice_selection4, LV_STATE_CHECKED);
      lv_obj_add_state(cb_voice_selection5, LV_STATE_CHECKED);
      break;

  }
}

void show_voicerecorder_message(boolean show_msg) {
  if (show_msg) {
    lv_obj_clear_flag(led_voice_recording, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_voice_recording, LV_OBJ_FLAG_HIDDEN);
  }
  else {
    lv_obj_add_flag(led_voice_recording, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_voice_recording, LV_OBJ_FLAG_HIDDEN);
  }
}

void update_voice_bar(void) {
  lv_bar_set_value(bar_voice_progress, speech_voice_progress, LV_ANIM_OFF);
}

// autokey screen

static void event_autokey_start_stop(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (speech_autokey_enabled) {
      autokey_stopped();
    }
    else {
      // set countdown to set time
      speech_autokey_countdown_time = lv_spinbox_get_value(spinbox_autokey_txtime) * 60;
      speech_autokey_countdown_interval = lv_spinbox_get_value(spinbox_autokey_interval) * 60;
      speech_autokey_countdown_interval_reload = speech_autokey_countdown_interval;
      // disable all
      lv_spinbox_show_cursor(spinbox_autokey_txtime, false);
      lv_spinbox_show_cursor(spinbox_autokey_interval, false);
      lv_obj_add_state(spinbox_autokey_txtime, LV_STATE_DISABLED);
      lv_obj_add_state(spinbox_autokey_interval, LV_STATE_DISABLED);
      lv_obj_add_state(btn_autokey_txtime_inc, LV_STATE_DISABLED);
      lv_obj_add_state(btn_autokey_txtime_dec, LV_STATE_DISABLED);
      lv_obj_add_state(btn_autokey_interval_inc, LV_STATE_DISABLED);
      lv_obj_add_state(btn_autokey_interval_dec, LV_STATE_DISABLED);
      lv_obj_add_state(rol_autokey_roller_sound, LV_STATE_DISABLED);

      lv_obj_add_state(btn_autokey_set_level, LV_STATE_DISABLED);

      lv_label_set_text(lbl_autokey_start_stop, LV_SYMBOL_STOP);

      // reset end flag
      speech_autokey_stopped = false;
      // show start times
      display_autokey_timers();
      speech_autokey_enabled = true;
      // trigger first transmission
      speech_autokey_trigger = true;
      esp_timer_start_periodic(speech_autokey_timer, 1000000);
    }
  }
}


void autokey_stopped(void) {
  // cancel by flagging completion, this will exit the tasks
  speech_task1_completed = true;
  speech_autokey_enabled = false;
  esp_timer_stop(speech_autokey_timer);
  // enable all
  lv_spinbox_show_cursor(spinbox_autokey_txtime, true);
  lv_spinbox_show_cursor(spinbox_autokey_interval, true);
  lv_obj_clear_state(spinbox_autokey_txtime, LV_STATE_DISABLED);
  lv_obj_clear_state(spinbox_autokey_interval, LV_STATE_DISABLED);
  lv_obj_clear_state(btn_autokey_txtime_inc, LV_STATE_DISABLED);
  lv_obj_clear_state(btn_autokey_txtime_dec, LV_STATE_DISABLED);
  lv_obj_clear_state(btn_autokey_interval_inc, LV_STATE_DISABLED);
  lv_obj_clear_state(btn_autokey_interval_dec, LV_STATE_DISABLED);
  lv_obj_clear_state(rol_autokey_roller_sound, LV_STATE_DISABLED);

  lv_obj_clear_state(btn_autokey_set_level, LV_STATE_DISABLED);


  lv_label_set_text(lbl_autokey_start_stop, LV_SYMBOL_PLAY);

  // reset timers
  speech_autokey_countdown_time = 0;
  speech_autokey_countdown_interval = 0;
  display_autokey_timers();
}

void set_autokey_soundclip_options(void) {
  // Vector to store valid numeric filenames
  std::vector<int> fileNumbers;
  // Open the SD card root directory
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  File root = SD.open("/");
  xSemaphoreGive(spi_mutex_lcd_sd);
  if (!root) {
    Serial.println("Failed to open SD card!");
    return;
  }
  // already add the prerecorded messages
  speech_autokey_sound_count = 10;
  // Read files and collect valid numbers
  while (true) {
    xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
    File file = root.openNextFile();
    xSemaphoreGive(spi_mutex_lcd_sd);
    if (!file) break;  // No more files

    String filename = String(file.name());
    file.close();

    // must be 7 characters long, start with 3 digits, and end in ".bin"
    if (filename.length() == 7 && filename.endsWith(".bin") &&
        isDigit(filename[0]) && isDigit(filename[1]) &&
        isDigit(filename[2])) {
      // Convert numeric part to an integer and store in vector
      int fileNumber = filename.substring(0, 3).toInt();
      fileNumbers.push_back(fileNumber);
      speech_autokey_sound_count++;
    }
  }
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  root.close();
  xSemaphoreGive(spi_mutex_lcd_sd);
  // Sort the collected numbers
  std::sort(fileNumbers.begin(), fileNumbers.end());
  // **Calculate the required buffer size**
  size_t totalSize = 1;  // Start with 1 for the null terminator
  // Add sizes of pre-recorded messages
  for (uint8_t i = 0; i < 5; i++) {
    totalSize = totalSize + speech_voice_msg[i].length() + 2;
  }
  // add morse codes
  for (uint8_t i = 0; i < 5; i++) {
    totalSize = totalSize + speech_morse_msg[i].length() + 2;
  }
  // Add sizes of file numbers (each 3-digit number + newline)
  totalSize += (fileNumbers.size() * 4);  // 3 digits + '\n'
  // char *options = (char *)malloc(totalSize);
  char *options = (char *)heap_caps_malloc(totalSize, MALLOC_CAP_SPIRAM);
  if (!options) {
    Serial.println("Memory allocation failed!");
    return;
  }
  options[0] = '\0';  // Ensure it's an empty string
  for (uint8_t i = 0; i < 5; i++) {
    // Truncate to 9 characters (or fewer if the string is shorter)
    String truncated = speech_voice_msg[i].substring(0, 9);
    strcat(options, truncated.c_str()); // Append truncated C-string
    strcat(options, "\n"); // Add newline
  }
  for (uint8_t i = 0; i < 5; i++) {
    // Truncate to 9 characters (or fewer if the string is shorter)
    String truncated = speech_morse_msg[i].substring(0, 9);
    strcat(options, truncated.c_str()); // Append truncated C-string
    strcat(options, "\n"); // Add newline
  }
  // Build the sorted options string
  for (size_t i = 0; i < fileNumbers.size(); i++) {
    char buffer[6];
    snprintf(buffer, sizeof(buffer), "%03d\n", fileNumbers[i]);  // Format as 3-digit
    strcat(options, buffer);
  }
  // Remove last newline (if any)
  if (strlen(options) > 0) {
    options[strlen(options) - 1] = '\0';
  }
  lv_roller_set_options(rol_autokey_roller_sound, options, LV_ROLLER_MODE_INFINITE);
  // Free dynamically allocated memory
  free(options);
}

void increment_autokey_txtime_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_autokey_txtime);
  }
}

void decrement_autokey_txtime_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_autokey_txtime);
  }
}

void increment_autokey_interval_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_autokey_interval);
  }
}

void decrement_autokey_interval_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_autokey_interval);
  }
}

void display_autokey_timers(void) {
  uint32_t hours = speech_autokey_countdown_time / 3600;
  uint32_t minutes = (speech_autokey_countdown_time % 3600) / 60;
  uint32_t secs = speech_autokey_countdown_time % 60;
  char time_str[16];
  snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u", hours, minutes, secs);
  lv_label_set_text(lbl_autokey_txtime, time_str);
  hours = speech_autokey_countdown_interval / 3600;
  minutes = (speech_autokey_countdown_interval % 3600) / 60;
  secs = speech_autokey_countdown_interval % 60;
  time_str[16];
  snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u", hours, minutes, secs);
  lv_label_set_text(lbl_autokey_interval, time_str);
}

void autokey_play(void) {
  // get selected file name
  uint32_t autokey_selected = lv_roller_get_selected(rol_autokey_roller_sound);
  if (autokey_selected > 4 and autokey_selected < 10) {
    // morse
    speech_morse_play = speech_morse_msg[autokey_selected - 5];
    speech_morse_state = 1;
  }
  else {
    if (autokey_selected < 5) {
      // prerecorded messages
      strncpy(speech_filename, speech_voice_filenames[autokey_selected + 1], sizeof(speech_filename) - 1);
      speech_filename[sizeof(speech_filename) - 1] = '\0';  // Ensure null termination
    }
    else if (autokey_selected > 9 and autokey_selected <= speech_autokey_sound_count - 1) {
      char roller_text[5];  // Buffer to store roller selection (max 3 digits + null terminator)
      // Get the selected string (up to 3 digits) from the LVGL roller
      lv_roller_get_selected_str(rol_autokey_roller_sound, roller_text, sizeof(roller_text));

      // Format the final string with prefix "/" and suffix ".bin"
      snprintf(speech_filename, sizeof(speech_filename), "/%s.bin", roller_text);
    }
    setup_sound_rb_playback();
  }
}

// morse screen

static void event_morse_sel1(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_morse_msg_sel = 1;
      set_morse_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(6, TAB_MORSE_REF,  cb_morse_selection1);
  }
}

static void event_morse_sel2(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_morse_msg_sel = 2;
      set_morse_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(7, TAB_MORSE_REF,  cb_morse_selection2);
  }
}

static void event_morse_sel3(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_morse_msg_sel = 3;
      set_morse_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(8, TAB_MORSE_REF,  cb_morse_selection3);
  }
}

static void event_morse_sel4(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_morse_msg_sel = 4;
      set_morse_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(9, TAB_MORSE_REF,  cb_morse_selection4);
  }
}

static void event_morse_sel5(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_morse_msg_sel = 5;
      set_morse_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(10, TAB_MORSE_REF,  cb_morse_selection5);
  }
}

void set_morse_selection_buttons_state(void) {
  switch (speech_morse_msg_sel) {
    case 1:
      lv_obj_add_state(cb_morse_selection1, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection2, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection3, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection4, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection5, LV_STATE_CHECKED);
      break;

    case 2:
      lv_obj_clear_state(cb_morse_selection1, LV_STATE_CHECKED);
      lv_obj_add_state(cb_morse_selection2, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection3, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection4, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection5, LV_STATE_CHECKED);
      break;

    case 3:
      lv_obj_clear_state(cb_morse_selection1, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection2, LV_STATE_CHECKED);
      lv_obj_add_state(cb_morse_selection3, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection4, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection5, LV_STATE_CHECKED);
      break;

    case 4:
      lv_obj_clear_state(cb_morse_selection1, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection2, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection3, LV_STATE_CHECKED);
      lv_obj_add_state(cb_morse_selection4, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection5, LV_STATE_CHECKED);
      break;

    case 5:
      lv_obj_clear_state(cb_morse_selection1, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection2, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection3, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_morse_selection4, LV_STATE_CHECKED);
      lv_obj_add_state(cb_morse_selection5, LV_STATE_CHECKED);
      break;
  }
}

void set_morse_labels(void) {
  // limit to available space
  speech_morse_msg[0].toCharArray(printbuf, 25);
  lv_checkbox_set_text(cb_morse_selection1, printbuf);
  speech_morse_msg[1].toCharArray(printbuf, 25);
  lv_checkbox_set_text(cb_morse_selection2, printbuf);
  speech_morse_msg[2].toCharArray(printbuf, 25);
  lv_checkbox_set_text(cb_morse_selection3, printbuf);
  speech_morse_msg[3].toCharArray(printbuf, 25);
  lv_checkbox_set_text(cb_morse_selection4, printbuf);
  speech_morse_msg[4].toCharArray(printbuf, 25);
  lv_checkbox_set_text(cb_morse_selection5, printbuf);
}

static void event_morseplay(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    speech_morse_state = 1;
  }
}

// vox screen

static void event_vox_switch(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      // disable eot if on
      if (lv_obj_get_state(cb_eot_on) & LV_STATE_CHECKED) {
        speech_eot_active = false;
        set_eot();
      }
    }
    else {
      if (!speech_input_muted) {
        mute_input(true);
      }
    }
    speech_vox_state = 0;
  }
}

static void event_vox_threshold_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    speech_vox_threshold_slider  = lv_slider_get_value(obj);
  }
}

void vox_set_threshold(void) {
  lv_slider_set_value(slider_vox_threshold, speech_vox_threshold_slider, LV_ANIM_OFF);
}

static void event_vox_hangtime_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    speech_vox_hangtime_slider   = lv_slider_get_value(obj);
  }
}

void vox_set_hangtime(void) {
  lv_slider_set_value(slider_vox_hangtime, speech_vox_hangtime_slider, LV_ANIM_OFF);
}

// test screen

static void event_test_mode0(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_test_mode = 0;
      set_test_mode_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
}

static void event_test_mode1(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_test_mode = 1;
      set_test_mode_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
}

static void event_test_mode2(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_test_mode = 2;
      set_test_mode_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
}

void increment_test_frequency1_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    if (speech_test_enc2_mode != 1 and !speech_test_cursor_override) {
      lv_spinbox_show_cursor(spinbox_test_frequency_1, true);
      lv_spinbox_show_cursor(spinbox_test_frequency_2, false);
      speech_test_enc2_mode = 1;
    }
    lv_spinbox_increment(spinbox_test_frequency_1);
  }
}

static void decrement_test_frequency1_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    if (speech_test_enc2_mode != 1 and !speech_test_cursor_override) {
      lv_spinbox_show_cursor(spinbox_test_frequency_1, true);
      lv_spinbox_show_cursor(spinbox_test_frequency_2, false);
      speech_test_enc2_mode = 1;
    }
    lv_spinbox_decrement(spinbox_test_frequency_1);
  }
}

void increment_test_frequency2_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    if (speech_test_enc2_mode != 2 and !speech_test_cursor_override) {
      lv_spinbox_show_cursor(spinbox_test_frequency_1, false);
      lv_spinbox_show_cursor(spinbox_test_frequency_2, true);
      speech_test_enc2_mode = 2;
    }
    lv_spinbox_increment(spinbox_test_frequency_2);
  }
}

static void decrement_test_frequency2_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    if (speech_test_enc2_mode != 2 and !speech_test_cursor_override) {
      lv_spinbox_show_cursor(spinbox_test_frequency_1, false);
      lv_spinbox_show_cursor(spinbox_test_frequency_2, true);
      speech_test_enc2_mode = 2;
    }
    lv_spinbox_decrement(spinbox_test_frequency_2);
  }
}

static void frequency1_change_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_VALUE_CHANGED) {
    speech_test_frequency1 = lv_spinbox_get_value(spinbox_test_frequency_1);
    set_sine_frequency1(speech_test_frequency1);
  }
  if (code == LV_EVENT_CLICKED) {
    lv_spinbox_show_cursor(spinbox_test_frequency_1, true);
    speech_test_enc2_mode = 1;
    lv_spinbox_show_cursor(spinbox_test_frequency_1, true);
    lv_spinbox_show_cursor(spinbox_test_frequency_2, false);
  }
}

static void frequency2_change_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_VALUE_CHANGED) {
    speech_test_frequency2 = lv_spinbox_get_value(spinbox_test_frequency_2);
    set_sine_frequency2(speech_test_frequency2);
  }
  if (code == LV_EVENT_CLICKED) {
    lv_spinbox_show_cursor(spinbox_test_frequency_2, true);
    speech_test_enc2_mode = 2;
    lv_spinbox_show_cursor(spinbox_test_frequency_1, false);
    lv_spinbox_show_cursor(spinbox_test_frequency_2, true);
  }
}

static void event_test_transmit(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (speech_transmitting) {
      speech_relay_action = 1;
    }
    else {
      speech_relay_action = 2;
    }
  }
}

static void event_test_signal_gain_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED | LV_EVENT_VALUE_CHANGED) {
    int32_t speech_test_signal_gain_previous = speech_test_signal_gain;
    speech_test_signal_gain = lv_slider_get_value(obj);
    if (speech_test_signal_gain != speech_test_signal_gain_previous) {
      set_test_signal_gain();
    }
  }
}

void set_test_mode_buttons_state(void) {
  switch (speech_test_mode) {
    // no signal
    case 0:
      lv_obj_add_state(cb_test_source1, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_test_source2, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_test_source3, LV_STATE_CHECKED);
      lv_obj_add_state(spinbox_test_frequency_1, LV_STATE_DISABLED);
      lv_obj_add_state(spinbox_test_frequency_2, LV_STATE_DISABLED);
      lv_obj_add_state(btn_test_frequency_1_dec, LV_STATE_DISABLED);
      lv_obj_add_state(btn_test_frequency_2_dec, LV_STATE_DISABLED);
      lv_obj_add_state(btn_test_frequency_1_inc, LV_STATE_DISABLED);
      lv_obj_add_state(btn_test_frequency_2_inc, LV_STATE_DISABLED);
      set_sine1_on(false);
      set_sine2_on(false);
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      dsp.mux(MOD_NX1_4_2_MONOSWSLEW_ADDR, 0, 0);
      xSemaphoreGive(i2c_mutex);
      select_output_source(1);
      speech_test_enc2_mode = 0;
      lv_spinbox_show_cursor(spinbox_test_frequency_1, false);
      lv_spinbox_show_cursor(spinbox_test_frequency_2, false);
      break;

    // 1 sine
    case 1:
      lv_obj_add_state(cb_test_source2, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_test_source1, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_test_source3, LV_STATE_CHECKED);
      lv_obj_add_state(spinbox_test_frequency_2, LV_STATE_DISABLED);
      lv_obj_clear_state(spinbox_test_frequency_1, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_test_frequency_1_dec, LV_STATE_DISABLED);
      lv_obj_add_state(btn_test_frequency_2_dec, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_test_frequency_1_inc, LV_STATE_DISABLED);
      lv_obj_add_state(btn_test_frequency_2_inc, LV_STATE_DISABLED);

      set_sine_frequency1(speech_test_frequency1);
      set_sine1_on(true);
      set_sine2_on(false);
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      dsp.mux(MOD_NX1_4_2_MONOSWSLEW_ADDR, 0, 0);
      xSemaphoreGive(i2c_mutex);
      select_output_source(1);
      speech_test_enc2_mode = 1;

      lv_spinbox_show_cursor(spinbox_test_frequency_1, true);
      lv_spinbox_show_cursor(spinbox_test_frequency_2, false);

      break;

    // 2 sine
    case 2:
      lv_obj_add_state(cb_test_source3, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_test_source1, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_test_source2, LV_STATE_CHECKED);

      lv_obj_clear_state(spinbox_test_frequency_1, LV_STATE_DISABLED);
      lv_obj_clear_state(spinbox_test_frequency_2, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_test_frequency_1_dec, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_test_frequency_2_dec, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_test_frequency_1_inc, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_test_frequency_2_inc, LV_STATE_DISABLED);

      set_sine_frequency1(speech_test_frequency1);
      set_sine_frequency2(speech_test_frequency2);

      set_sine1_on(true);
      set_sine2_on(true);
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      dsp.mux(MOD_NX1_4_2_MONOSWSLEW_ADDR, 1, 0);
      xSemaphoreGive(i2c_mutex);
      select_output_source(1);
      speech_test_enc2_mode = 1;

      lv_spinbox_show_cursor(spinbox_test_frequency_1, true);
      lv_spinbox_show_cursor(spinbox_test_frequency_2, false);

      break;
  }
}

// update test signal gain
void update_test_signal_gain(void) {
  lv_slider_set_value(slider_test_signal_gain, speech_test_signal_gain, LV_ANIM_OFF);
  set_test_signal_gain();
}

// update dsp test gain potmeter
void set_test_signal_gain(void) {
  // limit range
  float db_value = -50.0f + (float)speech_test_signal_gain;
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.volume(MOD_SINGLE1_5_GAIN1940ALGNS9_ADDR, db_value);
  xSemaphoreGive(i2c_mutex);
}

// settings screen

void filetransfer_set_buttons(void) {
  //  //speech_filetransfer_state
  //  // 1 = file tx started
  //  // 2 = file tx cancelled
  //  // 3 = completed
  switch (speech_filetransfer_state) {
    case 0:
      lv_obj_clear_state(btn_filetransfer_start, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_filetransfer_cancel, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_filetransfer_back, LV_STATE_DISABLED);
      break;

    case 1:
      lv_obj_add_state(btn_filetransfer_start, LV_STATE_DISABLED);
      lv_obj_clear_state(btn_filetransfer_cancel, LV_STATE_DISABLED);
      lv_obj_add_state(btn_filetransfer_back, LV_STATE_DISABLED);
      break;

    case 2:
      break;

    case 3:
      break;
  }
}

static void event_startfiletransfer(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    // start filetransfer
    speech_filetransfer_state = 1;
    printbuf[0] = '\0';
    xTaskCreatePinnedToCore(
      filetransfertask,         // Task function
      "file_tx",      // Task name
      8192,            // Stack size
      NULL,            // Parameters
      1,               // Priority
      &filetransferTaskHandle,  // Task handle
      0                // Core to pin the task
    );
    speech_filetransfer_statuschange = true;
  }
}

void filetransfertask(void *parameter) {
  while (true) {
    // Check for cancellation
    if (speech_filetransfer_state == 2) {
      speech_filetransfer_state = 3;
      speech_filetransfer_statuschange = true;
      vTaskDelete(NULL);
    }
    // Check for incoming serial data
    if (Serial.available()) {
      // Read file name
      String fileName = Serial.readStringUntil('\n');
      fileName.trim();  // Remove any extra whitespace
      // Create or overwrite the file on the SD card
      xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
      File file = SD.open(fileName, FILE_WRITE);
      xSemaphoreGive(spi_mutex_lcd_sd);
      if (!file) {
        vTaskDelete(NULL);
      }
      // Notify Python script that the ESP32 is ready to receive data
      Serial.println("Ready to receive file. Send data now...");
      // Receive file data
      uint8_t buffer[1024];
      size_t bytesRead;
      size_t totalBytesRead = 0;
      bool transferComplete = false;
      while (!transferComplete) {
        // Check for cancellation
        if (speech_filetransfer_state == 2) {
          xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
          file.close();
          xSemaphoreGive(spi_mutex_lcd_sd);
          speech_filetransfer_state = 3;
          speech_filetransfer_statuschange = true;
          vTaskDelete(NULL);
        }
        // Read data from serial
        if (Serial.available()) {
          bytesRead = Serial.readBytes(buffer, 1024);
          // Check for end-of-transfer marker (0xFFFFFFFF)
          if (bytesRead == 4 &&
              buffer[0] == 0xFF &&
              buffer[1] == 0xFF &&
              buffer[2] == 0xFF &&
              buffer[3] == 0xFF) {
            transferComplete = true;
            break;
          }
          // Write data to file
          xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
          file.write(buffer, bytesRead);
          xSemaphoreGive(spi_mutex_lcd_sd);
          totalBytesRead += bytesRead;
          // Update label with progress
          snprintf(printbuf, sizeof(printbuf), "RX %d b", totalBytesRead);
        }
        // Yield to other tasks
        vTaskDelay(1);
      }
      // Close the file
      xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
      file.close();
      xSemaphoreGive(spi_mutex_lcd_sd);
      snprintf(printbuf, sizeof(printbuf), "Completed %d b", totalBytesRead);
      // Set state to completed
      speech_filetransfer_state = 3;
      speech_filetransfer_statuschange = true;
      vTaskDelete(NULL);
    }
    // Yield to other tasks
    vTaskDelay(10);
  }
}

static void event_cancelfiletransfer(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    // cancel filetransfer
    speech_filetransfer_state = 2;
  }
}

static void event_filedeletescreen(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    set_filedelete_list();
    lv_tabview_set_act(tabview, TAB_DELETE_REF, LV_ANIM_OFF);
  }
}

void set_filedelete_list(void) {
  // Vector to store valid numeric filenames
  std::vector<int> fileNumbers;
  // Open the SD card root directory
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  File root = SD.open("/");
  xSemaphoreGive(spi_mutex_lcd_sd);
  if (!root) {
    Serial.println("Failed to open SD card!");
    return;
  }
  speech_filedelete_count = 0;
  // Read files and collect valid numbers
  while (true) {

    File file = root.openNextFile();

    if (!file) break;  // No more files
    String filename = String(file.name());
    xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
    file.close();
    xSemaphoreGive(spi_mutex_lcd_sd);
    // must be 7 characters long, start with 3 digits, and end in ".bin"
    if (filename.length() == 7 && filename.endsWith(".bin") &&
        isDigit(filename[0]) && isDigit(filename[1]) &&
        isDigit(filename[2])) {
      // Convert numeric part to an integer and store in vector
      int fileNumber = filename.substring(0, 3).toInt();
      fileNumbers.push_back(fileNumber);
      speech_filedelete_count++;
    }
  }
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  root.close();
  xSemaphoreGive(spi_mutex_lcd_sd);
  // Sort the collected numbers
  std::sort(fileNumbers.begin(), fileNumbers.end());
  // **Calculate the required buffer size**
  size_t totalSize = 1;  // Start with 1 for the null terminator
  // Add sizes of file numbers (each 3-digit number + newline)
  totalSize += (fileNumbers.size() * 4);  // 3 digits + '\n'
  // char *options = (char *)malloc(totalSize);
  char *options = (char *)heap_caps_malloc(totalSize, MALLOC_CAP_SPIRAM);
  if (!options) {
    Serial.println("Memory allocation failed!");
    return;
  }
  options[0] = '\0';  // Ensure it's an empty string
  // Build the sorted options string
  for (size_t i = 0; i < fileNumbers.size(); i++) {
    char buffer[6];
    snprintf(buffer, sizeof(buffer), "%03d\n", fileNumbers[i]);  // Format as 3-digit
    strcat(options, buffer);
  }
  // Remove last newline (if any)
  if (strlen(options) > 0) {
    options[strlen(options) - 1] = '\0';
  }
  lv_roller_set_options(rol_filedelete_file, options, LV_ROLLER_MODE_INFINITE);
  speech_filedelete_selected = 0;
  set_filedelete_selected();
  // Free dynamically allocated memory
  free(options);
}

void set_filedelete_selected(void) {
  lv_roller_set_selected(rol_filedelete_file, speech_filedelete_selected, LV_ANIM_OFF);
}

static void event_filedelete_select(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    speech_filedelete_selected = lv_roller_get_selected(obj);
  }
}

// delete selected file
static void event_filedelete(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    uint8_t selected = lv_roller_get_selected(rol_filedelete_file);
    if (selected < speech_filedelete_count and speech_filedelete_count > 0) {
      char roller_text[5];  // Buffer to store roller selection (max 3 digits + null terminator)
      // Get the selected string (up to 3 digits) from the LVGL roller
      lv_roller_get_selected_str(rol_filedelete_file, roller_text, sizeof(roller_text));
      // Format the final string with prefix "/" and suffix ".bin"
      snprintf(speech_filename, sizeof(speech_filename), "/%s.bin", roller_text);
      // Check if file exists
      xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
      if (SD.exists(speech_filename)) {
        if (SD.remove(speech_filename)) {
          //Serial.println("File deleted successfully.");
        } else {
          Serial.println("Failed to delete file.");
        }
      } else {
        Serial.println("File does not exist.");
      }
      xSemaphoreGive(spi_mutex_lcd_sd);
    }
    // reload list
    set_filedelete_list();
    // reload eot sound clip list
    set_eot_soundclip_options();
    // check if out of range
    if (speech_eot_sound_selected > speech_eot_sound_count) {
      speech_eot_sound_selected = 1;
      lv_roller_set_selected(rol_eot_roller_sound, speech_eot_sound_selected - 1, LV_ANIM_OFF);
    }
  }
}

static void event_bypass_switch(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      speech_bypass = true;
    }
    else {
      speech_bypass = false;
    }
    select_output_source(0);
  }
}



// eot settings screen

static void event_eot_level_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED | LV_EVENT_VALUE_CHANGED) {
    int32_t speech_eot_level_previous = speech_eot_level;
    speech_eot_level = lv_slider_get_value(obj);
    if (speech_eot_level != speech_eot_level_previous) {
      set_eot_signal_gain(speech_eot_level);
    }
  }
}

// update eot beep level
void update_eot_level(void) {
  lv_slider_set_value(slider_set_eot_level, speech_eot_level, LV_ANIM_OFF);
  set_eot_signal_gain(speech_eot_level);
}

static void event_eot_mp3_level_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED | LV_EVENT_VALUE_CHANGED) {
    // int32_t speech_eot_mp3_level_previous = speech_eot_mp3_level;
    speech_eot_mp3_level = lv_slider_get_value(obj);
    //    if (speech_eot_mp3_level != speech_eot_mp3_level_previous) {
    //      set_eot_mp3_signal_gain(speech_eot_mp3_level);
    //    }
  }
}

// update eot mp3 level
void update_eot_mp3_level(void) {
  lv_slider_set_value(slider_set_eot_mp3_level, speech_eot_mp3_level, LV_ANIM_OFF);
  // set_eot_mp3_signal_gain(speech_eot_mp3_level);
}

void set_eot_mp3_signal_gain(uint8_t setting) {
  float db_value = -50.0f + (float)setting;
  set_i2s_input_gain(db_value);
}

static void event_ampsetscreen(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_SETAMPLIFIER_REF, LV_ANIM_OFF);
  }
}

static void event_amp_level_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED | LV_EVENT_VALUE_CHANGED) {
    int32_t speech_amplifier_previous = speech_amplifier;
    speech_amplifier = lv_slider_get_value(obj);
    if (speech_amplifier != speech_amplifier_previous) {
      set_input_amp_gain(speech_amplifier);
    }
  }
}

// update amp level
void update_amp_level(void) {
  lv_slider_set_value(slider_set_amplifier, speech_amplifier, LV_ANIM_OFF);
  set_input_amp_gain(speech_amplifier);
}

static void event_vusetscreen(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_SETVURANGE_REF, LV_ANIM_OFF);
  }
}

static void event_vu_level_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED | LV_EVENT_VALUE_CHANGED) {
    speech_vu_scaling = lv_slider_get_value(obj);
  }
}

// update vu level
void update_vu_level(void) {
  lv_slider_set_value(slider_set_vu_range, speech_vu_scaling, LV_ANIM_OFF);
}




// morse level screen

static void event_morse_level_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED | LV_EVENT_VALUE_CHANGED) {
    speech_morse_level = lv_slider_get_value(obj);
  }
}

// update morse beep level
void update_morse_level(void) {
  lv_slider_set_value(slider_set_morse_level, speech_morse_level, LV_ANIM_OFF);
}


// autokey level screen

static void event_autokey_level_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED | LV_EVENT_VALUE_CHANGED) {
    speech_autokey_level = lv_slider_get_value(obj);
  }
}

// update autokey level
void update_autokey_level(void) {
  lv_slider_set_value(slider_set_autokey_level, speech_autokey_level, LV_ANIM_OFF);
}





// general

void select_output_source(uint8_t muxmode) {
  uint8_t parameter = 0;
  switch (muxmode) {
    case 0:
      // normal or bypassed
      if (speech_bypass) {
        parameter = 1;
      }
      else {
        parameter = 2;
      }
      break;

    case 1:
      // test signal
      parameter = 0;
      break;

    case 2:
      // bypass eq comp
      parameter = 3;
      break;

  }
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.mux(MOD_NX1_4_MONOSWSLEW_ADDR, parameter, 0);
  xSemaphoreGive(i2c_mutex);
}

void set_sine_frequency1(uint16_t freq) {
  //  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.sineSource(MOD_STATIC_TONE1_ALG0_MASK_ADDR, freq);
  //  xSemaphoreGive(i2c_mutex);
}

void set_sine_frequency2(uint16_t freq) {
  // xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.sineSource(MOD_STATIC_TONE1_2_ALG0_MASK_ADDR, freq);
  //  xSemaphoreGive(i2c_mutex);
}

void set_sine1_on(boolean switch_on) {
  // no mutex otherwise clicks in sound
  // xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  if (switch_on) {
    dsp.safeload_write(MOD_TONE1_ALG0_ON_ADDR, 0x00800000);
  }
  else {
    dsp.safeload_write(MOD_TONE1_ALG0_ON_ADDR, 0x00000000);
  }
  // xSemaphoreGive(i2c_mutex);
}

void set_sine2_on(boolean switch_on) {
  //  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  if (switch_on) {
    dsp.safeload_write(MOD_TONE1_2_ALG0_ON_ADDR, 0x00800000);
  }
  else {
    dsp.safeload_write(MOD_TONE1_2_ALG0_ON_ADDR, 0x00000000);
  }
  //  xSemaphoreGive(i2c_mutex);
}

// operate tx relay
void transmit(boolean tx) {
  digitalWrite(TX_RELAY, tx);
  speech_transmitting = tx;
  show_tx_led(tx);

}

void  lv_spinbox_show_cursor(lv_obj_t *spinbox, bool en) {
  lv_obj_set_style_text_color(spinbox, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_text_color(spinbox, (en) ? lv_color_white() : lv_color_black(), LV_PART_CURSOR);
  lv_obj_set_style_bg_opa(spinbox, (en) ? 255 : 0, LV_PART_CURSOR);
}

void checkbox_toggle(lv_obj_t *cbox) {
  if (lv_obj_has_state(cbox, LV_STATE_CHECKED)) {
    lv_obj_clear_state(cbox, LV_STATE_CHECKED);  // Uncheck
  } else {
    lv_obj_add_state(cbox, LV_STATE_CHECKED);    // Check
  }
}

// clear a file or create an empty file
void clear_file(uint8_t file_number) {
  // Open the file in write mode (which clears the file)
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  File sdfile = SD.open(speech_voice_filenames[file_number], "w");
  if (sdfile) {
    sdfile.flush();  // This writes a 0 byte file
    sdfile.close();
  }
  xSemaphoreGive(spi_mutex_lcd_sd);
}

// progress bar for recorder fucntions
void add_ruler_to_bar(lv_obj_t *bar, int bar_width, int bar_height,
                      int min, int max, int major_tick_interval,
                      int tick_height, int tick_line_width, int tick_spacing) {
  lv_obj_t *parent = lv_obj_get_parent(bar);
  if (!parent) return;

  int tick_count = (max - min) / major_tick_interval + 1;  // Number of major ticks

  // Create ruler ticks and labels
  for (int i = 0; i < tick_count; i++) {
    int value = min + i * major_tick_interval;
    int x_offset = (bar_width * i) / (tick_count - 1);  // Spread ticks evenly

    // Create tick mark (vertical line)
    lv_obj_t *tick = lv_line_create(parent);
    static lv_point_precise_t tick_points[2];
    tick_points[0].x = 0;
    tick_points[0].y = 0;
    tick_points[1].x = 0;
    tick_points[1].y = tick_height;

    lv_line_set_points(tick, tick_points, 2);
    lv_obj_set_style_line_width(tick, tick_line_width, LV_PART_MAIN);
    lv_obj_set_style_line_color(tick, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_line_opa(tick, LV_OPA_COVER, LV_PART_MAIN);

    // Position tick below the bar
    lv_obj_align_to(tick, bar, LV_ALIGN_BOTTOM_LEFT, x_offset, tick_spacing + (bar_height / 2));

    // Add labels under major ticks
    lv_obj_t *label = lv_label_create(parent);

    if (i == tick_count - 1)
      lv_label_set_text_fmt(label, "%d S", value);
    else
      lv_label_set_text_fmt(label, "%d", value);

    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);

    // Position label under the tick
    lv_obj_align_to(label, tick, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
  }
}

void keyboard_start(uint8_t caller, uint8_t tab, lv_obj_t *obj ) {
  kb_caller = caller;
  kb_returntab = tab;
  kb_text = "";

  if (caller >= 1 and caller <= 16) {
    kb_text = lv_checkbox_get_text(obj);
  }

  if (kb_caller >= 1 and kb_caller <= 5) {
    lv_textarea_set_max_length(edit_txtentry, 15);
  }

  if (kb_caller >= 6 and kb_caller <= 10) {
    lv_textarea_set_max_length(edit_txtentry, 20);
  }

  if (kb_caller >= 11 and kb_caller <= 16) {
    lv_textarea_set_max_length(edit_txtentry, 20);
  }

  kb_text.toCharArray(printbuf, 40);
  lv_textarea_set_text(edit_txtentry, printbuf);
  lv_tabview_set_act(tabview, TAB_EDIT_REF, LV_ANIM_OFF);
}

static void event_keyboardhandler(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);

  if (event == LV_EVENT_READY) {
    String kbtxt = lv_textarea_get_text(edit_txtentry);
    if (kb_caller >= 1 and kb_caller <= 5) {
      speech_voice_msg[kb_caller - 1] = kbtxt;
      save_voice_names();
      set_voice_labels();
      set_eot_soundclip_options();
    }

    if (kb_caller >= 6 and kb_caller <= 10) {
      speech_morse_msg[kb_caller - 6] = kbtxt;
      save_morse_names();
      set_morse_labels();
    }

    if (kb_caller >= 11 and kb_caller <= 16) {
      speech_presets_msg[kb_caller - 11] = kbtxt;
      save_presets_names();
      set_presets_labels();
      set_presetsmain_labels();

    }

    lv_tabview_set_act(tabview, kb_returntab, LV_ANIM_OFF);
  }
  if (event == LV_EVENT_CANCEL) {
    lv_tabview_set_act(tabview, kb_returntab, LV_ANIM_OFF);
  }
}

static void event_calibrate(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    calibratescreen();
    lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
  }
}

void calibratescreen() {
  TFT.fillScreen(TFT_BLACK);
  // clear this namespace
  preferences.begin("settings", false);
  preferences.clear();
  preferences.end();

  TFT.calibrateTouch(speech_setting_calibration_data, TFT_WHITE, TFT_RED, 14);

  //  Serial.println("New calibration data:");
  //  for (int i = 0; i < 8; i++) {
  //    Serial.printf("  %d: %d\n", i, speech_setting_calibration_data[i]);
  //  }

  save_settings();
  TFT.setTouchCalibrate(speech_setting_calibration_data);
}

void save_settings() {
  preferences.begin("settings", false);
  preferences.putBytes("calib", speech_setting_calibration_data, sizeof(speech_setting_calibration_data));
  preferences.end();
}

bool load_settings() {
  preferences.begin("settings", true);
  size_t len = preferences.getBytes("calib", speech_setting_calibration_data, sizeof(speech_setting_calibration_data));
  preferences.end();
  if (len != sizeof(speech_setting_calibration_data)) {
    // Serial.println("No valid calibration data found.");
    return false;
  }
  // check if all 0
  bool allzero = true;
  for (int i = 0; i < 8; i++) {
    if (speech_setting_calibration_data[i] != 0) {
      allzero = false;
      break;
    }
  }
  if (allzero) {
    // Serial.println("Calibration data empty.");
    return false;
  }
  //  Serial.println("Calibration data loaded:");
  //  for (int i = 0; i < 8; i++) {
  //    Serial.printf("  %d: %d\n", i, speech_setting_calibration_data[i]);
  //  }
  return true;
}

// reset all settings and save
void reset_settings() {
  preferences.begin("settings", false); // Open Preferences in read-write mode
  preferences.clear(); // Clear all stored data
  clear_settings();
  save_settings(); // Save defaults
  preferences.end(); // Close Preferences
}

// reset all settings to default
void clear_settings() {
  for (int i = 0; i < 8; i++) {
    speech_setting_calibration_data[i] = 0; // Default value
  }
}

// debug routine in case of problems
// alternative enable monitoring in lvgl
void print_memory_info_to_serial() {
  // Gather memory info
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
  size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
  size_t stack_free = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

  // Print memory information to the serial monitor
  Serial.println("=== Memory Info ===");
  Serial.printf("Heap: %u / %u bytes free\n", (unsigned int)free_heap, (unsigned int)total_heap);

  // If PSRAM is available, display its info
  if (total_psram > 0) {
    Serial.printf("PSRAM: %u / %u bytes free\n", (unsigned int)free_psram, (unsigned int)total_psram);
  } else {
    Serial.println("PSRAM: Not available");
  }

  // Display stack memory info
  Serial.printf("Stack: %u bytes free\n", (unsigned int)stack_free);
  Serial.println("====================");

  // Free DRAM (internal RAM where `dram0_0_seg` is located)
  size_t free_dram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  size_t total_dram = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);

  Serial.printf("Total DRAM: %d bytes\n", total_dram);
  Serial.printf("Free DRAM: %d bytes\n", free_dram);
  Serial.printf("Largest Free Block in DRAM: %d bytes\n", largest_free_block);

  lv_mem_monitor_t mon;
  lv_mem_monitor(&mon);

  Serial.printf("mon.total_size: %d \n", mon.total_size);
  Serial.printf("mon.free_cnt: %d \n", mon.free_cnt);
  Serial.printf("mon.free_size: %d \n", mon.free_size);
  Serial.printf("mon.free_biggest_size: %d \n", mon.free_biggest_size);
  Serial.printf("mon.used_cnt: %d \n", mon.used_cnt);
  Serial.printf("mon.max_used: %d \n", mon.max_used);
  Serial.printf("mon.used_pct: %d \n", mon.used_pct);
  Serial.printf("mon.frag_pct: %d \n", mon.frag_pct);

}

void get_event_name(lv_event_code_t  event) {
  switch (event) {
    case LV_EVENT_ALL:
      Serial.println( "LV_EVENT_ALL");
      break;
    case LV_EVENT_PRESSED:
      Serial.println( "LV_EVENT_PRESSED");
      break;
    case LV_EVENT_PRESSING:
      Serial.println( "LV_EVENT_PRESSING");
      break;
    case LV_EVENT_PRESS_LOST:
      Serial.println( "LV_EVENT_PRESS_LOST");
      break;
    case LV_EVENT_SHORT_CLICKED:
      Serial.println( "LV_EVENT_SHORT_CLICKED");
      break;
    case LV_EVENT_LONG_PRESSED:
      Serial.println( "LV_EVENT_LONG_PRESSED");
      break;
    case LV_EVENT_LONG_PRESSED_REPEAT:
      Serial.println( "LV_EVENT_LONG_PRESSED_REPEAT");
      break;
    case LV_EVENT_CLICKED:
      Serial.println( "LV_EVENT_CLICKED");
      break;
    case LV_EVENT_RELEASED:
      Serial.println( "LV_EVENT_RELEASED");
      break;
    case LV_EVENT_SCROLL_BEGIN:
      Serial.println( "LV_EVENT_SCROLL_BEGIN");
      break;
    case LV_EVENT_SCROLL_THROW_BEGIN:
      Serial.println( "LV_EVENT_SCROLL_THROW_BEGIN");
      break;
    case LV_EVENT_SCROLL_END:
      Serial.println( "LV_EVENT_SCROLL_END");
      break;
    case LV_EVENT_SCROLL:
      Serial.println( "LV_EVENT_SCROLL");
      break;
    case LV_EVENT_GESTURE:
      Serial.println( "LV_EVENT_GESTURE");
      break;
    case LV_EVENT_KEY:
      Serial.println( "LV_EVENT_KEY");
      break;
    case LV_EVENT_ROTARY:
      Serial.println( "LV_EVENT_ROTARY");
      break;
    case LV_EVENT_FOCUSED:
      Serial.println( "LV_EVENT_FOCUSED");
      break;
    case LV_EVENT_DEFOCUSED:
      Serial.println( "LV_EVENT_DEFOCUSED");
      break;
    case LV_EVENT_LEAVE:
      Serial.println( "LV_EVENT_LEAVE");
      break;
    case LV_EVENT_HIT_TEST:
      Serial.println( "LV_EVENT_HIT_TEST");
      break;
    case LV_EVENT_INDEV_RESET:
      Serial.println( "LV_EVENT_INDEV_RESET");
      break;
    case LV_EVENT_HOVER_OVER:
      Serial.println( "LV_EVENT_HOVER_OVER");
      break;
    case LV_EVENT_HOVER_LEAVE:
      Serial.println( "LV_EVENT_HOVER_LEAVE");
      break;
    case LV_EVENT_COVER_CHECK:
      Serial.println( "LV_EVENT_COVER_CHECK");
      break;
    case LV_EVENT_REFR_EXT_DRAW_SIZE:
      Serial.println( "LV_EVENT_REFR_EXT_DRAW_SIZE");
      break;
    case LV_EVENT_DRAW_MAIN_BEGIN:
      Serial.println( "LV_EVENT_DRAW_MAIN_BEGIN");
      break;
    case LV_EVENT_DRAW_MAIN:
      Serial.println( "LV_EVENT_DRAW_MAIN");
      break;
    case LV_EVENT_DRAW_MAIN_END:
      Serial.println( "LV_EVENT_DRAW_MAIN_END");
      break;
    case LV_EVENT_DRAW_POST_BEGIN:
      Serial.println( "LV_EVENT_DRAW_POST_BEGIN");
      break;
    case LV_EVENT_DRAW_POST:
      Serial.println( "LV_EVENT_DRAW_POST");
      break;
    case LV_EVENT_DRAW_POST_END:
      Serial.println( "LV_EVENT_DRAW_POST_END");
      break;
    case LV_EVENT_DRAW_TASK_ADDED:
      Serial.println( "LV_EVENT_DRAW_TASK_ADDED");
      break;
    case LV_EVENT_VALUE_CHANGED:
      Serial.println( "LV_EVENT_VALUE_CHANGED");
      break;
    case LV_EVENT_INSERT:
      Serial.println( "LV_EVENT_INSERT");
      break;
    case LV_EVENT_REFRESH:
      Serial.println( "LV_EVENT_REFRESH");
      break;
    case LV_EVENT_READY:
      Serial.println( "LV_EVENT_READY");
      break;
    case LV_EVENT_CANCEL:
      Serial.println( "LV_EVENT_CANCEL");
      break;
    case LV_EVENT_CREATE:
      Serial.println( "LV_EVENT_CREATE");
      break;
    case LV_EVENT_DELETE:
      Serial.println( "LV_EVENT_DELETE");
      break;
    case LV_EVENT_CHILD_CHANGED:
      Serial.println( "LV_EVENT_CHILD_CHANGED");
      break;
    case LV_EVENT_CHILD_CREATED:
      Serial.println( "LV_EVENT_CHILD_CREATED");
      break;
    case LV_EVENT_CHILD_DELETED:
      Serial.println( "LV_EVENT_CHILD_DELETED");
      break;
    case LV_EVENT_SCREEN_UNLOAD_START:
      Serial.println( "LV_EVENT_SCREEN_UNLOAD_START");
      break;
    case LV_EVENT_SCREEN_LOAD_START:
      Serial.println( "LV_EVENT_SCREEN_LOAD_START");
      break;
    case LV_EVENT_SCREEN_LOADED:
      Serial.println( "LV_EVENT_SCREEN_LOADED");
      break;
    case LV_EVENT_SCREEN_UNLOADED:
      Serial.println( "LV_EVENT_SCREEN_UNLOADED");
      break;
    case LV_EVENT_SIZE_CHANGED:
      Serial.println( "LV_EVENT_SIZE_CHANGED");
      break;
    case LV_EVENT_STYLE_CHANGED:
      Serial.println( "LV_EVENT_STYLE_CHANGED");
      break;
    case LV_EVENT_LAYOUT_CHANGED:
      Serial.println( "LV_EVENT_LAYOUT_CHANGED");
      break;
    case LV_EVENT_GET_SELF_SIZE:
      Serial.println( "LV_EVENT_GET_SELF_SIZE");
      break;
    case LV_EVENT_INVALIDATE_AREA:
      Serial.println( "LV_EVENT_INVALIDATE_AREA");
      break;
    case LV_EVENT_RESOLUTION_CHANGED:
      Serial.println( "LV_EVENT_RESOLUTION_CHANGED");
      break;
    case LV_EVENT_COLOR_FORMAT_CHANGED:
      Serial.println( "LV_EVENT_COLOR_FORMAT_CHANGED");
      break;
    case LV_EVENT_VSYNC:
      Serial.println( "LV_EVENT_VSYNC");
      break;
    default:
      Serial.println( "UNKNOWN_EVENT");
      break;
  }
}

// read audio stream from i2s and save to alternating buffers
void recordAudioTask(void *parameter) {
  // should be cleared before but just in case
  speech_buffer_done = 0;

  // Serial.println("Recording audio...");

  uint32_t bcount = 0;
  // size_t speech_bytes_read;
  uint8_t buffer_number = 0;
  // dummy read and discard to remove ptt click
  xSemaphoreTake(i2s_mutex, portMAX_DELAY);
  for (uint8_t dummy_read = 0; dummy_read < 10; dummy_read++) {
    i2s_read(I2S_NUM, (void*)i2s_buffer, sizeof(i2s_buffer), &speech_bytes_read, portMAX_DELAY);
  }
  xSemaphoreGive(i2s_mutex);
  // start record timer
  speech_rec_pb_timer = millis();
  for (uint16_t outerloop = 0; outerloop < 300; outerloop++) {

    // loop 4 times to get 1024 32 bit values = 6144 bytes * 4 = 24576 or 1 buffer
    xSemaphoreTake(i2s_mutex, portMAX_DELAY);
    for (uint8_t innerloop = 0; innerloop < 6; innerloop++) {
      // Read I2S data
      i2s_read(I2S_NUM, (void*)i2s_buffer, sizeof(i2s_buffer), &speech_bytes_read, portMAX_DELAY);
      if (speech_bytes_read == 4096) {
        if (buffer_number == 0) {
          memcpy(&rec_pb_buffer0[innerloop * 1024], i2s_buffer, sizeof(i2s_buffer));
        }
        else {
          memcpy(&rec_pb_buffer1[innerloop * 1024], i2s_buffer, sizeof(i2s_buffer));
        }
      }
    }
    xSemaphoreGive(i2s_mutex);
    // pass completed buffer 0= none 1=0 2=1
    speech_buffer_done = buffer_number + 1;
    // next buffer
    buffer_number++;
    // loop back
    if (buffer_number > 1) {
      buffer_number = 0;
    }
    bcount++;
    // exit early if ptt released
    if (speech_rec_pb_state != 2 or !speech_recording_playback_active) {
      //  Serial.println("writing ended by trigger record");
      break;
    }
    // check for time out
    if (millis() - speech_rec_pb_timer >= 20000) {
      //print_task_status();
      // Serial.println("writing ended by time out");
      break;
    }
  }

  // Serial.println("Recording done...");

  // last one completed cleans up
  if (speech_task2_completed) {
    cleanup_recording();
  }
  speech_task1_completed = true;
  vTaskDelete(NULL); // End this task after recording
}

// write audio to file alternating from 2 buffers
void writeAudioTask(void *parameter) {
  uint16_t buffer_counter = 0;
  uint8_t buffer_old = 0;
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  File sdfile = SD.open(speech_voice_filenames[speech_selected_file], "w");
  xSemaphoreGive(spi_mutex_lcd_sd);
  if (!sdfile) {
    Serial.println("Failed to open file for writing");
    vTaskDelete(NULL);
  }
  // wait until at least one buffer is filled
  while (speech_buffer_done == 0) {
    vTaskDelay(1);
  }
  // Serial.println("started writing");
  buffer_counter = 0;
  // trigger first change
  buffer_old = 99;
  // cycles of 16K buffers
  while (buffer_counter < 300) {
    if (speech_buffer_done != buffer_old) {
      // keep current buffer
      buffer_old = speech_buffer_done;
      xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
      if (buffer_old == 1) {
        sdfile.write((uint8_t*)rec_pb_buffer0, 24576);
      }
      else {
        sdfile.write((uint8_t*)rec_pb_buffer1, 24576);
      }
      xSemaphoreGive(spi_mutex_lcd_sd);
      buffer_counter++;
    }
    // exit
    if (speech_rec_pb_state != 2 or !speech_recording_playback_active) {
      // note since the writing task is faster than the sampling task this will trigger first
      // the last sample buffer will not be saved but thats good, it's interrupted anyway
      break;
    }
    // stop if first task is completed
    if (speech_task1_completed) {
      // Serial.println("writing ended by first task done");
      // Serial.println(buffer_counter);
      break;
    }
  }
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  sdfile.flush();
  sdfile.close();
  xSemaphoreGive(spi_mutex_lcd_sd);
  // Serial.println("done writing to SD card.");
  speech_record_complete = true;
  // last one completed cleans up
  if (speech_task1_completed) {
    cleanup_recording();
  }
  speech_task2_completed = true;
  vTaskDelete(NULL); // End this task after recording
}

// read back from file and put into buffers
void readAudioTask(void *parameter) {
  // ca 60ms loops
  uint8_t buffer_counter = 0;
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  File sdfile2 = SD.open(speech_voice_filenames[speech_selected_file], "r");
  xSemaphoreGive(spi_mutex_lcd_sd);
  if (sdfile2) {
    //  Serial.print("start read file  ");
    //  Serial.println(speech_voice_filenames[speech_selected_file]);
    // determine number of blocks
    speech_file_blocks = (sdfile2.size() / 24576);
    // we have some buffers to read
    if (speech_file_blocks > 2) {
      // start playback timer
      speech_rec_pb_timer = millis();
      while (sdfile2.available() and !speech_task2_completed) {
        // wait for flag
        if (speech_buffer_next) {
          if (buffer_counter == 0) {
            sdfile2.read((uint8_t*)rec_pb_buffer0, 24576);
            speech_buffer_done = 1;
          }
          else {
            sdfile2.read((uint8_t*)rec_pb_buffer1, 24576);
            speech_buffer_done = 2;
          }
          // reset flag
          speech_buffer_next = false;
          // toggle buffer
          buffer_counter ^= 1;
        }
        else {
          // delay to avoid watchdog trigger
          vTaskDelay(1);
        }
      }
    }
  }
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  sdfile2.close();
  xSemaphoreGive(spi_mutex_lcd_sd);
  // Serial.println("done read file");
  // last one completed cleans up
  if (speech_task2_completed) {
    cleanup_playback();
  }
  speech_task1_completed = true;
  vTaskDelete(NULL); // End this task after recording
}

void playAudioTask(void *parameter) {
  // ca 127ms loops
  // block loop counter
  uint32_t outerloop = 0;
  // copy the buffer pointer because it changes during the processing once speech_buffer_next is changed
  uint8_t keep_buffer = 0;
  // keep to see if buffer has changed
  uint8_t old_buffer = 99;
  // flag for next buffer
  speech_buffer_next = true;
  // Serial.println("wait buffer rdy");
  // wait until a buffer is ready or task 1 reports as completed
  while (speech_buffer_done  == 0 and !speech_task1_completed) {
    vTaskDelay(1);
  }
  // Serial.println("start play");
  // number of speech_file_blocks is determined from the file size
  while (outerloop < speech_file_blocks and !speech_task1_completed) {
    // copy pointer
    keep_buffer = speech_buffer_done;
    if ( speech_buffer_done != old_buffer) {
      // ask for next buffer to be prepared
      speech_buffer_next = true;
      // loop 6 times to get 1024 32 bit values = 4096 bytes * 6 = 24576 or 1 buffer
      for (uint8_t innerloop = 0; innerloop < 6; innerloop++) {
        if (keep_buffer == 1) {
          memcpy(i2s_buffer, &rec_pb_buffer0[innerloop * 1024], sizeof(i2s_buffer));
        }
        else {
          memcpy(i2s_buffer, &rec_pb_buffer1[innerloop * 1024], sizeof(i2s_buffer));
        }
        xSemaphoreTake(i2s_mutex, portMAX_DELAY);
        i2s_write(I2S_NUM,  i2s_buffer, sizeof(i2s_buffer), &speech_bytes_written, portMAX_DELAY);
        xSemaphoreGive(i2s_mutex);
      }
      outerloop++;
      old_buffer = keep_buffer;
    }
    else {
      vTaskDelay(1);
    }
  }
  // Serial.println("play done");
  // last one completed cleans up
  if (speech_task1_completed) {
    cleanup_playback();
  }
  speech_task2_completed = true;
  vTaskDelete(NULL); // End this task after recording
}

void setup_recording(void) {
  speech_buffer_done = 0;
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.mux(MOD_NX1_3_2_MONOSWSLEW_ADDR, 2, 0);
  xSemaphoreGive(i2c_mutex);
  // delay for slew otherwise feedback
  vTaskDelay(pdMS_TO_TICKS(200));
  speech_task1_completed = false;
  speech_task2_completed = false;
  xTaskCreatePinnedToCore(recordAudioTask, "Record Audio Task", 8192, NULL, 1, &recordTaskHandle, 1);
  xTaskCreatePinnedToCore(writeAudioTask, "Write Audio Task", 8192, NULL, 1, &writeTaskHandle, 0);
}

void cleanup_recording(void) {
  // nothing for now
}

void setup_playback(void) {
  // set gain for i2s input, compensate for amplifier
  set_i2s_input_gain(-1.58);
  // check file
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  File sdfile = SD.open(speech_voice_filenames[speech_selected_file], "r");
  xSemaphoreGive(spi_mutex_lcd_sd);
  if (sdfile) {
    xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
    sdfile.close();
    xSemaphoreGive(spi_mutex_lcd_sd);
    // set bypass only for recording screen
    if ((lv_obj_get_state(cb_recorder_bypass_on) & LV_STATE_CHECKED) and lv_tabview_get_tab_active(tabview) == TAB_RECORDER_REF) {
      select_output_source(2);
    }
    else {
      select_output_source(0);
    }
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.mux(MOD_NX1_3_MONOSWSLEW_ADDR, 1, 0);
    xSemaphoreGive(i2c_mutex);
    speech_buffer_done = 0;
    speech_buffer_next = false;
    speech_task1_completed = false;
    speech_task2_completed = false;
    vTaskDelay(pdMS_TO_TICKS(200));
    if (lv_tabview_get_tab_active(tabview) == TAB_RECORDER_REF) {
      if (lv_obj_get_state(cb_recorder_tx_on) & LV_STATE_CHECKED) {
        transmit(true);
      }
    }
    else {
      if (lv_tabview_get_tab_active(tabview) == TAB_VOICE_REF) {
        if (lv_obj_get_state(cb_voice_tx_on) & LV_STATE_CHECKED) {
          transmit(true);
        }
      }
    }
    // this combination of cores seems to give the best performance
    xTaskCreatePinnedToCore(readAudioTask, "Read Audio Task", 8192, NULL, 1, &readTaskHandle, 1);
    xTaskCreatePinnedToCore(playAudioTask, "Play Audio Task", 8192, NULL, 1, &playTaskHandle, 0);
  }
  else {
    Serial.println("failed to open file");
    speech_rec_pb_state = 6;
  }
}

void cleanup_playback(void) {
  // Serial.println("cleanup_playback");
  if (lv_tabview_get_tab_active(tabview) == TAB_RECORDER_REF) {
    if (lv_obj_get_state(cb_recorder_tx_on) & LV_STATE_CHECKED) {
      transmit(false);
    }
  }
  else {
    if (lv_tabview_get_tab_active(tabview) == TAB_VOICE_REF) {
      if (lv_obj_get_state(cb_voice_tx_on) & LV_STATE_CHECKED) {
        transmit(false);
      }
    }
  }
  // alow for dma buffer to empty
  vTaskDelay(pdMS_TO_TICKS(150));
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.mux(MOD_NX1_3_MONOSWSLEW_ADDR, 0, 0);
  xSemaphoreGive(i2c_mutex);
  // reset to default
  set_effects_echo_fb();
  // bypass off
  if ((lv_obj_get_state(cb_recorder_bypass_on) & LV_STATE_CHECKED) and lv_tabview_get_tab_active(tabview) == TAB_RECORDER_REF) {
    select_output_source(0);
  }
  // add delay to settle slew
  vTaskDelay(pdMS_TO_TICKS(300));
  speech_rec_pb_state = 6;
}

// read back from file and put into buffers
void readEotAudioTask(void *parameter) {
  // ca 60ms loops
  uint8_t buffer_counter = 0;
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  File sdfile2 = SD.open(speech_filename, "r");
  xSemaphoreGive(spi_mutex_lcd_sd);
  if (sdfile2) {
    // determine number of blocks
    speech_file_blocks = (sdfile2.size() / 24576);
    // we have some buffers to read
    if (speech_file_blocks > 2) {
      // start playback timer
      speech_rec_pb_timer = millis();
      while (sdfile2.available() and !speech_task2_completed) {
        // wait for flag
        if (speech_buffer_next) {
          xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
          if (buffer_counter == 0) {
            sdfile2.read((uint8_t*)rec_pb_buffer0, 24576);
            speech_buffer_done = 1;
          }
          else {
            sdfile2.read((uint8_t*)rec_pb_buffer1, 24576);
            speech_buffer_done = 2;
          }
          xSemaphoreGive(spi_mutex_lcd_sd);
          // reset flag
          speech_buffer_next = false;
          // toggle buffer
          buffer_counter ^= 1;
        }
        else {
          // delay to avoid watchdog trigger
          vTaskDelay(1);
        }
      }
    }
  }
  xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
  sdfile2.close();
  xSemaphoreGive(spi_mutex_lcd_sd);
  // Serial.println("done read file");
  // last one completed cleans up
  if (speech_task2_completed) {
    cleanup_sound_rb_playback();
  }
  speech_task1_completed = true;
  vTaskDelete(NULL); // End this task after recording
}

void playEotAudioTask(void *parameter) {
  // ca 127ms loops
  // block loop counter
  uint32_t outerloop = 0;
  // copy the buffer pointer because it changes during the processing once speech_buffer_next is changed
  uint8_t keep_buffer = 0;
  // keep to see if buffer has changed
  uint8_t old_buffer = 99;
  // flag for next buffer
  speech_buffer_next = true;
  // wait until a buffer is ready or task 1 reports as completed
  while (speech_buffer_done  == 0 and !speech_task1_completed) {
    vTaskDelay(1);
  }
  // number of speech_file_blocks is determined from the file size
  while (outerloop < speech_file_blocks and !speech_task1_completed) {
    // copy pointer
    keep_buffer = speech_buffer_done;
    if ( speech_buffer_done != old_buffer) {
      // ask for next buffer to be prepared
      speech_buffer_next = true;
      // loop 6 times to get 1024 32 bit values = 4096 bytes * 6 = 24576 or 1 buffer
      for (uint8_t innerloop = 0; innerloop < 6; innerloop++) {
        if (keep_buffer == 1) {
          memcpy(i2s_buffer, &rec_pb_buffer0[innerloop * 1024], sizeof(i2s_buffer));
        }
        else {
          memcpy(i2s_buffer, &rec_pb_buffer1[innerloop * 1024], sizeof(i2s_buffer));
        }
        xSemaphoreTake(i2s_mutex, portMAX_DELAY);
        i2s_write(I2S_NUM,  i2s_buffer, sizeof(i2s_buffer), &speech_bytes_written, portMAX_DELAY);
        xSemaphoreGive(i2s_mutex);
      }
      outerloop++;
      old_buffer = keep_buffer;
    }
    else {
      vTaskDelay(1);
    }
  }
  // last one completed cleans up
  if (speech_task1_completed) {
    cleanup_sound_rb_playback();
  }
  speech_task2_completed = true;
  vTaskDelete(NULL); // End this task after recording
}

void setup_sound_rb_playback(void) {

  //  set_esp32_amp_gain(12);

  if (speech_autokey_enabled) {
    set_eot_mp3_signal_gain(speech_autokey_level);
  }
  else {
    set_eot_mp3_signal_gain(speech_eot_mp3_level);
  }

  const int maxRetries = 5;
  const int retryDelayMs = 100;
  File sdfile;
  bool fileOpened = false;

  // probeer meerdere keren te openen
  for (int attempt = 1; attempt <= maxRetries; attempt++) {
    xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
    sdfile = SD.open(speech_filename, "r");
    xSemaphoreGive(spi_mutex_lcd_sd);
    if (sdfile) {
      fileOpened = true;
      break;
    } else {
      Serial.printf("Attempt %d to open %s failed\n", attempt, speech_filename);
      vTaskDelay(pdMS_TO_TICKS(retryDelayMs));
    }
  }

  if (fileOpened) {
    xSemaphoreTake(spi_mutex_lcd_sd, portMAX_DELAY);
    sdfile.close();
    xSemaphoreGive(spi_mutex_lcd_sd);
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.mux(MOD_NX1_3_MONOSWSLEW_ADDR, 1, 0);
    xSemaphoreGive(i2c_mutex);
    // this will bypass the eq and compressor
    //  select_output_source(2);
    select_output_source(0);

    speech_buffer_done = 0;
    speech_buffer_next = false;
    speech_task1_completed = false;
    speech_task2_completed = false;

    vTaskDelay(pdMS_TO_TICKS(200));

    xTaskCreatePinnedToCore(readEotAudioTask, "Read Audio Task", 8192, NULL, 1, &readEotSoundTaskHandle, 1);
    xTaskCreatePinnedToCore(playEotAudioTask, "Play Audio Task", 8192, NULL, 1, &playEotSoundTaskHandle, 0);
  }
  else {
    Serial.print("Unable to open file after ");
    Serial.print(maxRetries);
    Serial.print(" retries: ");
    Serial.println(speech_filename);

    cleanup_sound_rb_playback();
  }
}

void cleanup_sound_rb_playback(void) {
  // allow for dma buffer to empty
  vTaskDelay(pdMS_TO_TICKS(150));
  set_effects_echo_fb();

  // switch back
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  dsp.mux(MOD_NX1_3_MONOSWSLEW_ADDR, 0, 0);
  xSemaphoreGive(i2c_mutex);

  select_output_source(0);

  // set_esp32_amp_gain(1.2);

  // add delay to settle slew
  vTaskDelay(pdMS_TO_TICKS(300));

  // reset lock
  speech_lock = false;
  // this will trigger the release of the transmit relay
  speech_relay_action = 1;
  speech_eot_state = 3;

}

// play eot
void eot_task(void *parameter) {
  while (true) {
    // wait for task trigger
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for trigger
    // mute rest of the processor to avoid noise since the input is open when ptt is released
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.mute(MOD_MUTE3_2_MUTENOSLEWALG1MUTE_ADDR, 0);
    // if eot is not active unmute the tome source
    if (!speech_eot_active) {
      dsp.mute(MOD_MUTE3_3_MUTENOSLEWALG2MUTE_ADDR, 1);
    }
    xSemaphoreGive(i2c_mutex);
    // handle separate to avoid mutex double lock
    if (!speech_eot_active) {
      set_eot_signal_gain(speech_eot_level);
    }
    switch (speech_eot_beep_selected) {
      case 1:
        set_sine_frequency1(1700);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(200));
        break;

      case 2:
        set_sine_frequency1(2360);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(105));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(30));
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(190));
        break;

      case 3:
        set_sine_frequency1(645);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(145));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(45));
        set_sine_frequency1(500);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(50));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(45));
        set_sine_frequency1(580);
        vTaskDelay(pdMS_TO_TICKS(155));
        break;

      case 4:
        set_sine_frequency1(1295);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(145));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(45));
        set_sine_frequency1(900);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(50));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(45));
        set_sine_frequency1(1460);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(155));
        break;

      case 5:
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(150));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(45));
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(45));
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(145));
        break;

      case 6:
        set_sine_frequency1(900);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(50));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(45));
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(155));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(45));
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(50));
        break;

      case 7:
        set_sine_frequency1(400);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        set_sine_frequency1(730);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(180));
        break;

      case 8:
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(75));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine_frequency1(670);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(185));
        break;

      case 9:
        set_sine_frequency1(1460);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(120));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine1_on(false);
        set_sine_frequency1(1060);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(120));
        break;

      case 10:
        set_sine_frequency1(1935);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(45));
        set_sine_frequency1(1460);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(185));
        break;

      case 11:
        set_sine_frequency1(280);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(280));
        set_sine1_on(false);
        set_sine_frequency1(645);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine1_on(false);
        set_sine_frequency1(900);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(75));
        set_sine1_on(false);
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        break;

      case 12:
        set_sine_frequency1(1294);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(30));
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(50));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(30));
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(75));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(70));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(70));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(970);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(50));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(30));
        set_sine_frequency1(1058);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(30));
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(1295);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(70));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(85));
        set_sine_frequency1(1290);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(75));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine_frequency1(1295);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(75));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine_frequency1(1060);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(60));
        break;

      case 13:
        set_sine_frequency1(2325);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(1455);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(500);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(355);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(300);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(1660);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        break;

      case 14:
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(185));
        set_sine1_on(false);
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(185));
        break;

      case 15:
        set_sine_frequency1(2325);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(35));
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(30));
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(200));
        break;

      case 16:
        set_sine_frequency1(265);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        set_sine_frequency1(414);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(120));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(30));
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(150));
        break;

      case 17:
        set_sine_frequency1(484);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine_frequency1(528);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine_frequency1(551);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine_frequency1(896);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(160));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(85));
        set_sine_frequency1(729);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(125));
        break;

      case 18:
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(40));
        set_sine_frequency1(1295);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(40));
        set_sine_frequency1(897);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(50));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(40));
        set_sine_frequency1(1295);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(50));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(40));
        set_sine_frequency1(1456);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(70));
        break;

      case 19:
        set_sine_frequency1(897);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(125));
        set_sine1_on(false);
        set_sine_frequency1(2325);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(155));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine_frequency1(897);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(200));
        set_sine1_on(false);
        set_sine_frequency1(357);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        break;

      case 20:
        set_sine_frequency1(685);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine_frequency1(730);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine_frequency1(779);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(85));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(1060);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(195));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine_frequency1(779);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(125));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(1060);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(200));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine_frequency1(779);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(125));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(1060);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(200));
        break;

      case 21:
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(897);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(135));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(645);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(135));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(105));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(897);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(135));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(65));
        set_sine_frequency1(645);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(135));
        break;

      case 22:
        set_sine_frequency1(645);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(580);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(779);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(225));
        break;

      case 23:
        set_sine_frequency1(1060);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine1_on(false);
        set_sine_frequency1(1295);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        set_sine_frequency1(1457);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(120));
        break;

      case 24:
        set_sine_frequency1(645);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(55));
        set_sine1_on(false);
        set_sine_frequency1(733);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(74));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(3));
        set_sine_frequency1(897);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(85));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(200));
        break;

      case 25:
        set_sine_frequency1(320);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(358);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(400);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        set_sine_frequency1(423);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(210));
        break;

      case 26:
        set_sine_frequency1(1060);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine1_on(false);
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(20));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(1060);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine1_on(false);
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(20));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(1060);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine1_on(false);
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(20));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(1060);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine1_on(false);
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(20));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(1060);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(60));
        set_sine1_on(false);
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(145));
        break;

      case 27:
        set_sine_frequency1(2326);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(1456);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(504);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(358);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(309);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(145));
        break;

      case 28:
        set_sine_frequency1(400);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(200));
        set_sine1_on(false);
        break;

      case 29:
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(1456);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        set_sine_frequency1(1456);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        break;

      case 30:
        set_sine_frequency1(310);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(50));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(357);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(504);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(830);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(1160);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(1456);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_sine_frequency1(2326);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(145));
        break;

      case 31:
        set_sine_frequency1(523);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(70));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(50));
        set_sine_frequency1(440);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(70));
        set_sine1_on(false);
        set_sine_frequency1(523);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(70));
        set_sine1_on(false);
        vTaskDelay(pdMS_TO_TICKS(50));
        set_sine_frequency1(440);
        set_sine1_on(true);
        vTaskDelay(pdMS_TO_TICKS(70));
        break;

      case 32: // Morse 0 → "-----"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between dashes
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between dashes
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between dashes
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between dashes
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 33: // Morse 1 → ".----"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between dashes
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between dashes
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between dashes
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 34: // Morse 2 → "..---"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between dashes
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between dashes
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 35: // Morse 3 → "...--"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between dashes
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 36: // Morse 4 → "....-"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 37: // Morse 5 → "....."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 38: // Morse 6 → "-...."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 39: // Morse 7 → "--..."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 40: // Morse 8 → "---.."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 41: // Morse 9 → "----."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 42: // Morse A → ".-"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 43: // Morse B → "-..."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 44: // Morse C → "-.-."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 45: // Morse D → "-.."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 46: // Morse E → "."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 47: // Morse F → "..-."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 48: // Morse G → "--."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 49: // Morse H → "...."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 50: // Morse I → ".."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 51: // Morse J → ".---"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 52: // Morse K → "-.-"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 53: // Morse L → ".-.."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 54: // Morse M → "--"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 55: // Morse N → "-."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 56: // Morse O → "---"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 57: // Morse P → ".--."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 58: // Morse Q → "--.-"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 59: // Morse R → ".-."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 60: // Morse S → "..."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;

      case 61: // Morse T → "-"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 62: // Morse U → "..-"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 63: // Morse V → "...-"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 64: // Morse W → ".--"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 65: // Morse X → "-..-"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 66: // Morse Y → "-.--"
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        break;

      case 67: // Morse Z → "--.."
        set_sine_frequency1(900);
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dash
        vTaskDelay(pdMS_TO_TICKS(150)); // 150ms for dash
        set_sine1_on(false);         // Turn off after dash
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        set_sine1_on(false);         // Turn off after dot
        vTaskDelay(pdMS_TO_TICKS(50));  // Space between symbols
        set_sine1_on(true);           // Dot
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms for dot
        break;




    }
    // turn off the sine generator
    set_sine1_on(false);
    // reset lock
    speech_lock = false;
    // this will trigger the release of the transmit relay
    speech_relay_action = 1;
    // connect the compressor back to the output
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.mute(MOD_MUTE3_2_MUTENOSLEWALG1MUTE_ADDR, 1);
    // if eot is not active mute the tome source
    if (!speech_eot_active) {
      dsp.mute(MOD_MUTE3_3_MUTENOSLEWALG2MUTE_ADDR, 0);
    }
    xSemaphoreGive(i2c_mutex);
    // if eot is not active mute the tome source
    if (!speech_eot_active) {
      set_eot_signal_gain(0);
    }
    // reset state
    speech_eot_state = 3;
  }
}


// play morse code string
void morse_task(void *parameter) {
  const int DOT = 50;
  const int DASH = 3 * DOT;
  const int INTER_ELEMENT = DOT;
  const int INTER_LETTER = 3 * DOT;
  const int INTER_WORD = 7 * DOT;

  const char* morseTable[36] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---",
    "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-",
    "..-", "...-", ".--", "-..-", "-.--", "--..",
    "-----", ".----", "..---", "...--", "....-", ".....",
    "-....", "--...", "---..", "----."
  };
  while (true) {
    // wait for task trigger
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for trigger

    if (speech_autokey_enabled) {
      set_eot_signal_gain(speech_autokey_level);
    }
    else {
      set_eot_signal_gain(speech_morse_level);
    }

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    // mute rest of the processor to avoid noise since the input is open when ptt is released
    dsp.mute(MOD_MUTE3_2_MUTENOSLEWALG1MUTE_ADDR, 0);

    // output sine gens
    dsp.mute(MOD_MUTE3_3_MUTENOSLEWALG2MUTE_ADDR, 1);
    xSemaphoreGive(i2c_mutex);

    int inputLength = speech_morse_play.length();
    for (int i = 0; i < 40 && speech_morse_play[i] != '\0'; ++i) {
      char c = toupper(speech_morse_play[i]);

      if (c == ' ') {
        // Only send word space if not at end of message
        if (i < inputLength - 1) {
          morse_tone(0, INTER_WORD);
        }
        continue;
      }

      const char* code = nullptr;
      if (c >= 'A' && c <= 'Z') {
        code = morseTable[c - 'A'];
      } else if (c >= '0' && c <= '9') {
        code = morseTable[26 + (c - '0')];
      }

      if (code) {
        for (int j = 0; code[j] != '\0'; ++j) {
          if (code[j] == '.') {
            morse_tone(DOT, INTER_ELEMENT);
          } else if (code[j] == '-') {
            morse_tone(DASH, INTER_ELEMENT);
          }
        }
        // Add inter-letter space only if followed by another character
        if (i < inputLength - 1 && speech_morse_play[i + 1] != ' ' && speech_morse_play[i + 1] != '\0') {
          morse_tone(0, INTER_LETTER - INTER_ELEMENT);
        }
      }
    }

    // this will trigger the release of the transmit relay
    speech_relay_action = 1;

    // connect the compressor back to the output
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    dsp.mute(MOD_MUTE3_2_MUTENOSLEWALG1MUTE_ADDR, 1);

    // disable output sine gens
    dsp.mute(MOD_MUTE3_3_MUTENOSLEWALG2MUTE_ADDR, 0);
    xSemaphoreGive(i2c_mutex);

    // turn off the sine generator
    set_sine1_on(false);

    speech_morse_state = 3;

  }
}

void morse_tone(int onTimeMs, int pauseTimeMs) {
  if (onTimeMs > 0) {
    // Turn tone on
    set_sine_frequency1(900);
    set_sine1_on(true);
    vTaskDelay(pdMS_TO_TICKS(onTimeMs));
    set_sine1_on(false);
  }
  vTaskDelay(pdMS_TO_TICKS(pauseTimeMs));
}

// save prefs to set 0-3
void save_presets(int setNumber) {
  String ns_config = "speech" + String(setNumber);

  // String ns_msg = ns_config + "_msg";

  preferences.begin(ns_config.c_str(), false);
  preferences.putUChar("version", 1); // Mark this set as valid

  // main screen
  preferences.putInt("gain", speech_gain);
  preferences.putInt("output", speech_output);
  preferences.putInt("hpoutput", speech_headphone);
  preferences.putBool("eot_active", speech_eot_active);

  preferences.putInt("amp", speech_amplifier);
  preferences.putInt("vuscale", speech_vu_scaling);


  // compressor screen
  preferences.putInt("comp_thresh", speech_compressor_threshold);
  preferences.putInt("comp_ratio", speech_compressor_ratio);
  preferences.putInt("comp_postgain", speech_compressor_postgain);
  preferences.putInt("comp_ngate", speech_compressor_noisegate);

  // equaliser screen
  preferences.putInt("eq0", speech_eq0);
  preferences.putInt("eq1", speech_eq1);
  preferences.putInt("eq2", speech_eq2);
  preferences.putInt("eq3", speech_eq3);
  preferences.putInt("eq4", speech_eq4);
  preferences.putInt("eq5", speech_eq5);
  preferences.putInt("eq6", speech_eq6);
  preferences.putInt("eq7", speech_eq7);
  preferences.putInt("eq8", speech_eq8);
  preferences.putInt("eq9", speech_eq9);
  preferences.putInt("eq10", speech_eq10);
  preferences.putInt("eq11", speech_eq11);

  // effects screen
  preferences.putBool("fx_echo_on", speech_effect_echo_on);
  preferences.putBool("fx_sb_on", speech_effect_sb_on);
  preferences.putBool("fx_pitch_on", speech_effect_pitch_on);
  preferences.putBool("fx_ring_on", speech_effect_ring_on);

  preferences.putInt("fx_echo", speech_effect_echo_time);
  preferences.putInt("fx_fb", speech_effect_echo_fb);
  preferences.putInt("fx_pitch", speech_effect_pitch);
  preferences.putInt("fx_ring", speech_effect_ring);

  // eot screen
  preferences.putUShort("eot_beep", speech_eot_beep_selected);
  preferences.putUShort("eot_sound", speech_eot_sound_selected);
  preferences.putUChar("eot_sel", speech_eot_selection);

  preferences.putInt("eot_level", speech_eot_level);
  preferences.putInt("eot_mp3level", speech_eot_mp3_level);

  // vox screen
  preferences.putInt("vox_sens", speech_vox_threshold_slider);
  preferences.putInt("vox_htime", speech_vox_hangtime_slider);

  preferences.putInt("morse_level", speech_morse_level);

  preferences.putInt("autokey_level", speech_autokey_level);


  preferences.end();

}

// load prefs from set 0-5
bool load_presets(int setNumber) {
  String ns_config = "speech" + String(setNumber);

  // String ns_msg = ns_config + "_msg";

  preferences.begin(ns_config.c_str(), true);
  uint8_t version = preferences.getUChar("version", 0);
  if (version != 1) {
    preferences.end();
    return false;
  }

  // main screen
  speech_gain = preferences.getInt("gain", 0);
  speech_output = preferences.getInt("output", 0);
  speech_headphone = preferences.getInt("hpoutput", 0);
  speech_eot_active = preferences.getBool("eot_active", false);

  speech_amplifier = preferences.getInt("amp", 4);
  speech_vu_scaling = preferences.getInt("vuscale", 10);

  // compressor screen
  speech_compressor_threshold = preferences.getInt("comp_thresh", 0);
  speech_compressor_ratio = preferences.getInt("comp_ratio", 0);
  speech_compressor_postgain = preferences.getInt("comp_postgain", 0);
  speech_compressor_noisegate = preferences.getInt("comp_ngate", 0);

  // equaliser screen
  speech_eq0 = preferences.getInt("eq0", 0.0);
  speech_eq1 = preferences.getInt("eq1", 0.0);
  speech_eq2 = preferences.getInt("eq2", 0.0);
  speech_eq3 = preferences.getInt("eq3", 0.0);
  speech_eq4 = preferences.getInt("eq4", 0.0);
  speech_eq5 = preferences.getInt("eq5", 0.0);
  speech_eq6 = preferences.getInt("eq6", 0.0);
  speech_eq7 = preferences.getInt("eq7", 0.0);
  speech_eq8 = preferences.getInt("eq8", 0.0);
  speech_eq9 = preferences.getInt("eq9", 0.0);
  speech_eq10 = preferences.getInt("eq10", 0.0);
  speech_eq11 = preferences.getInt("eq11", 0.0);

  // effects screen
  speech_effect_echo_on = preferences.getBool("fx_echo_on", false);
  speech_effect_sb_on = preferences.getBool("fx_sb_on", false);
  speech_effect_pitch_on = preferences.getBool("fx_pitch_on", false);
  speech_effect_ring_on = preferences.getBool("fx_ring_on", false);

  speech_effect_echo_time = preferences.getInt("fx_echo", 0);
  speech_effect_echo_fb = preferences.getInt("fx_fb", 0);
  speech_effect_pitch = preferences.getInt("fx_pitch", 0);
  speech_effect_ring = preferences.getInt("fx_ring", 0);

  // eot screen
  speech_eot_beep_selected = preferences.getUShort("eot_beep", 1);
  speech_eot_sound_selected = preferences.getUShort("eot_sound", 1);
  speech_eot_selection = preferences.getUChar("eot_sel", 0);

  speech_eot_level = preferences.getInt("eot_level", 20);
  speech_eot_mp3_level = preferences.getInt("eot_mp3level", 20);

  // vox screen
  speech_vox_threshold_slider = preferences.getInt("vox_sens", 0);
  speech_vox_hangtime_slider = preferences.getInt("vox_htime", 0);

  speech_morse_level = preferences.getInt("morse_level", 20);

  speech_autokey_level = preferences.getInt("autokey_level", 20);

  preferences.end();

  apply_presets();

  return true;
}


// set all relevant objects from preferences

void apply_presets(void) {
  // input amplifier factor
  update_amp_level();

  update_vu_level();

  update_morse_level();

  update_autokey_level();

  // speech_gain
  update_gain();
  // speech_output
  update_output();
  // headphone output
  update_headphone();

  // speech_eot_active
  set_eot();

  //speech_compressor_threshold
  update_compressor_threshold();

  // speech_compressor_ratio
  update_compressor_ratio();

  // speech_compressor_postgain
  update_compressor_postgain();

  // speech_compressor_noisegate
  update_compressor_noisegate();
  // equaliser
  update_eq_all_sliders();


  set_effect_checkboxes();

  // speech_effect_echo_time
  update_effects_echo_time();

  // speech_effect_echo_fb
  update_effects_echo_fb();

  set_effects_echo_sb_slider();
  set_effect_echo_sb_mux(speech_effect_echo_on, speech_effect_sb_on);

  // speech_effect_pitch
  update_effects_pitch();
  set_effects_pitch_slider();
  set_effect_pitch_mux(speech_effect_pitch_on);

  // speech_effect_ring
  update_effects_ring();
  set_effects_ring_slider();
  set_effect_ring_mux(speech_effect_ring_on);

  set_effect_sliders_labels();

  // speech_eot_beep_selected
  set_eot_beep_roller();

  // speech_eot_sound_selected
  set_eot_sound_roller();

  // speech_eot_selection
  set_eot_selection_buttons_state();

  update_eot_level();

  update_eot_mp3_level();

  // speech_vox_threshold_slider
  vox_set_threshold();

  // speech_vox_hangtime_slider
  vox_set_hangtime();

  // speech_presets_msg
  set_presets_labels();

  // speech_voice_msg
  set_voice_labels();
}

void save_presets_names(void) {
  preferences.begin("pref_names", false);
  for (int i = 0; i < 6; i++) {
    preferences.putString(("presetmsg" + String(i)).c_str(), speech_presets_msg[i]);
  }
  preferences.end();
}

void load_presets_names(void) {
  preferences.begin("pref_names", true);
  for (int i = 0; i < 6; i++) {
    speech_presets_msg[i] = preferences.getString(("presetmsg" + String(i)).c_str(), "Preset");
  }
  preferences.end();
}

void save_voice_names(void) {
  preferences.begin("voice_names", false);
  for (int i = 0; i < 5; i++) {
    preferences.putString(("voicemsg" + String(i)).c_str(), speech_voice_msg[i]);
  }
  preferences.end();
}

void load_voice_names(void) {
  preferences.begin("voice_names", true);
  for (int i = 0; i < 5; i++) {
    speech_voice_msg[i] = preferences.getString(("voicemsg" + String(i)).c_str(), "Voice msg");
  }
  preferences.end();
}

void save_morse_names(void) {
  preferences.begin("morse_names", false);
  for (int i = 0; i < 5; i++) {
    preferences.putString(("morsemsg" + String(i)).c_str(), speech_morse_msg[i]);
  }
  preferences.end();
}

void load_morse_names(void) {
  preferences.begin("morse_names", true);
  for (int i = 0; i < 5; i++) {
    speech_morse_msg[i] = preferences.getString(("morsemsg" + String(i)).c_str(), "Morse msg");
  }
  preferences.end();
}

void print_task_status(void) {
  char buffer[1024];
  vTaskList(buffer);
  Serial.println(buffer);
  vTaskGetRunTimeStats(buffer);
  Serial.println("Task Runtime Stats:");
  Serial.println(buffer);
}
