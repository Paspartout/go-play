#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_task_wdt.h"
#include "esp_spiffs.h"
#include "driver/rtc_io.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"

#include "../components/gnuboy/loader.h"
#include "../components/gnuboy/hw.h"
#include "../components/gnuboy/lcd.h"
#include "../components/gnuboy/fb.h"
#include "../components/gnuboy/cpu.h"
#include "../components/gnuboy/mem.h"
#include "../components/gnuboy/sound.h"
#include "../components/gnuboy/pcm.h"
#include "../components/gnuboy/regs.h"
#include "../components/gnuboy/rtc.h"
#include "../components/gnuboy/gnuboy.h"
#include "../components/gnuboy/dmgpal.h"

#include <string.h>

#include "../components/odroid/odroid_settings.h"
#include "../components/odroid/odroid_input.h"
#include "../components/odroid/odroid_display.h"
#include "../components/odroid/odroid_audio.h"
#include "../components/odroid/odroid_system.h"
#include "../components/odroid/odroid_sdcard.h"
#include "../components/odroid/odroid_ui.h"


extern int debug_trace;

struct fb fb;
struct pcm pcm;


uint16_t* displayBuffer[2]; //= { fb0, fb0 }; //[160 * 144];
uint8_t currentBuffer;

uint16_t* framebuffer;
int frame = 0;
uint elapsedTime = 0;

int16_t* audioBuffer[2];
uint8_t currentAudioBuffer = 0;
uint16_t currentAudioSampleCount;
int16_t* currentAudioBufferPtr;

odroid_battery_state battery_state;

const char* StateFileName = "/storage/gnuboy.sav";

#define GAMEBOY_WIDTH (160)
#define GAMEBOY_HEIGHT (144)

#define AUDIO_SAMPLE_RATE (32000)

#define PIXEL_MASK 0x3F

const char* SD_BASE_PATH = "/sd";

// --- MAIN
QueueHandle_t vidQueue;
QueueHandle_t audioQueue;

float Volume = 1.0f;

int pcm_submit()
{
    odroid_audio_submit(currentAudioBufferPtr, currentAudioSampleCount >> 1);

    return 1;
}


int BatteryPercent = 100;
bool enable_partial_updates = false;
bool force_screen_update = false;
bool force_clear = false;
bool stretch_screen = false;

typedef enum {
    // Full update, like the original gnuboy
    UPDATE_FULL,
    // Experimental Partial update with palette diffing
    UPDATE_PARTIAL,
} update_type_t;

struct display_update_full {
    update_type_t type;

    uint16_t* buffer;
};

struct display_update_partial {
    update_type_t type;

    odroid_scanline diff[GAMEBOY_HEIGHT];
	bool force_full_update;
    uint8_t *buffer;
    uint16_t palette[64];
    int stride;
};

// Can hold either full or partial update
typedef union {
    update_type_t type;
    struct display_update_partial partial;
    struct display_update_full full;
} display_update_t;


display_update_t update1 = {0, };
display_update_t update2 = {0, };
display_update_t *current_update = &update2;
bool skipFrames = false;
int cntr = 0;
int field = 1;

void run_to_vblank(bool display_frame)
{
  /* FRAME BEGIN */

  /* FIXME: djudging by the time specified this was intended
  to emulate through vblank phase which is handled at the
  end of the loop. */
  cpu_emulate(2280);

  /* FIXME: R_LY >= 0; comparsion to zero can also be removed
  altogether, R_LY is always 0 at this point */
  while (R_LY > 0 && R_LY < 144)
  {
    /* Step through visible line scanning phase */
    emu_step();
  }
  interlace_first_run = !interlace_first_run;
  /* VBLANK BEGIN */

  //vid_end();
  
  if (display_frame)
  {
        if (enable_partial_updates) {
            display_update_t* old_update = current_update;

            // Swap updates
            current_update = (current_update == &update1) ? &update2 : &update1;

            current_update->type = UPDATE_PARTIAL;

            current_update->partial.buffer = (uint8_t*)framebuffer;
            current_update->partial.stride = GAMEBOY_WIDTH;
         
            memcpy(current_update->partial.palette, scan.pal2, 64 * sizeof(uint16_t));

            isNewFrame = true;
            int interlace;
            if (interlace_first_run) {
                interlace = 0;
            } else { interlace = 1; 
            }
            
            interlace = 1 - interlace;
            
            for (int y = interlace_first_run ? 1 : 0; y < GAMEBOY_HEIGHT; y += 2){
                current_update->partial.diff[y].width = 0;
            }            

            odroid_buffer_diff_interlaced(current_update->partial.buffer,
                old_update->partial.buffer, current_update->partial.palette, old_update->partial.palette,
                GAMEBOY_WIDTH, GAMEBOY_HEIGHT,
                current_update->partial.stride, PIXEL_MASK, 0, interlace, current_update->partial.diff, old_update->partial.diff);
        
            
		  /* TODO: Figure out why interlacing doesnt work quite right. */
		  /* field = 1 - field; */
	      /* odroid_buffer_diff_interlaced(current_update->partial.buffer, */
                  /* old_update->partial.buffer, current_update->partial.palette, old_update->partial.palette, */
			  /* GAMEBOY_WIDTH, GAMEBOY_HEIGHT, */
			  /* current_update->partial.stride, PIXEL_MASK, 0, field, current_update->partial.diff, old_update->partial.diff); */


        /* // TODO: determine right threshold and make it dependend on scale setting
		  const int threshold = 16000;
		  int n_pixels = odroid_buffer_diff_count(current_update->partial.diff, GAMEBOY_HEIGHT);
          
          if (lcd_interlace) {
              n_pixels = n_pixels * 2;
         }*/

		  // State machine that does automatic frame skipping
		  current_update->partial.force_full_update = false;
          xQueueSend(vidQueue, &current_update, portMAX_DELAY);
        } else {
          current_update->type = UPDATE_FULL;
          current_update->full.buffer = framebuffer;
		  xQueueSend(vidQueue, &current_update, portMAX_DELAY);
        }

        //merge & swap buffers
        currentBuffer = currentBuffer ? 0 : 1;
        framebuffer = displayBuffer[currentBuffer];
        
        fb.ptr = (uint8_t*)framebuffer;
  }

  rtc_tick();

  sound_mix();

  //if (pcm.pos > 100)
  {
        currentAudioBufferPtr = audioBuffer[currentAudioBuffer];
        currentAudioSampleCount = pcm.pos;

        void* tempPtr = (void*)0x1234;
        xQueueSend(audioQueue, &tempPtr, portMAX_DELAY);

        // Swap buffers
        currentAudioBuffer = currentAudioBuffer ? 0 : 1;
        pcm.buf = audioBuffer[currentAudioBuffer];
        pcm.pos = 0;
  }

  if (!(R_LCDC & 0x80)) {
    /* LCDC operation stopped */
    /* FIXME: djudging by the time specified, this is
    intended to emulate through visible line scanning
    phase, even though we are already at vblank here */
    cpu_emulate(32832);
  }

  while (R_LY > 0) {
    /* Step through vblank phase */
    emu_step();
  }
}


uint16_t* menuFramebuffer = 0;

volatile bool videoTaskIsRunning = false;
bool scaling_enabled = true;
bool previous_scale_enabled = true;

static void update_scaling() {
    if (scaling_enabled) {
        odroid_display_set_scale(GAMEBOY_WIDTH, GAMEBOY_HEIGHT, stretch_screen ? 1.2f : 1.0f);
    } else {
        odroid_display_reset_scale(GAMEBOY_WIDTH, GAMEBOY_HEIGHT);
    }
}

void videoTask(void *arg)
{
  videoTaskIsRunning = true;
  uint16_t display_palette[64] = {0, };
  update_type_t old_update_type = UPDATE_FULL;

  display_update_t* update;
  while(1)
  {
        xQueuePeek(vidQueue, &update, portMAX_DELAY);

        if ((int)update == 1)
            break;

        const bool scale_changed = previous_scale_enabled != scaling_enabled;
        if (scale_changed) {
            previous_scale_enabled = scaling_enabled;
			update_scaling();
        }
        const bool video_path_changed = update->type != old_update_type; 
        if (video_path_changed) {
            old_update_type = update->type;
        }
        const bool clear_display = scale_changed || video_path_changed || force_clear;
		if (clear_display)
		{
			ili9341_clear(0x0000);
			force_clear = false;
		}
        if (update->type == UPDATE_FULL) {
            // Full update video path
            
            ili9341_write_frame_gb(update->full.buffer, scaling_enabled);
        } else {
			for(int i = 0; i < PIXEL_MASK+1; i++) {
				const uint16_t pix = update->partial.palette[i];
				display_palette[i] =  pix << 8 | pix >> 8;
			}

			ili9341_write_frame_8bit(update->partial.buffer,
					update->partial.diff,
					GAMEBOY_WIDTH, GAMEBOY_HEIGHT, update->partial.stride, PIXEL_MASK, display_palette);
            
			if (force_screen_update) {
				force_screen_update = false;
			}
        }

        odroid_input_battery_level_read(&battery_state);

        xQueueReceive(vidQueue, &update, portMAX_DELAY);
    }


    // Draw hourglass
    odroid_display_lock();

    odroid_display_show_hourglass();

    odroid_display_unlock();


    videoTaskIsRunning = false;
    vTaskDelete(NULL);

    while (1) {}
}


volatile bool AudioTaskIsRunning = false;
void audioTask(void* arg)
{
  // sound
  uint16_t* param;

  AudioTaskIsRunning = true;
  while(1)
  {
    xQueuePeek(audioQueue, &param, portMAX_DELAY);

    if (param == 0)
    {
        // TODO: determine if this is still needed
        abort();
    }
    else if ((int)param == 1)
    {
        break;
    }
    else if (!config_speedup)
    {
        pcm_submit();
    }

    xQueueReceive(audioQueue, &param, portMAX_DELAY);
  }

  printf("audioTask: exiting.\n");
  odroid_audio_terminate();

  AudioTaskIsRunning = false;
  vTaskDelete(NULL);

  while (1) {}
}

bool QuickSaveState(FILE* f)
{
	savestate(f);
    return true;
}

bool QuickLoadState(FILE *f)
{
    loadstate(f);
    fclose(f);
    odroid_display_unlock();
    
    vram_dirty();
    pal_dirty();
    sound_dirty();
    mem_updatemap();
    return true;
}

void DoMenuHome(bool save)
{
    uint16_t* param = (uint16_t*)1;

    // Clear audio to prevent studdering
    printf("PowerDown: stopping audio.\n");

    xQueueSend(audioQueue, &param, portMAX_DELAY);
    while (AudioTaskIsRunning) {}


    // Stop tasks
    printf("PowerDown: stopping tasks.\n");

    xQueueSend(vidQueue, &param, portMAX_DELAY);
    while (videoTaskIsRunning) {}
    
    //prevents go-play breaking from unexpected volume level on exit
    //todo save volume level & apply at next load
    odroid_ui_volume_exit();

    DoReboot(save);
    }

uint menu_restart_timer = 0;

void menu_gb_pal_update(odroid_ui_entry *entry) {
    sprintf(entry->text, "%-9s: %d", "pal", pal_get());
}

odroid_ui_func_toggle_rc menu_gb_pal_toggle(odroid_ui_entry *entry, odroid_gamepad_state *joystick) {
    if (joystick->values[ODROID_INPUT_RIGHT] || joystick->values[ODROID_INPUT_A]) {
		pal_next();
    } else if (joystick->values[ODROID_INPUT_LEFT]) {
		pal_prev();
    }
    odroid_settings_GBPalette_set(pal_get());
    menu_restart_timer = 4;
    return ODROID_UI_FUNC_TOGGLE_RC_MENU_RESTART;
}

extern struct rtc rtc;

void menu_gb_rtc_day_update(odroid_ui_entry *entry) {
    sprintf(entry->text, "%-9s: %d", "rtc-day", rtc.d);
}

odroid_ui_func_toggle_rc menu_gb_rtc_day_toggle(odroid_ui_entry *entry, odroid_gamepad_state *joystick) {
    if (joystick->values[ODROID_INPUT_RIGHT] || joystick->values[ODROID_INPUT_A]) {
        rtc.d = (rtc.d + 1)%365;
    } else if (joystick->values[ODROID_INPUT_LEFT]) {
        rtc.d = (rtc.d - 1 + 365)%365;
    }
    return ODROID_UI_FUNC_TOGGLE_RC_CHANGED;
}

void menu_gb_rtc_hour_update(odroid_ui_entry *entry) {
    sprintf(entry->text, "%-9s: %d", "rtc-hour", rtc.h);
}

odroid_ui_func_toggle_rc menu_gb_rtc_hour_toggle(odroid_ui_entry *entry, odroid_gamepad_state *joystick) {
    if (joystick->values[ODROID_INPUT_RIGHT] || joystick->values[ODROID_INPUT_A]) {
        rtc.h = (rtc.h + 1)%24;
    } else if (joystick->values[ODROID_INPUT_LEFT]) {
        rtc.h = (rtc.h - 1 + 24)%24;
    }
    return ODROID_UI_FUNC_TOGGLE_RC_CHANGED;
}

void menu_gb_rtc_minute_update(odroid_ui_entry *entry) {
    sprintf(entry->text, "%-9s: %d", "rtc-min", rtc.m);
}

odroid_ui_func_toggle_rc menu_gb_rtc_minute_toggle(odroid_ui_entry *entry, odroid_gamepad_state *joystick) {
    if (joystick->values[ODROID_INPUT_RIGHT] || joystick->values[ODROID_INPUT_A]) {
        rtc.m = (rtc.m + 1)%60;
    } else if (joystick->values[ODROID_INPUT_LEFT]) {
        rtc.m = (rtc.m - 1 + 60)%60;
    }
    return ODROID_UI_FUNC_TOGGLE_RC_CHANGED;
}

void menu_gb_partial_updates_update(odroid_ui_entry *entry) {
    sprintf(entry->text, "%-9s: %s", "partial", enable_partial_updates ? "on" : "off");
}

odroid_ui_func_toggle_rc menu_gb_partial_updates_toggle(odroid_ui_entry *entry, odroid_gamepad_state *joystick) {
	enable_partial_updates = !enable_partial_updates;
	odroid_settings_PartialUpdates_set(enable_partial_updates);
    return ODROID_UI_FUNC_TOGGLE_RC_CHANGED;
}

void menu_gb_stretch_update(odroid_ui_entry *entry) {
	const char *text;
	if (enable_partial_updates) {
		text = stretch_screen ? "on" : "off";
	} else {
		text = "unavail";
	}
    sprintf(entry->text, "%-9s: %s", "stretch", text);
}

odroid_ui_func_toggle_rc menu_gb_stretch_toggle(odroid_ui_entry *entry, odroid_gamepad_state *joystick) {
	if (!enable_partial_updates)
		return ODROID_UI_FUNC_TOGGLE_RC_NOTHING;
	stretch_screen = !stretch_screen;
	force_clear = true;
	update_scaling();
	odroid_settings_ScaleStretch_set(enable_partial_updates);
    return ODROID_UI_FUNC_TOGGLE_RC_CHANGED;

    menu_restart_timer = 4;
    return ODROID_UI_FUNC_TOGGLE_RC_MENU_RESTART;
}

void menu_gb_init(odroid_ui_window *window) {
    odroid_ui_create_entry(window, &menu_gb_pal_update, &menu_gb_pal_toggle);
    odroid_ui_create_entry(window, &menu_gb_rtc_day_update, &menu_gb_rtc_day_toggle);
    odroid_ui_create_entry(window, &menu_gb_rtc_hour_update, &menu_gb_rtc_hour_toggle);
    odroid_ui_create_entry(window, &menu_gb_rtc_minute_update, &menu_gb_rtc_minute_toggle);

    odroid_ui_create_entry(window, &menu_gb_partial_updates_update, &menu_gb_partial_updates_toggle);
    odroid_ui_create_entry(window, &menu_gb_stretch_update, &menu_gb_stretch_toggle);
}

void app_main(void)
{
    printf("gnuboy (%s-%s).\n", COMPILEDATE, GITREV);

    nvs_flash_init();

    odroid_system_init();

    odroid_input_gamepad_init();

    check_boot_cause();

    // Display
    ili9341_prepare();
    ili9341_init();
    //odroid_display_show_splash();

    // Load ROM
    loader_init(NULL);

    // Clear display
    ili9341_write_frame_gb(NULL, true);

    // Audio hardware
    odroid_audio_init(odroid_settings_AudioSink_get(), AUDIO_SAMPLE_RATE);

    // Allocate display buffers
    displayBuffer[0] = heap_caps_malloc(160 * 144 * 2, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    displayBuffer[1] = heap_caps_malloc(160 * 144 * 2, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);

    if (displayBuffer[0] == 0 || displayBuffer[1] == 0)
        abort();

    framebuffer = displayBuffer[0];

    for (int i = 0; i < 2; ++i)
    {
        memset(displayBuffer[i], 0, 160 * 144 * 2);
    }

    printf("app_main: displayBuffer[0]=%p, [1]=%p\n", displayBuffer[0], displayBuffer[1]);

    // blue led
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 0);

    //  Charge
    odroid_input_battery_level_init();

    // video
    vidQueue = xQueueCreate(1, sizeof(uint16_t*));
    audioQueue = xQueueCreate(1, sizeof(uint16_t*));

    xTaskCreatePinnedToCore(&videoTask, "videoTask", 2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(&audioTask, "audioTask", 2048, NULL, 6, NULL, 1); //768


    //debug_trace = 1;

    emu_reset();

    //&rtc.carry, &rtc.stop,
    rtc.d = 1;
    rtc.h = 1;
    rtc.m = 1;
    rtc.s = 1;
    rtc.t = 1;

    // vid_begin
    memset(&fb, 0, sizeof(fb));
    fb.w = 160;
  	fb.h = 144;
  	fb.pelsize = 2;
  	fb.pitch = fb.w * fb.pelsize;
  	fb.indexed = 0;
	fb.ptr = (uint8_t*)framebuffer;
  	fb.enabled = 1;
  	fb.dirty = 0;


    // Note: Magic number obtained by adjusting until audio buffer overflows stop.
    const int audioBufferLength = AUDIO_SAMPLE_RATE / 10 + 1;
    //printf("CHECKPOINT AUDIO: HEAP:0x%x - allocating 0x%x\n", esp_get_free_heap_size(), audioBufferLength * sizeof(int16_t) * 2 * 2);
    const int AUDIO_BUFFER_SIZE = audioBufferLength * sizeof(int16_t) * 2;

    // pcm.len = count of 16bit samples (x2 for stereo)
    memset(&pcm, 0, sizeof(pcm));
    pcm.hz = AUDIO_SAMPLE_RATE;
  	pcm.stereo = 1;
  	pcm.len = /*pcm.hz / 2*/ audioBufferLength;
  	pcm.buf = heap_caps_malloc(AUDIO_BUFFER_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  	pcm.pos = 0;

    audioBuffer[0] = pcm.buf;
    audioBuffer[1] = heap_caps_malloc(AUDIO_BUFFER_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);

    if (audioBuffer[0] == 0 || audioBuffer[1] == 0)
        abort();

    sound_reset();
    lcd_begin();
    // Load state

    DoStartupPost();

    LoadState(rom.name);

    // Update DMG color palette
    printf("dmgpal: loaded palette %d\n", dmg_cur_pal);
    if (!hw.cgb && dmg_color && dmg_cur_pal < 13)
        dmg_pal_update(dmg_cur_pal);

    uint startTime;
    uint stopTime;
    uint totalElapsedTime = 0;
    uint actualFrameCount = 0;
	uint skippedFrames = 0;
	bool display_frame = true;
    odroid_gamepad_state lastJoysticState;

    scaling_enabled = odroid_settings_ScaleDisabled_get(ODROID_SCALE_DISABLE_GB) ? false : true;
	stretch_screen = odroid_settings_ScaleStretch_get();
	enable_partial_updates = odroid_settings_PartialUpdates_get();
    //pal_set(odroid_settings_GBPalette_get());
	update_scaling();

    odroid_input_gamepad_read(&lastJoysticState);
    
    QuickSaveSetBuffer( (void*)(0x3f800000 + (0x100000 * 3) + (0x100000 / 2)));
    ODROID_UI_MENU_HANDLER_INIT_V1(lastJoysticState)
    

    while (true)
    {
        odroid_gamepad_state joystick;
        odroid_input_gamepad_read(&joystick);

        ODROID_UI_MENU_HANDLER_LOOP_V2(lastJoysticState, joystick, DoMenuHome);

        if (joystick.values[ODROID_INPUT_VOLUME] || menu_restart)
        {
            if (menu_restart_timer > 0) {
               menu_restart_timer--;
            } else {
            do {
              
              menu_restart = odroid_ui_menu_ext(menu_restart, &menu_gb_init);
			  if (enable_partial_updates) {
				  force_screen_update = true;
			  }
            } while(menu_restart_timer == 0 && menu_restart);
            }
        }

        // Scaling
        if (joystick.values[ODROID_INPUT_START] && !lastJoysticState.values[ODROID_INPUT_RIGHT] && joystick.values[ODROID_INPUT_RIGHT])
        {
            scaling_enabled = !scaling_enabled;
            odroid_settings_ScaleDisabled_set(ODROID_SCALE_DISABLE_GB, scaling_enabled ? 0 : 1);
			force_screen_update = true;
        }

		// Cycle through palets
		if (joystick.values[ODROID_INPUT_START] && !lastJoysticState.values[ODROID_INPUT_LEFT] && joystick.values[ODROID_INPUT_LEFT])
        {
			pal_next();
			odroid_settings_GBPalette_set(pal_get());
        }

        // DMG color palette cycling
        if (!hw.cgb && dmg_color && joystick.values[ODROID_INPUT_START] && !lastJoysticState.values[ODROID_INPUT_LEFT] && joystick.values[ODROID_INPUT_LEFT])
        {
            dmg_pal_cycle();
        }


        pad_set(PAD_UP, joystick.values[ODROID_INPUT_UP]);
        pad_set(PAD_RIGHT, joystick.values[ODROID_INPUT_RIGHT]);
        pad_set(PAD_DOWN, joystick.values[ODROID_INPUT_DOWN]);
        pad_set(PAD_LEFT, joystick.values[ODROID_INPUT_LEFT]);

        pad_set(PAD_SELECT, joystick.values[ODROID_INPUT_SELECT]);
        pad_set(PAD_START, joystick.values[ODROID_INPUT_START]);

        pad_set(PAD_A, joystick.values[ODROID_INPUT_A]);
        pad_set(PAD_B, joystick.values[ODROID_INPUT_B]);

        if (!enable_partial_updates) {
            display_frame = config_speedup ? (frame % 10) == 0 : (frame % 2) == 0;
        } else {
			display_frame = true;
		}

        startTime = xthal_get_ccount();
        run_to_vblank(display_frame);
        stopTime = xthal_get_ccount();

        // Cycle budget we can spend to emulate one frame to reach roughly 60 fps
        const int frameTime = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000 / 50;
        // Figure out if we should skip next frame
        display_frame = (elapsedTime > frameTime) ? false : true;
        skippedFrames += display_frame ? 0 : 1;

        lastJoysticState = joystick;

        if (stopTime > startTime)
          elapsedTime = (stopTime - startTime);
        else
          elapsedTime = ((uint64_t)stopTime + (uint64_t)0xffffffff) - (startTime);

        totalElapsedTime += elapsedTime;
        ++frame;
        ++actualFrameCount;

		// TODO: Skip on release to save some more cycles?
        if (actualFrameCount == 60)
        {
          float seconds = totalElapsedTime / (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000.0f); // 240000000.0f; // (240Mhz)
          float fps = actualFrameCount / seconds;

		printf("GAME_FPS:%f, DISPLAYED_FPS: %d, SKIPPED_FRAMES: %d, BATTERY:%d [%d]\n", 
				fps, (int)fps-skippedFrames, skippedFrames, battery_state.millivolts, battery_state.percentage);
	  actualFrameCount = 0;
          totalElapsedTime = 0;
		  skippedFrames = 0;
        }
    }
}
