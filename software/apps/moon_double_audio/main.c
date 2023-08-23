#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/structs/bus_ctrl.h"
#include "pico/multicore.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "tmds_encode.h"
#include "tmds_double.h"
#include "audio.h"

#include "moon_1bpp_640x480.h"
#define moon_img moon_1bpp_640x480
#define IMAGE_WIDTH 640
  
#define AUDIO_RATE 32000    // Audio Sample rate
#define HDMI_N     4096     // From HDMI standard for 32kHz

#define FIFTYHZMS   20      // ms between frames at 50Hz
#define TICKMS      2       // Time between audio callbacks
#define TICKCOUNT   (FIFTYHZMS/TICKMS)
#define MAX_SIZE    (TICKMS * AUDIO_RATE / 1000)

static semaphore_t fifty_hz;
static void core1_main();

// Display 2x2 pixels for each pixel in B&W image
// Also plays audio
// Note: Audio will sound slow, as recorded at 44.1kHz,
// but playback set to 32kHz

// Pick one:
//#define MODE_640x480_60Hz
#define MODE_720x540_50Hz
//#define MODE_720x576_50Hz

#if defined(MODE_640x480_60Hz)
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define DVI_TIMING dvi_timing_640x480p_60hz
#elif defined(MODE_720x540_50Hz)
#define FRAME_WIDTH 720
#define FRAME_HEIGHT 540
#define DVI_TIMING dvi_timing_720x540p_50hz
#elif defined(MODE_720x576_50Hz)
#define FRAME_WIDTH 720
#define FRAME_HEIGHT 576
#define DVI_TIMING dvi_timing_720x576p_50hz

#else
#error "Select a video mode!"
#endif

struct dvi_inst dvi0;

//Audio Related
#define AUDIO_BUFFER_SIZE   512
audio_sample_t      audio_buffer[AUDIO_BUFFER_SIZE];
struct repeating_timer audio_timer;

static bool __not_in_flash_func(audio_timer_callback)(struct repeating_timer *t) 
{
    static uint call_count = 0;
    static uint sample_count = 0;

    ++call_count;

    // write in chunks
    int size = get_write_size(&dvi0.audio_ring, true);
    __mem_fence_release();
    if (size >= MAX_SIZE)
    {   
        size = get_write_size(&dvi0.audio_ring, false);
        audio_sample_t *audio_ptr = get_write_pointer(&dvi0.audio_ring);
        if (size > ((3*AUDIO_BUFFER_SIZE)>>2))
        {
            // Allow to refill buffer
            size = (MAX_SIZE<<1);
        }
        else
        {
            size = (size>MAX_SIZE) ? MAX_SIZE : size;
        }

        for (int cnt = 0; cnt < size; cnt++) 
        {
            audio_ptr->channels[0] = commodore_argentina[sample_count % commodore_argentina_len] << 8;
            audio_ptr->channels[1] = commodore_argentina[(sample_count+1024) % commodore_argentina_len] << 8;
            sample_count = (sample_count + 1) % commodore_argentina_len;
            audio_ptr++;
        }
        increase_write_pointer(&dvi0.audio_ring, size);

        if (size < MAX_SIZE)
        {
            int sizeleft = get_write_size(&dvi0.audio_ring, false);
            audio_ptr = get_write_pointer(&dvi0.audio_ring);
            sizeleft = ((sizeleft + size) >MAX_SIZE) ?  (MAX_SIZE - size) : sizeleft; 

            for (int cnt = 0; cnt < sizeleft; cnt++) 
            {
                audio_ptr->channels[0] = commodore_argentina[sample_count % commodore_argentina_len] << 8;
                audio_ptr->channels[1] = commodore_argentina[(sample_count+1024) % commodore_argentina_len] << 8;
                sample_count = (sample_count + 1) % commodore_argentina_len;
                audio_ptr++;
            }
            increase_write_pointer(&dvi0.audio_ring, sizeleft);
        }
    }
    if (call_count == TICKCOUNT)
    {
        sem_release(&fifty_hz);
    }
    return true;
}

int main() 
{
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
	setup_default_uart();

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

    // Give priority to 2nd core
	hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);

    // HDMI Audio related
    dvi_get_blank_settings(&dvi0)->top    = 0;
    dvi_get_blank_settings(&dvi0)->bottom = 0;
    dvi_audio_sample_buffer_set(&dvi0, audio_buffer, AUDIO_BUFFER_SIZE);
    dvi_set_audio_freq(&dvi0, AUDIO_RATE, dvi0.timing->bit_clk_khz*HDMI_N/(AUDIO_RATE/1000)/128, HDMI_N);

	printf("Set CTS %i\n", dvi0.timing->bit_clk_khz*HDMI_N/(AUDIO_RATE/1000)/128);

    sem_init(&fifty_hz, 0, 1);
 
    // launch all the video on core 1
    multicore_launch_core1(core1_main);
    add_repeating_timer_ms(-TICKMS, audio_timer_callback, NULL, &audio_timer);

	while(1) __wfi();
}
	
static void __not_in_flash_func(render_loop)()
{
	while (true) 
    {
		for (uint y = 0; y < FRAME_HEIGHT/2; ++y)
        {
			const uint32_t *colourbuf = &((const uint32_t*)moon_img)[(y + (FRAME_HEIGHT >> 3))* IMAGE_WIDTH / 32];
			uint32_t *tmdsbuf;
			queue_remove_blocking_u32(&dvi0.q_tmds_free, &tmdsbuf);

            // 32 pixels create 8 * 2 32 bit words
            // tmds_double_1bpp requires multiple of 32 pixels
            // For input of 360 to generate 720 pixels:
            // 360 pixels -> 11.25 32 bit words
            // Will therefore generate 12 * 32 * 2 pixels = 768
            // tmdsbuf size must be rounded up to multiple of 64
            // in dvi.c
			tmds_double_1bpp(colourbuf, tmdsbuf, FRAME_WIDTH);
			queue_add_blocking_u32(&dvi0.q_tmds_valid, &tmdsbuf);
		}
	}
}

static void core1_main()
{
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	dvi_start(&dvi0);
    render_loop();
}
