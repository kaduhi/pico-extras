/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "pico/audio_fm_transmitter.h"

#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "pico/audio_fm_transmitter/sample_encoding.h"

#include "audio_fm_transmitter.pio.h"

#include "pico_fractional_pll.h"

// TODO: add noise shaped fixed dither


#define audio_pio __CONCAT(pio, PICO_AUDIO_FM_TRANSMITTER_PIO)
#define GPIO_FUNC_PIOx __CONCAT(GPIO_FUNC_PIO, PICO_AUDIO_FM_TRANSMITTER_PIO)
#define DREQ_PIOx_TX0 __CONCAT(__CONCAT(DREQ_PIO, PICO_AUDIO_FM_TRANSMITTER_PIO), _TX0)

// ======================
// == DEBUGGING =========

#define ENABLE_PIO_AUDIO_FM_TRANSMITTER_ASSERTIONS

CU_REGISTER_DEBUG_PINS(audio_timing, audio_underflow)

// ---- select at most one ---
//CU_SELECT_DEBUG_PINS(audio_timing)

// ======================

#ifdef ENABLE_PIO_AUDIO_FM_TRANSMITTER_ASSERTIONS
#define audio_assert(x) assert(x)
#else
#define audio_assert(x) (void)0
#endif

#define _UNDERSCORE(x, y) x ## _ ## y
#define _CONCAT(x, y) _UNDERSCORE(x,y)
#define audio_program _CONCAT(program_name,program)
#define audio_program_get_default_config _CONCAT(program_name,program_get_default_config)
#define audio_entry_point _CONCAT(program_name,offset_entry_point)

static bool audio_enabled;
static bool push_queuing_to_core1;

static void __isr __time_critical_func(audio_fm_transmitter_dma_irq_handler)();

static struct {
    audio_buffer_pool_t *playback_buffer_pool[PICO_AUDIO_FM_TRANSMITTER_MAX_CHANNELS];
    audio_buffer_t *playing_buffer[PICO_AUDIO_FM_TRANSMITTER_MAX_CHANNELS];
    // ----- begin protected by free_list_spin_lock -----
    uint8_t pio_sm[PICO_AUDIO_FM_TRANSMITTER_MAX_CHANNELS];
    uint8_t dma_channel[PICO_AUDIO_FM_TRANSMITTER_MAX_CHANNELS];
    int channel_count;
} shared_state;

const audio_fm_transmitter_channel_config_t default_left_channel_config =
        {
                .core = {
                        .base_pin = PICO_AUDIO_FM_TRANSMITTER_L_PIN,
                        .pio_sm = 0,
                        .dma_channel = 0
                },
                .pattern = 1,
        };

const audio_fm_transmitter_channel_config_t default_right_channel_config =
        {
                .core = {
                        .base_pin = PICO_AUDIO_FM_TRANSMITTER_R_PIN,
                        .pio_sm = 1,
                        .dma_channel = 1
                },
                .pattern = 1,
        };

const audio_fm_transmitter_channel_config_t default_mono_channel_config =
        {
                .core = {
                        .base_pin = PICO_AUDIO_FM_TRANSMITTER_MONO_PIN,
                        .pio_sm = 0,
                        .dma_channel = 0
                },
                .pattern = 3,
        };

static audio_buffer_t silence_buffer;

// hack: keeps the current sampling rate
uint32_t g_current_sample_freq = 48000;

static inline void audio_start_dma_transfer(int ch)
{
#if PICO_AUDIO_FM_TRANSMITTER_NOOP
    assert(false);
#else
    assert(!shared_state.playing_buffer[ch]);
    audio_buffer_t *ab = take_audio_buffer(shared_state.playback_buffer_pool[ch], false);

    shared_state.playing_buffer[ch] = ab;
    DEBUG_PINS_SET(audio_underflow, 4);
    if (!ab)
    {
        DEBUG_PINS_XOR(audio_underflow, 2);
        // just play some silence
        ab = &silence_buffer;
//        static int foo;
//        printf("underflow %d\n", foo++);
        DEBUG_PINS_XOR(audio_underflow, 2);
    }
    DEBUG_PINS_CLR(audio_underflow, 4);
    assert(ab->sample_count);
    // todo better naming of format->format->format!!
    assert(ab->format->format->format == NATIVE_BUFFER_FORMAT);
    assert(ab->format->format->channel_count == 1);
    assert(ab->format->sample_stride == sizeof(fm_transmitter_cmd_t));
#if 0
    dma_channel_transfer_from_buffer_now(shared_state.dma_channel[ch], ab->buffer->bytes,
                                         ab->sample_count * sizeof(fm_transmitter_cmd_t) / 4);
#else
    // only use the number of samples for the same time length of the specified sampling rate to avoid the buffer overflow
    // e.g. if the specified sampling rate is 48kHz and the sample_count is 1024, only use 1024 x 22058 / 24000 = 941 samples
    // yes, i know this will shift the pitch of the music... but this PWM output is only for debugging purpose, so fine.
    // by the way, 22058 is the only sampling rate the PWM audio can handle, due to the fixed PIO processing time
    // hack: use g_current_sample_freq to know the current sampling rate of the USB audio, because somehow ab->format->format->sample_freq
    //       does not have it... even supposed to be set in _audio_reconfigure() in usb_sound_card.c
    dma_channel_transfer_from_buffer_now(shared_state.dma_channel[ch], ab->buffer->bytes,
                                         ((uint32_t)ab->sample_count * 22058 / (g_current_sample_freq / 2)) * sizeof(fm_transmitter_cmd_t) / 4);
#endif
}

semaphore_t sem_transfer_buffer_fill, sem_transfer_buffer_drain;
void *volatile transfer_buffer;
int32_t transfer_buffer_sample_count;

// irq handler for DMA
static void __isr __time_critical_func(audio_fm_transmitter_dma_irq_handler)()
{
#if PICO_AUDIO_FM_TRANSMITTER_NOOP
    assert(false);
#else
    // todo better DMA channel handling? (should we combine to keep channels in sync?)
    //  (pico_audio - sync should be maintained by source of pico_audio buffers, though we need to be able to insert
    //  the correct amount of silence to re-align)
    for(int ch = 0; ch < shared_state.channel_count; ch++)
    {
        uint dma_channel = shared_state.dma_channel[ch];
        if (dma_irqn_get_channel_status(PICO_AUDIO_FM_TRANSMITTER_DMA_IRQ, dma_channel)) {
            dma_irqn_acknowledge_channel(PICO_AUDIO_FM_TRANSMITTER_DMA_IRQ, dma_channel);
            DEBUG_PINS_SET(audio_timing, 4);
            // free the buffer we just finished
            if (shared_state.playing_buffer[ch])
            {
                give_audio_buffer(shared_state.playback_buffer_pool[ch], shared_state.playing_buffer[ch]);
#ifndef NDEBUG
                shared_state.playing_buffer[ch] = 0;
#endif
            }
            audio_start_dma_transfer(ch);
            DEBUG_PINS_CLR(audio_timing, 4);
        }
    }
#endif
}

int16_t *g_curr_sample = 0;
static uint32_t g_transmit_freq_base_hz = 87900000;

#define FM_BANDWIDTH        15000
#define CENTER_FREQ_OFFS    (FM_BANDWIDTH / 2)

static void __isr __time_critical_func(pwm_wrap_callback)()
{
    pwm_clear_irq(1);

    if (g_curr_sample) {
#if 0
        uint32_t freq = 87907525 + (*g_curr_sample++ / 4);  // bandwidth of FM broadcase is 15kHz, 65535 / 15000 = 4.369 = 4
        // uint32_t freq = 146520000 + (*g_curr_sample++ / 8);  // test on 2m ham radio band, 5kHz bandwidth, 65535 / 5000 = 13.1 = 8
        pico_fractional_pll_set_freq_u32(freq);
#elif 0
        uint32_t freq_28p4 = (87907525 << 4) + *g_curr_sample++;
        pico_fractional_pll_set_freq_28p4(freq_28p4);
#else
        uint32_t freq = g_transmit_freq_base_hz + CENTER_FREQ_OFFS + *g_curr_sample++;    // supposed to be (*g_curr_sample++ / 4), but most of the case it won't clipped
        pico_fractional_pll_set_freq_u32(freq);
#endif
    }
}

audio_format_t fm_transmitter_consumer_format;
audio_buffer_format_t fm_transmitter_consumer_buffer_format = {
        .format = &fm_transmitter_consumer_format,
        .sample_stride = sizeof(fm_transmitter_cmd_t)
};
audio_buffer_pool_t *fm_transmitter_consumer_pool;

// GPIO21 is the RF output pin, don't put a long wire as an antenna, you will be caught by FCC!!
#define CLOCK_GPOUT0_PIN    21

const audio_format_t *audio_fm_transmitter_setup(const audio_format_t *intended_audio_format, int32_t max_latency_ms,
                                               const audio_fm_transmitter_channel_config_t *channel_config0,
                                               uint32_t transmit_freq_base_hz,
                                               ...)
{
    va_list args;

    assert(max_latency_ms == -1); // not implemented yet
    __builtin_memset(&shared_state, 0, sizeof(shared_state));
    // init non zero members
#if !PICO_AUDIO_FM_TRANSMITTER_NOOP

    shared_state.channel_count = intended_audio_format->channel_count;
#if !PICO_AUDIO_FM_TRANSMITTER_ENABLE_NOISE_SHAPING
    fm_transmitter_consumer_format.format = AUDIO_BUFFER_FORMAT_PIO_FM_TRANSMITTER_CMD1;
    fm_transmitter_consumer_format.channel_count = 1;
#else
    fm_transmitter_consumer_format.format = AUDIO_BUFFER_FORMAT_PIO_FM_TRANSMITTER_CMD3;
    fm_transmitter_consumer_format.channel_count = 1;
#endif
#ifndef AUDIO_HALF_FREQ
    fm_transmitter_consumer_format.sample_freq = 22058;
#else
    fm_transmitter_consumer_format.sample_freq = 11029;
#endif

    g_transmit_freq_base_hz = transmit_freq_base_hz;

    for(int i = 0; i < shared_state.channel_count; i++)
    {
        shared_state.playback_buffer_pool[i] = audio_new_consumer_pool(&fm_transmitter_consumer_buffer_format,
                                                                       PICO_AUDIO_FM_TRANSMITTER_BUFFERS_PER_CHANNEL,
                                                                       PICO_AUDIO_FM_TRANSMITTER_BUFFER_SAMPLE_LENGTH);
    }
    __mem_fence_release();

    silence_buffer.buffer = pico_buffer_alloc(PICO_AUDIO_FM_TRANSMITTER_SILENCE_BUFFER_SAMPLE_LENGTH * sizeof(silence_cmd));
    for(int i = 0; i < PICO_AUDIO_FM_TRANSMITTER_SILENCE_BUFFER_SAMPLE_LENGTH; i++)
    {
        __builtin_memcpy((void *) (silence_buffer.buffer->bytes + i * sizeof(silence_cmd)), &silence_cmd,
                         sizeof(silence_cmd));
    }
    silence_buffer.sample_count = PICO_AUDIO_FM_TRANSMITTER_SILENCE_BUFFER_SAMPLE_LENGTH;
    silence_buffer.format = &fm_transmitter_consumer_buffer_format;

    va_start(args, transmit_freq_base_hz);
    uint offset = pio_add_program(audio_pio, &audio_program);

    const audio_fm_transmitter_channel_config_t *config = channel_config0;

    irq_add_shared_handler(DMA_IRQ_0 + PICO_AUDIO_FM_TRANSMITTER_DMA_IRQ, audio_fm_transmitter_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    for(int ch = 0; ch < shared_state.channel_count; ch++)
    {
        if (!config)
        {
            config = va_arg(args, const struct audio_fm_transmitter_channel_config *);
        }

        gpio_set_function(config->core.base_pin, GPIO_FUNC_PIOx);

        uint8_t sm = shared_state.pio_sm[ch] = config->core.pio_sm;
        pio_sm_claim(audio_pio, sm);

        pio_sm_config sm_config = audio_program_get_default_config(offset);
        sm_config_set_out_pins(&sm_config, config->core.base_pin, 1);
        sm_config_set_sideset_pins(&sm_config, config->core.base_pin);
        // disable auto-pull for !OSRE (which doesn't work with auto-pull)
        static_assert(CYCLES_PER_SAMPLE <= 18, "");
        sm_config_set_out_shift(&sm_config, true, false, CMD_BITS + CYCLES_PER_SAMPLE);
        pio_sm_init(audio_pio, sm, offset, &sm_config);

        pio_sm_set_consecutive_pindirs(audio_pio, sm, config->core.base_pin, 1, true);
        pio_sm_set_pins(audio_pio, sm, 0);

        // todo this should be part of sm_init
        pio_sm_exec(audio_pio, sm, pio_encode_jmp(offset + audio_entry_point)); // jmp to ep

        uint8_t dma_channel = config->core.dma_channel;
        dma_channel_claim(dma_channel);

        shared_state.dma_channel[ch] = dma_channel;

        dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);

        channel_config_set_dreq(&dma_config, DREQ_PIOx_TX0 + sm);
        dma_channel_configure(dma_channel,
                              &dma_config,
                              &audio_pio->txf[sm],  // dest
                              NULL, // src
                              0, // count
                              false // trigger
        );
        dma_irqn_set_channel_enabled(PICO_AUDIO_FM_TRANSMITTER_DMA_IRQ, dma_channel, 1);
        config = 0;
    }

    va_end(args);
#endif
#ifndef NDEBUG
    puts("PicoAudio: initialized\n");
#endif
    fm_transmitter_consumer_pool = (shared_state.playback_buffer_pool[0]); // forcing channel 0 to be consumer for now
    // todo we need to update this to what is exact
    return intended_audio_format;
}

void audio_fm_transmitter_set_enabled(bool enabled)
{
    if (enabled != audio_enabled)
    {
#ifndef NDEBUG
        if (enabled)
        {
            puts("Enabling PIO FM_TRANSMITTER audio\n");
        }
#endif
#if !PICO_AUDIO_FM_TRANSMITTER_NOOP
        irq_set_enabled(DMA_IRQ_0 + PICO_AUDIO_FM_TRANSMITTER_DMA_IRQ, enabled);

        if (enabled)
        {
            // todo this is wrong
            for(int ch = 0; ch < shared_state.channel_count; ch++)
            {
                audio_start_dma_transfer(ch);
            }
        }

        // todo need to start them in sync - need WAIT in program
        for(int ch = 0; ch < shared_state.channel_count; ch++)
        {
            pio_sm_set_enabled(audio_pio, shared_state.pio_sm[ch], enabled);
        }

        if (enabled) {
            uint32_t freq_low = g_transmit_freq_base_hz;
            uint32_t freq_high = g_transmit_freq_base_hz + FM_BANDWIDTH;
            uint32_t freq_center = g_transmit_freq_base_hz + CENTER_FREQ_OFFS;
            if (pico_fractional_pll_init(pll_sys, CLOCK_GPOUT0_PIN, freq_low, freq_high, GPIO_DRIVE_STRENGTH_12MA, GPIO_SLEW_RATE_FAST) != 0) {
                // ahhhh, the specified frequency range (freq_low ~ freq_high) cannot be within the two PLL divider values
                // therefore, the pico_fractional_pll cannot work!
                while (1) { }
            }
            pico_fractional_pll_set_freq_u32(freq_center);
            pico_fractional_pll_enable_output(true);

            uint slice_num = 1;
            // Mask our slice's IRQ output into the PWM block's single interrupt line,
            // and register our interrupt handler
            pwm_clear_irq(slice_num);
            pwm_set_irq_enabled(slice_num, true);
            irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_callback);
            irq_set_enabled(PWM_IRQ_WRAP, true);

            // 22050 or 24000, 441 or 480, 147 or 160
            // sys_clk 48,000,000Hz / 24,000Hz = 2,0000.0, 2,000.0 / 255 = 7.84, 2,000 / 10 = 200
            // sys_clk 48,000,000Hz / 22,050Hz = 2,176.87, 2,176.87 / 255 = 8.54, 2,176.87 / 9 = 241.875 = int:241 + frac:14

            pwm_config config = pwm_get_default_config();
            if (g_current_sample_freq == 44100) {
                pwm_config_set_clkdiv_int_frac(&config, 241, 14);
                pwm_config_set_wrap(&config, (9 - 1));
            }
            else {  // 48000
                pwm_config_set_clkdiv_int(&config, 200);
                pwm_config_set_wrap(&config, (10 - 1));
            }
            pwm_init(slice_num, &config, true);
        }
        else {
            irq_set_enabled(PWM_IRQ_WRAP, false);
            g_curr_sample = 0;
            pico_fractional_pll_enable_output(false);
            pico_fractional_pll_deinit();
        }
#endif

        audio_enabled = enabled;
    }
}

#pragma GCC push_options
#ifdef __arm__
// seems uber keen to inline audio_queue_samples which is large
#pragma GCC optimize("O1")
#endif

// void core1_worker()
// {
//     while (true)
//     {
//         sem_acquire_blocking(&sem_transfer_buffer_drain);
// //        audio_queue_samples(0, transfer_buffer, transfer_buffer_sample_count, 1, true);
//         sem_release(&sem_transfer_buffer_fill);
//     }
//     __builtin_unreachable();
// }

#pragma GCC pop_options

// bool audio_start_queue_work_on_core_1()
// {
//     if (!push_queuing_to_core1)
//     {
//         puts("In the spirit of the season, core 1 is helping out too...\n");
//         sem_init(&sem_transfer_buffer_drain, 0, 1);
//         // one fill is implicitly owned by the client application as it has a buffer
//         // (note the count here is actually the number of buffers the client has)
//         sem_init(&sem_transfer_buffer_fill, 2, 1);
//         multicore_launch_core1(core1_worker);
//         push_queuing_to_core1 = true;
//     }
//     return true;
// }

#endif

static struct producer_pool_blocking_give_connection producer_pool_blocking_give_connection_singleton = {
        .core = {
                .consumer_pool_take = consumer_pool_take_buffer_default,
                .consumer_pool_give = consumer_pool_give_buffer_default,
                .producer_pool_take = producer_pool_take_buffer_default,
        }
        // rest 0 initialized
};

bool audio_fm_transmitter_default_connect(audio_buffer_pool_t *producer_pool, bool dedicate_core_1)
{
    if (!dedicate_core_1)
    {
        printf("Connecting PIO FM_TRANSMITTER audio via 'blocking give'\n");
        assert(fm_transmitter_consumer_pool);
        assert(fm_transmitter_consumer_pool->format->channel_count == 1); // for now
        // todo oops this is pulling in everything!
        switch (producer_pool->format->format) {
            case AUDIO_BUFFER_FORMAT_PCM_S16:
                producer_pool_blocking_give_connection_singleton.core.producer_pool_give = producer_pool_blocking_give_to_fm_transmitter_s16;
                break;
            case AUDIO_BUFFER_FORMAT_PCM_S8:
                producer_pool_blocking_give_connection_singleton.core.producer_pool_give = producer_pool_blocking_give_to_fm_transmitter_s8;
                break;
            case AUDIO_BUFFER_FORMAT_PCM_U16:
                producer_pool_blocking_give_connection_singleton.core.producer_pool_give = producer_pool_blocking_give_to_fm_transmitter_s16;
                break;
            case AUDIO_BUFFER_FORMAT_PCM_U8:
                producer_pool_blocking_give_connection_singleton.core.producer_pool_give = producer_pool_blocking_give_to_fm_transmitter_s8;
                break;
            default:
                return false;
        }
        audio_complete_connection(&producer_pool_blocking_give_connection_singleton.core, producer_pool,
                                  fm_transmitter_consumer_pool);
        return true;
    }
    else
    {
        assert(false);
    }
    return false;
}
