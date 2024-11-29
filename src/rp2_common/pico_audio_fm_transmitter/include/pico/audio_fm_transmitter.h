/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _PICO_AUDIO_FM_TRANSMITTER_H
#define _PICO_AUDIO_FM_TRANSMITTER_H

#include "pico/audio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ======================
// == CONFIG ============

#ifndef PICO_AUDIO_FM_TRANSMITTER_DMA_IRQ
#ifdef PICO_AUDIO_IRQ
#define PICO_AUDIO_FM_TRANSMITTER_DMA_IRQ PICO_AUDIO_DMA_IRQ
#else
#define PICO_AUDIO_FM_TRANSMITTER_DMA_IRQ 1
#endif
#endif

#ifndef PICO_AUDIO_FM_TRANSMITTER_PIO
#ifdef PICO_AUDIO_PIO
#define PICO_AUDIO_FM_TRANSMITTER_PIO PICO_AUDIO_PIO
#else
#define PICO_AUDIO_FM_TRANSMITTER_PIO 0
#endif
#endif

#if !(PICO_AUDIO_FM_TRANSMITTER_DMA_IRQ == 0 || PICO_AUDIO_FM_TRANSMITTER_DMA_IRQ == 1)
#error PICO_AUDIO_FM_TRANSMITTER_DMA_IRQ must be 0 or 1
#endif

#if !(PICO_AUDIO_FM_TRANSMITTER_PIO == 0 || PICO_AUDIO_FM_TRANSMITTER_PIO == 1)
#error PICO_AUDIO_FM_TRANSMITTER_PIO ust be 0 or 1
#endif

#ifndef PICO_AUDIO_FM_TRANSMITTER_MAX_CHANNELS
#ifdef PICO_AUDIO_MAX_CHANNELS
#define PICO_AUDIO_FM_TRANSMITTER_MAX_CHANNELS PICO_AUDIO_MAX_CHANNELS
#else
#define PICO_AUDIO_FM_TRANSMITTER_MAX_CHANNELS 2u
#endif
#endif

#ifndef PICO_AUDIO_FM_TRANSMITTER_BUFFERS_PER_CHANNEL
#ifdef PICO_AUDIO_BUFFERS_PER_CHANNEL
#define PICO_AUDIO_FM_TRANSMITTER_BUFFERS_PER_CHANNEL PICO_AUDIO_BUFFERS_PER_CHANNEL
#else
#define PICO_AUDIO_FM_TRANSMITTER_BUFFERS_PER_CHANNEL 3u
#endif
#endif

#ifndef PICO_AUDIO_FM_TRANSMITTER_BUFFER_SAMPLE_LENGTH
#ifdef PICO_AUDIO_BUFFER_SAMPLE_LENGTH
#define PICO_AUDIO_FM_TRANSMITTER_BUFFER_SAMPLE_LENGTH PICO_AUDIO_BUFFER_SAMPLE_LENGTH
#else
#define PICO_AUDIO_FM_TRANSMITTER_BUFFER_SAMPLE_LENGTH 576u
#endif
#endif

#ifndef PICO_AUDIO_FM_TRANSMITTER_SILENCE_BUFFER_SAMPLE_LENGTH
#ifdef PICO_AUDIO_FM_TRANSMITTER_SILENCE_BUFFER_SAMPLE_LENGTH
#define PICO_AUDIO_FM_TRANSMITTER_SILENCE_BUFFER_SAMPLE_LENGTH PICO_AUDIO_SILENCE_BUFFER_SAMPLE_LENGTH
#else
#define PICO_AUDIO_FM_TRANSMITTER_SILENCE_BUFFER_SAMPLE_LENGTH 256u
#endif
#endif

// Enable noise shaping when super-sampling
//
// This allows for runtime selection of noise shaping or not, however having the compile
// time definition requires triple the pico_audio buffer RAM usage at runtime, and leads to marginally
// slower code in general.
#ifndef PICO_AUDIO_FM_TRANSMITTER_ENABLE_NOISE_SHAPING
#define PICO_AUDIO_FM_TRANSMITTER_ENABLE_NOISE_SHAPING 0
#endif

#ifndef PICO_AUDIO_FM_TRANSMITTER_L_PIN
#define PICO_AUDIO_FM_TRANSMITTER_L_PIN 0
#endif

#ifndef PICO_AUDIO_FM_TRANSMITTER_R_PIN
#define PICO_AUDIO_FM_TRANSMITTER_R_PIN 1
#endif

#ifndef PICO_AUDIO_FM_TRANSMITTER_MONO_PIN
#define PICO_AUDIO_FM_TRANSMITTER_MONO_PIN PICO_AUDIO_FM_TRANSMITTER_L_PIN
#endif

#ifndef PIO_AUDIO_FM_TRANSMITTER_INTERP_SAVE
#define PIO_AUDIO_FM_TRANSMITTER_INTERP_SAVE 1
#endif

// Allow use of pico_audio driver without actually doing anything much
#ifndef PICO_AUDIO_FM_TRANSMITTER_NOOP
#ifdef PICO_AUDIO_NOOP
#define PICO_AUDIO_FM_TRANSMITTER_NOOP PICO_AUDIO_NOOP
#else
#define PICO_AUDIO_FM_TRANSMITTER_NOOP 0
#endif
#endif

/** \file audio_fm_transmitter.h
 *  \defgroup pico_audio_fm_transmitter pico_audio_fm_transmitter
 *  FM Transmitter audio output (with optional noise shaping and error diffusion) using the PIO
 *
 * This library uses the \ref hardware_pio system to implement a FM Transmitter audio interface
 *
 * \todo Must be more we need to say here.
 * \todo certainly need an example
 *
 */

// todo we need a place to register these or just allow them to overlap, or base them on a FOURCC - this is just made up
#define AUDIO_BUFFER_FORMAT_PIO_FM_TRANSMITTER_FIRST 1000
#define AUDIO_BUFFER_FORMAT_PIO_FM_TRANSMITTER_CMD1 (AUDIO_BUFFER_FORMAT_PIO_FM_TRANSMITTER_FIRST)
#define AUDIO_BUFFER_FORMAT_PIO_FM_TRANSMITTER_CMD3 (AUDIO_BUFFER_FORMAT_PIO_FM_TRANSMITTER_FIRST+1)

typedef struct __packed audio_fm_transmitter_channel_config {
    pio_audio_channel_config_t core;
    uint8_t pattern;
} audio_fm_transmitter_channel_config_t;

// can copy this to modify just the pin
extern const audio_fm_transmitter_channel_config_t default_left_channel_config;
extern const audio_fm_transmitter_channel_config_t default_right_channel_config;
extern const audio_fm_transmitter_channel_config_t default_mono_channel_config;

/*! \brief
 *  \ingroup pico_audio_fm_transmitter
 *  \todo
 *
 * max_latency_ms may be -1 (for don't care)
 * \param intended_audio_format
 * \param max_latency_ms
 * \param channel_config0
 * \param ...
 * \return
 */
extern const audio_format_t *
audio_fm_transmitter_setup(const audio_format_t *intended_audio_format, int32_t max_latency_ms,
                    const audio_fm_transmitter_channel_config_t *channel_config0, uint32_t transmit_freq_base_hz, ...);

/*! \brief
 *  \ingroup pico_audio_fm_transmitter
 *  \todo
 *
 * \param producer_pool
 * \param dedicate_core_1
 * attempt a default mapping of producer buffers to pio fm_transmitter pico_audio output
 * dedicate_core_1 to have core 1 set aside entirely to do work offloading as much stuff from the producer side as possible
 * todo also allow IRQ handler to do it I guess
 */
extern bool audio_fm_transmitter_default_connect(audio_buffer_pool_t *producer_pool, bool dedicate_core_1);

/*! \brief
 *  \ingroup pico_audio_fm_transmitter
 *  \todo
 *
 * \param enable true to enable the FM Transmitter audio, false to disable
 */
extern void audio_fm_transmitter_set_enabled(bool enabled);

/*! \brief Set the FM_TRANSMITTER correction mode
 *  \ingroup pico_audio_fm_transmitter
 *
 * \param mode \todo
 */
extern bool audio_fm_transmitter_set_correction_mode(enum audio_correction_mode mode);

/*! \brief Get the FM Transmitter correction mode
 *  \ingroup pico_audio_fm_transmitter
 *
 * \return  mode
 */
extern enum audio_correction_mode audio_fm_transmitter_get_correction_mode();

#ifdef __cplusplus
}
#endif

#endif //_PIO_AUDIO_FM_TRANSMITTER_H
