/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_arch/io_timer_hw_description.h>

/* Timer allocation
 *
 * Motor 1 - PA2 - TIM2_CH1
 * Motor 2 - PA3 - TIM2_CH2
 * Motor 3 - PA4 - TIM2_CH3
 * Motor 4 - PA5 - TIM2_CH4
 *
 * Motor 5 - PE9 - TIM1_CH1
 * Motor 6 - PE11 - TIM1_CH2
 * Motor 7 - PE13 - TIM1_CH3
 * Motor 8 - PE14 - TIM1_CH4
 *
 * Servo 1 - PD12 - TIM4_CH1
 * Servo 2 - PD13 - TIM4_CH2
 * Servo 3 - PD14 - TIM4_CH3
 * Servo 4 - PD15 - TIM4_CH4
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer2, DMA{DMA::Index1, DMA::Stream1, DMA::Channel3}),
	initIOTimer(Timer::Timer1, DMA{DMA::Index2, DMA::Stream5, DMA::Channel6}),
	initIOTimer(Timer::Timer4),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel1}, {GPIO::PortA, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel2}, {GPIO::PortA, GPIO::Pin1}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel3}, {GPIO::PortA, GPIO::Pin2}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel4}, {GPIO::PortA, GPIO::Pin3}),

	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel1}, {GPIO::PortE, GPIO::Pin9}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortE, GPIO::Pin11}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortE, GPIO::Pin13}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel4}, {GPIO::PortE, GPIO::Pin14}),

	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel1}, {GPIO::PortD, GPIO::Pin12}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel4}, {GPIO::PortD, GPIO::Pin15}),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
