/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <inttypes.h>

#include "pico.h"

#include "pico/stdlib.h"
#include "pico/sleep.h"

#include "hardware/pll.h"
#include "hardware/regs/clocks.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "hardware/xosc.h"
#include "hardware/rosc.h"
#include "hardware/regs/io_bank0.h"
// For __wfi
#include "hardware/sync.h"
#include "pico/runtime_init.h"

#ifdef __riscv
#include "hardware/riscv.h"
#else
// For scb_hw so we can enable deep sleep
#include "hardware/structs/scb.h"
#endif

#if !PICO_RP2040
#include "hardware/powman.h"
#endif

// The difference between sleep and dormant is that ALL clocks are stopped in dormant mode,
// until the source (either xosc or rosc) is started again by an external event.
// In sleep mode some clocks can be left running controlled by the SLEEP_EN registers in the clocks
// block. For example you could keep clk_rtc running. Some destinations (proc0 and proc1 wakeup logic)
// can't be stopped in sleep mode otherwise there wouldn't be enough logic to wake up again.

static dormant_source_t _dormant_source;

// Static variables for internal use by the GPIO IRQ handler
static volatile bool __sleep_pin_triggered;
static hardware_alarm_callback_t __sleep_user_callback;  // hardware alarm user cb (sleep_for paths)
static aon_timer_alarm_handler_t __sleep_user_aon_cb;    // AON timer user cb (sleep_until paths)
static int __sleep_alarm_num;
static uint __sleep_gpio_pin;
static uint32_t __sleep_event;

// Distinguish which timing mechanism is active for the current sleep
static bool __sleep_uses_hardware_alarm = false; // true: hardware alarm; false: AON timer

bool dormant_source_valid(dormant_source_t dormant_source)
{
    switch (dormant_source) {
        case DORMANT_SOURCE_XOSC:
            return true;
        case DORMANT_SOURCE_ROSC:
            return true;
#if !PICO_RP2040
        case DORMANT_SOURCE_LPOSC:
            return true;
#endif
        default:
            return false;
    }
}

// In order to go into dormant mode we need to be running from a stoppable clock source:
// either the xosc or rosc with no PLLs running. This means we disable the USB and ADC clocks
// and all PLLs
void sleep_run_from_dormant_source(dormant_source_t dormant_source) {
    assert(dormant_source_valid(dormant_source));
    _dormant_source = dormant_source;

    uint src_hz;
    uint clk_ref_src;
    switch (dormant_source) {
        case DORMANT_SOURCE_XOSC:
            src_hz = XOSC_HZ;
            clk_ref_src = CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC;
            break;
        case DORMANT_SOURCE_ROSC:
            src_hz = 6500 * KHZ; // todo
            clk_ref_src = CLOCKS_CLK_REF_CTRL_SRC_VALUE_ROSC_CLKSRC_PH;
            break;
#if !PICO_RP2040
        case DORMANT_SOURCE_LPOSC:
            src_hz = 32 * KHZ;
            clk_ref_src = CLOCKS_CLK_REF_CTRL_SRC_VALUE_LPOSC_CLKSRC;
            break;
#endif
        default:
            hard_assert(false);
    }

    // CLK_REF = XOSC or ROSC
    clock_configure(clk_ref,
                    clk_ref_src,
                    0, // No aux mux
                    src_hz,
                    src_hz);

    // CLK SYS = CLK_REF
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF,
                    0, // Using glitchless mux
                    src_hz,
                    src_hz);

    // CLK ADC = 0MHz
    clock_stop(clk_adc);
    clock_stop(clk_usb);
#if PICO_RP2350
    clock_stop(clk_hstx);
#endif

#if PICO_RP2040
    // CLK RTC = ideally XOSC (12MHz) / 256 = 46875Hz but could be rosc
    uint clk_rtc_src = (dormant_source == DORMANT_SOURCE_XOSC) ?
                       CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC :
                       CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH;

    clock_configure(clk_rtc,
                    0, // No GLMUX
                    clk_rtc_src,
                    src_hz,
                    46875);
#endif

    // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    src_hz,
                    src_hz);

    pll_deinit(pll_sys);
    pll_deinit(pll_usb);

    // Assuming both xosc and rosc are running at the moment
    if (dormant_source == DORMANT_SOURCE_XOSC) {
        // Can disable rosc
        rosc_disable();
    } else {
        // Can disable xosc
        xosc_disable();
    }

    // Reconfigure uart with new clocks
    setup_default_uart();
}

static void processor_deep_sleep(void) {
    // Enable deep sleep at the proc
#ifdef __riscv
    uint32_t bits = RVCSR_MSLEEP_POWERDOWN_BITS;
    if (!get_core_num()) {
        bits |= RVCSR_MSLEEP_DEEPSLEEP_BITS;
    }
    riscv_set_csr(RVCSR_MSLEEP_OFFSET, bits);
#else
    scb_hw->scr |= ARM_CPU_PREFIXED(SCR_SLEEPDEEP_BITS);
#endif
}

static void __sleep_gpio_irq_callback(uint gpio, uint32_t events) {
    if ((gpio == __sleep_gpio_pin) && (events & __sleep_event)) {
        __sleep_pin_triggered = true;

        // Cancel whichever timing mechanism was armed
        if (__sleep_uses_hardware_alarm) {
            // Hardware alarm path (sleep_goto_sleep_for[_or_pin])
            hardware_alarm_set_callback(__sleep_alarm_num, NULL);
            hardware_alarm_unclaim(__sleep_alarm_num);
            // Run the user callback (if provided) in IRQ context
            if (__sleep_user_callback) {
                __sleep_user_callback(__sleep_alarm_num);
            }
        } else {
            // AON timer path (sleep_goto_sleep_until[_or_pin])
            aon_timer_disable_alarm();
            // Optionally invoke the user's AON callback on pin wake as well
            if (__sleep_user_aon_cb) {
                __sleep_user_aon_cb();
            }
        }

        // Clear the irq so we can go back to sleep/dormant again if we want
        gpio_set_irq_enabled(gpio, __sleep_event, false);
        gpio_acknowledge_irq(gpio, __sleep_event);
    }
}

void sleep_goto_sleep_until(struct timespec *ts, aon_timer_alarm_handler_t callback)
{

    // We should have already called the sleep_run_from_dormant_source function
    // This is only needed for dormancy although it saves power running from xosc while sleeping
    //assert(dormant_source_valid(_dormant_source));

#if PICO_RP2040
    clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_RTC_RTC_BITS;
    clocks_hw->sleep_en1 = 0x0;
#else
    clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_REF_POWMAN_BITS;
    clocks_hw->sleep_en1 = 0x0;
#endif

    __sleep_uses_hardware_alarm = false;
    __sleep_user_aon_cb = callback;
    __sleep_user_callback = NULL;

    aon_timer_enable_alarm(ts, callback, false);

    stdio_flush();

    // Enable deep sleep at the proc
    processor_deep_sleep();

    // Go to sleep
    __wfi();
}

bool sleep_goto_sleep_until_or_pin(struct timespec *ts,
                                   aon_timer_alarm_handler_t callback,
                                   uint gpio_pin, bool edge, bool high) {
    assert(dormant_source_valid(_dormant_source));

    // 1) Figure out the GPIO wake event
    bool level = !edge;
    bool low   = !high;
    uint32_t gpio_event = 0;
    if (level) {
        gpio_event = low ? GPIO_IRQ_LEVEL_LOW : GPIO_IRQ_LEVEL_HIGH;
    } else {
        gpio_event = high ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL;
    }

    // 2) Leave only AON/RTC clock running (and prepare powman tick if present)
  #if PICO_RP2040
    clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_RTC_RTC_BITS;
    clocks_hw->sleep_en1 = 0;
  #else
    if (_dormant_source == DORMANT_SOURCE_LPOSC) {
        uint64_t restore_ms = powman_timer_get_ms();
        powman_timer_set_1khz_tick_source_lposc();
        powman_timer_set_ms(restore_ms);
    }
    clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_REF_POWMAN_BITS;
    clocks_hw->sleep_en1 = 0;
  #endif

    // This path uses the AON timer (not hardware alarm)
    __sleep_uses_hardware_alarm = false;
    __sleep_user_aon_cb = callback;
    __sleep_user_callback = NULL;
    __sleep_pin_triggered = false;
    __sleep_gpio_pin = gpio_pin;
    __sleep_event = gpio_event;

    // 3) Arm the AON timer (allowing low power wake)
    aon_timer_enable_alarm(ts, callback, true);

    // 4) Configure the GPIO as a second wake source
    gpio_init(gpio_pin);
    gpio_set_input_enabled(gpio_pin, true);
    gpio_set_dormant_irq_enabled(gpio_pin, gpio_event, true);
    gpio_set_irq_enabled_with_callback(
        gpio_pin, gpio_event, true, &__sleep_gpio_irq_callback);

    stdio_flush();

    // 5) Enter dormant (all stoppable clocks off)
    processor_deep_sleep();
    _go_dormant();

    // 6) Woke up — first clear the GPIO IRQ
    gpio_set_irq_enabled(gpio_pin, gpio_event, false);
    gpio_set_dormant_irq_enabled(gpio_pin, gpio_event, false);
    gpio_acknowledge_irq(gpio_pin, gpio_event);
    gpio_set_input_enabled(gpio_pin, false);

    // 7) If we woke on the pin, cancel the AON timer before it fires
    bool woke_from_pin = __sleep_pin_triggered;
    if (!woke_from_pin && !level) {
        woke_from_pin = gpio_get_irq_event_mask(gpio_pin) & gpio_event;
    }

    if (woke_from_pin) {
        aon_timer_disable_alarm(); // Ensure the AON alarm won't fire after the pin wake
        if (__sleep_user_aon_cb && !__sleep_pin_triggered) {
            __sleep_user_aon_cb();
        }
    }

    return true;
}

bool sleep_goto_sleep_for(uint32_t delay_ms, hardware_alarm_callback_t callback)
{
    // We should have already called the sleep_run_from_dormant_source function
    // This is only needed for dormancy although it saves power running from xosc while sleeping
    //assert(dormant_source_valid(_dormant_source));

    // Turn off all clocks except for the timer
    clocks_hw->sleep_en0 = 0x0;
#if PICO_RP2040
    clocks_hw->sleep_en1 = CLOCKS_SLEEP_EN1_CLK_SYS_TIMER_BITS;
#elif PICO_RP2350
    clocks_hw->sleep_en1 = CLOCKS_SLEEP_EN1_CLK_REF_TICKS_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_TIMER0_BITS;
#else
#error Unknown processor
#endif

    __sleep_uses_hardware_alarm = true;
    __sleep_user_callback = callback;
    __sleep_user_aon_cb = NULL;

    int alarm_num = hardware_alarm_claim_unused(true);
    __sleep_alarm_num = alarm_num;

    hardware_alarm_set_callback(alarm_num, callback);
    absolute_time_t t = make_timeout_time_ms(delay_ms);
    if (hardware_alarm_set_target(alarm_num, t)) {
        hardware_alarm_set_callback(alarm_num, NULL);
        hardware_alarm_unclaim(alarm_num);
        return false;
    }

    stdio_flush();

    // Enable deep sleep at the proc
    processor_deep_sleep();

    // Go to sleep
    __wfi();
    return true;
}

bool sleep_goto_sleep_for_or_pin(uint32_t delay_ms, hardware_alarm_callback_t callback,
                                 uint gpio_pin, bool edge, bool high) {
    bool level = !edge;
    uint32_t event = 0;
    if (level) {
        event = high ? GPIO_IRQ_LEVEL_HIGH : GPIO_IRQ_LEVEL_LOW;
    } else {
        event = high ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL;
    }

    // Turn off all clocks except for the timer
    clocks_hw->sleep_en0 = 0x0;
#if PICO_RP2040
    clocks_hw->sleep_en1 = CLOCKS_SLEEP_EN1_CLK_SYS_TIMER_BITS;
#elif PICO_RP2350
    clocks_hw->sleep_en1 = CLOCKS_SLEEP_EN1_CLK_REF_TICKS_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_TIMER0_BITS;
#else
#error Unknown processor
#endif

    __sleep_uses_hardware_alarm = true;
    __sleep_user_callback = callback;
    __sleep_user_aon_cb = NULL;

    int alarm_num = hardware_alarm_claim_unused(true);
    __sleep_alarm_num = alarm_num;

    hardware_alarm_set_callback(alarm_num, callback);
    absolute_time_t t = make_timeout_time_ms(delay_ms);
    if (hardware_alarm_set_target(alarm_num, t)) {
        hardware_alarm_set_callback(alarm_num, NULL);
        hardware_alarm_unclaim(alarm_num);
        return false;
    }

    // Configure the GPIO as a wake-up source
    gpio_set_input_enabled(gpio_pin, true);
    __sleep_gpio_pin = gpio_pin;
    __sleep_event = event;
    __sleep_pin_triggered = false;
    gpio_set_irq_enabled_with_callback(gpio_pin, event, true, &__sleep_gpio_irq_callback);

    stdio_flush();
    processor_deep_sleep();
    __wfi();

    // Clear the irq so we can go back to dormant mode again if we want
    gpio_set_irq_enabled(gpio_pin, event, false);
    gpio_acknowledge_irq(gpio_pin, event);
    if (!__sleep_pin_triggered) {
        hardware_alarm_set_callback(alarm_num, NULL);
        hardware_alarm_unclaim(alarm_num);
    }

    return true;
}

static void _go_dormant(void) {
    assert(dormant_source_valid(_dormant_source));

    if (_dormant_source == DORMANT_SOURCE_XOSC) {
        xosc_dormant();
    } else {
        rosc_set_dormant();
    }
}

void sleep_goto_dormant_until(struct timespec *ts, aon_timer_alarm_handler_t callback)   {
    // We should have already called the sleep_run_from_dormant_source function

#if PICO_RP2040
    clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_RTC_RTC_BITS;
    clocks_hw->sleep_en1 = 0x0;
#else
    assert(_dormant_source == DORMANT_SOURCE_LPOSC);
    uint64_t restore_ms = powman_timer_get_ms();
    powman_timer_set_1khz_tick_source_lposc();
    powman_timer_set_ms(restore_ms);

    clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_REF_POWMAN_BITS;
    clocks_hw->sleep_en1 = 0x0;
#endif

    // AON path
    __sleep_uses_hardware_alarm = false;
    __sleep_user_aon_cb = callback;
    __sleep_user_callback = NULL;

    // Set the AON timer to wake up the proc from dormant mode
    aon_timer_enable_alarm(ts, callback, true);

    stdio_flush();

    // Enable deep sleep at the proc
    processor_deep_sleep();

    // Go dormant
    _go_dormant();
}

void sleep_goto_dormant_until_pin(uint gpio_pin, bool edge, bool high) {
    bool low = !high;
    bool level = !edge;

    // Configure the appropriate IRQ at IO bank 0
    assert(gpio_pin < NUM_BANK0_GPIOS);

    uint32_t event = 0;

    if (level && low) event = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_LEVEL_LOW_BITS;
    if (level && high) event = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_LEVEL_HIGH_BITS;
    if (edge && high) event = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_HIGH_BITS;
    if (edge && low) event = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS;

    gpio_init(gpio_pin);
    gpio_set_input_enabled(gpio_pin, true);
    gpio_set_dormant_irq_enabled(gpio_pin, event, true);

    _go_dormant();
    // Execution stops here until woken up

    // Clear the irq so we can go back to dormant mode again if we want
    gpio_acknowledge_irq(gpio_pin, event);
    gpio_set_input_enabled(gpio_pin, false);
}

// To be called after waking up from sleep/dormant mode to restore system clocks properly
void sleep_power_up(void)
{
    // Re-enable the ring oscillator, which will essentially kickstart the proc
    rosc_enable();

    // Reset the sleep enable register so peripherals and other hardware can be used
    clocks_hw->sleep_en0 |= ~(0u);
    clocks_hw->sleep_en1 |= ~(0u);

    // Restore all clocks
    clocks_init();

#if PICO_RP2350
    // make powerman use xosc again
    uint64_t restore_ms = powman_timer_get_ms();
    powman_timer_set_1khz_tick_source_xosc();
    powman_timer_set_ms(restore_ms);
#endif

    // UART needs to be reinitialised with the new clock frequencies for stable output
    setup_default_uart();
}
