/*

  odometer.c - axis odometers including run time + spindle run time

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

// The whole plugin is compiled out unless the driver enables it.
#if ODOMETER_ENABLE

#include <string.h>
#include <stdio.h>

#include "grbl/system.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/task.h"

// Persistent odometer record, stored as a single block in NVS (EEPROM/FRAM).
typedef struct {
    uint64_t spindle;           // Accumulated spindle run time in milliseconds (ticks).
    float distance[N_AXIS];     // Accumulated travel per axis in mm.
} odometer_data_t;

// Step pulses counted per axis since the last time they were converted to distance.
static uint32_t steps[N_AXIS] = {0};
// States in which travel is recorded. Jogging is deliberately excluded.
#define ODOMETER_COUNT_STATES (STATE_CYCLE|STATE_HOMING)
// Set by the state change handler: true only while step pulses should be counted.
static volatile bool count_steps = false;
// Set by the step pulse handler when counted motion has occurred and the data needs saving.
static volatile bool odometer_changed = false;
// NVS addresses of the current and the previous (backup) odometer records.
static uint32_t odometers_address, odometers_address_prv;
// Working copies of the current and previous odometer records.
static odometer_data_t odometers, odometers_prv;
// Local copy of the physical NVS driver interface (read/write functions etc.).
static nvs_io_t nvs;
// Saved pointers to the original handlers that this plugin chains onto.
static stepper_pulse_start_ptr stepper_pulse_start;
static on_state_change_ptr on_state_change;
static on_spindle_selected_ptr on_spindle_selected;
static spindle_set_state_ptr spindle_set_state_;
// The spindle that currently has the run time hook installed (only one at a time).
static spindle_ptrs_t *hooked_spindle = NULL;
static settings_changed_ptr settings_changed;
static on_report_options_ptr on_report_options;

typedef union {
    uint8_t value;
    struct {
        uint8_t enable :1,  // maintenance tracking enabled
                unused :7;
    };
} odometer_flags_t;

// Plugin settings, stored in their own NVS block (allocated with nvs_alloc).
typedef struct {
    odometer_flags_t flags;
    uint16_t maint_distance[N_AXIS];    // distance in metres before maintenance is required, 0 = not used
} odometer_settings_t;

static nvs_address_t nvs_addr;
static odometer_settings_t config;

/* Setting IDs. Change these if they clash with another plugin in your build. */
#define SETTING_MAINT_REQ       1500
#define SETTING_X_MAINT         1501
#define SETTING_Y_MAINT         1502
#define SETTING_Z_MAINT         1503
#define SETTING_A_MAINT         1504
#define SETTING_B_MAINT         1505

// Each axis setting points at its own element, only for axes present in the build.
static const setting_detail_t plugin_settings[] = {
    { SETTING_MAINT_REQ, Group_Stepper, "Odometer Maintenance Tracking", NULL, Format_XBitfield, "Enable", NULL, NULL, Setting_NonCore, &config.flags.value },
    { SETTING_X_MAINT,   Group_Stepper, "Distance before X requires Maintenance", "m", Format_Int16, "####0", "0", "65535", Setting_NonCore, &config.maint_distance[X_AXIS] },
    { SETTING_Y_MAINT,   Group_Stepper, "Distance before Y requires Maintenance", "m", Format_Int16, "####0", "0", "65535", Setting_NonCore, &config.maint_distance[Y_AXIS] },
    { SETTING_Z_MAINT,   Group_Stepper, "Distance before Z requires Maintenance", "m", Format_Int16, "####0", "0", "65535", Setting_NonCore, &config.maint_distance[Z_AXIS] },
    { SETTING_A_MAINT,   Group_Stepper, "Distance before A requires Maintenance", "m", Format_Int16, "####0", "0", "65535", Setting_NonCore, &config.maint_distance[A_AXIS] },
    { SETTING_B_MAINT,   Group_Stepper, "Distance before B requires Maintenance", "m", Format_Int16, "####0", "0", "65535", Setting_NonCore, &config.maint_distance[B_AXIS] },
};

/* --- Settings save / restore / load --- */

static void odometer_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void odometer_settings_restore (void)
{
    memset(&config, 0, sizeof(config)); // disabled by default, user must enable

    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void odometer_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        odometer_settings_restore();
}

// Copies the current odometer record to dst with interrupts briefly disabled.
// odometers.spindle is 64 bit and is updated from the spindle handler, which can
// run in interrupt context. On a 32 bit MCU that update is not atomic, so reading
// the record directly could give a torn value, or data that does not match its
// CRC when written to NVS. Call from foreground code only.
static void odometers_snapshot (odometer_data_t *dst)
{
    hal.irq_disable();
    memcpy(dst, &odometers, sizeof(odometer_data_t));
    hal.irq_enable();
}

// Writes a consistent snapshot of the current odometer record to NVS.
// The (slow) NVS transfer is done from the local copy with interrupts enabled.
static void odometers_save (void)
{
    odometer_data_t snapshot;

    odometers_snapshot(&snapshot);
    nvs.memcpy_to_nvs(odometers_address, (uint8_t *)&snapshot, sizeof(odometer_data_t), true);
}

// Step pulse start hook: counts a step for every axis that is pulsed (only
// in CYCLE and HOMING), then passes control on to the original handler.
static void stepperPulseStart (stepper_t *stepper)
{
    if(count_steps) {

        // Flag that there is new data to be accounted for and saved.
        odometer_changed = true;

        // Count one step for each axis that has its step output active.
        if(stepper->step_out.x)
            steps[X_AXIS]++;

        if(stepper->step_out.y)
            steps[Y_AXIS]++;

        if(stepper->step_out.z)
            steps[Z_AXIS]++;

        // Optional axes, only present when configured for the build.
#ifdef A_AXIS
        if(stepper->step_out.a)
            steps[A_AXIS]++;
#endif

#ifdef B_AXIS
        if(stepper->step_out.b)
            steps[B_AXIS]++;
#endif

#ifdef C_AXIS
        if(stepper->step_out.c)
            steps[C_AXIS]++;
#endif
    }

    // Chain to the original step pulse start handler. This must stay outside
    // the if above, otherwise the axes would not move when counting is off.
    stepper_pulse_start(stepper);
}

// System state change hook: enables step counting in CYCLE and HOMING, and
// converts counted steps to distance when the machine leaves those states.
void onStateChanged (sys_state_t state)
{
    // Only count steps while in a state whose travel should be recorded.
    count_steps = !!(state & ODOMETER_COUNT_STATES);

    // If not in a counting state and counted motion has taken place since the last update, account for it.
    if(!count_steps && odometer_changed) {

        uint_fast8_t idx = N_AXIS;

        odometer_changed = false;

        // Walk the axes from last to first, converting steps to mm and resetting the counters.
        do {
            if(steps[--idx]) {
                odometers.distance[idx] += (float)steps[idx] / settings.axis[idx].steps_per_mm;
                steps[idx] = 0;
            }
        } while(idx);

        // Save the updated record to NVS.
        odometers_save();
    }

    // Chain to the previously registered state change handler, if any.
    if(on_state_change)
        on_state_change(state);
}

// Called by foreground process.
// Deferred NVS write, scheduled from the spindle handler so the write
// does not happen in interrupt context.
static void odometers_write (void *data)
{
    odometers_save();
}

// Spindle set state hook: wraps the spindle driver's set_state function to
// time how long the spindle is running.
// The timer starts once, when the spindle is commanded on (speed is ignored),
// and keeps running through any further speed or direction changes.
// The elapsed time is added to the total, and saved, only when the spindle
// is commanded off.
ISR_CODE static void ISR_FUNC(onSpindleSetState)(spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    // Tick count captured when the spindle started running.
    static uint32_t ms = 0;
    // True while the spindle timer is running (separate from ms, as a tick count of 0 is valid).
    static bool running = false;

    // Let the original spindle driver apply the new state first.
    spindle_set_state_(spindle, state, rpm);

    if(state.on) {
        // Spindle commanded on: start the timer only if not already running.
        if(!running) {
            ms = hal.get_elapsed_ticks();
            running = true;
        }
    } else if(running) {
        // Spindle commanded off: add the elapsed time to the total.
        odometers.spindle += (hal.get_elapsed_ticks() - ms);
        running = false;
        // Write odometer data in foreground process.
        task_add_immediate(odometers_write, NULL);
    }
}

// Spindle selected hook: installs the run time wrapper on whichever spindle is
// selected, regardless of its id. Only one spindle is assumed to run at a time,
// so a single saved original set_state pointer is used and the hook is moved
// from the previously selected spindle to the new one.
static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    if(spindle != hooked_spindle) {

        // Give the previously hooked spindle its own set_state function back.
        if(hooked_spindle && hooked_spindle->set_state == onSpindleSetState)
            hooked_spindle->set_state = spindle_set_state_;

        hooked_spindle = NULL;

        // Remember the original set_state function of the new spindle and substitute ours.
        if(spindle->set_state != onSpindleSetState) {
            spindle_set_state_ = spindle->set_state;
            spindle->set_state = onSpindleSetState;
            hooked_spindle = spindle;
        }
    }

    // Chain to the previously registered handler, if any.
    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

// Reclaim entry points that may have been changed on settings change.
// The settings change handler may reinstall the stepper driver's own pulse
// start function, so this plugin hooks it again if it has been replaced.
static void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    // Let the original handler do its work first.
    settings_changed(settings, changed);

    // If our step pulse hook was overwritten, save the new original and reinstall the hook.
    if(hal.stepper.pulse_start != stepperPulseStart) {
        stepper_pulse_start = hal.stepper.pulse_start;
        hal.stepper.pulse_start = stepperPulseStart;
    }
}

// Clears the current odometer record in memory and NVS.
// If backup is true the current values are first copied to the "previous" record.
static void odometer_data_reset (bool backup)
{
    if(backup) {
        // Save a consistent snapshot of the current values to the previous record, in memory and in NVS.
        odometers_snapshot(&odometers_prv);
        nvs.memcpy_to_nvs(odometers_address_prv, (uint8_t *)&odometers_prv, sizeof(odometer_data_t), true);
    }
    // Zero the current record (with interrupts disabled so the spindle handler cannot interleave) and write it to NVS.
    hal.irq_disable();
    memset(&odometers, 0, sizeof(odometer_data_t));
    hal.irq_enable();
    odometers_save();
}

// Sends the given odometer record to the user as a series of plain messages.
static void odometers_report (odometer_data_t *odometers)
{
    char buf[40];
    uint_fast8_t idx;
    // Convert spindle time from milliseconds to hours and minutes.
    uint32_t hr = odometers->spindle / 3600000, min = (odometers->spindle / 60000) % 60;

    sprintf(buf, "SPINDLEHRS %ld:%.2ld", hr, min);
    report_message(buf, Message_Plain);

    // One line per axis with the travelled distance.
    for(idx = 0 ; idx < N_AXIS ; idx++) {
        sprintf(buf, "ODOMETER%s %s", axis_letter[idx], ftoa(odometers->distance[idx] / 1000.0f, 1)); // meters
        report_message(buf, Message_Plain);
    }
}

// Handler for the $ODOMETERS system command.
//   $ODOMETERS      - report current values
//   $ODOMETERS=PREV - report the previous (backed up) values
//   $ODOMETERS=RST  - back up current values and clear them
static status_code_t odometer_command (sys_state_t state, char *args)
{
    // Default to "not handled" so unknown arguments fall through.
    status_code_t retval = Status_Unhandled;

    // No argument: report a consistent snapshot of the current odometers.
    if(args == NULL) {
        odometer_data_t snapshot;
        odometers_snapshot(&snapshot);
        odometers_report(&snapshot);
        retval = Status_OK;
    } else {

        // Make the argument upper case for comparison.
        strcaps(args);

        // PREV: read the backup record from NVS and report it if it is valid.
        if(!strcmp(args, "PREV")) {
            if(nvs.memcpy_from_nvs((uint8_t *)&odometers_prv, odometers_address_prv, sizeof(odometer_data_t), true) == NVS_TransferResult_OK)
                odometers_report(&odometers_prv);
            else
                report_message("Previous odometer values not available", Message_Warning);
            retval = Status_OK;
        }

        // RST: back up the current record, then clear it.
        if(!strcmp(args, "RST")) {
            odometer_data_reset(true);
            retval = Status_OK;
        }
    }

    return retval;
}

// Command table entry for $ODOMETERS, including its help text.
const sys_command_t odometer_command_list[] = {
    {"ODOMETERS", odometer_command, {}, {
        .str = "$ODOMETERS - list odometer log"
     ASCII_EOL "$ODOMETERS=PREV - list previous odometer log when available"
     ASCII_EOL "$ODOMETERS=RST - copy current log to previous and clear current"
    } }
};

// Command group registered with the core in odometer_init().
static sys_commands_t odometer_commands = {
    .n_commands = sizeof(odometer_command_list) / sizeof(sys_command_t),
    .commands = odometer_command_list
};

// Report options hook: advertises the plugin to the host.
static void onReportOptions (bool newopt)
{
    // Let the previous handler(s) output first.
    on_report_options(newopt);

    if(newopt)
        // Machine readable option list: add the ODO tag.
        hal.stream.write(",ODO");
    else
        // Human readable plugin list: name and version.
        report_plugin("Odometers", "0.08");
}

// Plugin entry point, called once during driver startup.
// Checks that suitable non-volatile storage is available, loads the saved
// odometer data and installs all the hooks.
void odometer_init()
{
    static setting_details_t settings = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
        .load = odometer_settings_load,
        .save = odometer_settings_save,
        .restore = odometer_settings_restore
    };

    // Allocate the settings block first so it is counted in the driver area
    // before the free space check below. Same mechanism as the modbus flow plugin.
    nvs_addr = nvs_alloc(sizeof(config));

    // Take a copy of the physical NVS interface so data can be accessed outside the normal settings area.
    memcpy(&nvs, nvs_buffer_get_physical(), sizeof(nvs_io_t));

    if(nvs_addr == 0)
        task_run_on_startup(report_warning, "Odometers: NVS allocation failed, settings not available!");
    // Only EEPROM or FRAM can take the frequent writes, warn on startup otherwise.
    else if(!(nvs.type == NVS_EEPROM || nvs.type == NVS_FRAM))
        task_run_on_startup(report_warning, "EEPROM or FRAM is required for odometers!");
    // Need room for two records (current + previous) plus CRC after the grbl and driver areas.
    // Written as an addition on the right-hand side so the unsigned arithmetic cannot wrap below zero.
    else if(NVS_SIZE < GRBL_NVS_SIZE + hal.nvs.driver_area.size + ((sizeof(odometer_data_t) + NVS_CRC_BYTES) * 2))
        task_run_on_startup(report_warning, "Not enough NVS storage for odometers!");
    else {

        // The records are placed at the very end of NVS: current last, previous just below it.
        odometers_address = NVS_SIZE - (sizeof(odometer_data_t) + NVS_CRC_BYTES);
        odometers_address_prv = odometers_address - (sizeof(odometer_data_t) + NVS_CRC_BYTES);

        // Load the saved record; if it is missing or fails the CRC check, start from zero.
        if(nvs.memcpy_from_nvs((uint8_t *)&odometers, odometers_address, sizeof(odometer_data_t), true) != NVS_TransferResult_OK)
            odometer_data_reset(false);

        // Register the plugin settings (1500 - 1505).
        settings_register(&settings);

        // Tell the core that odometers are supported.
        hal.driver_cap.odometers = On;

        // Install hooks, saving each original pointer so it can be chained.

        // Machine state changes (distance accounting and saving).
        on_state_change = grbl.on_state_change;
        grbl.on_state_change = onStateChanged;

        // Report options (plugin name/version).
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        // Settings changes (to reclaim the step pulse hook).
        settings_changed = hal.settings_changed;
        hal.settings_changed = onSettingsChanged;

        // Spindle selection (to wrap set_state for spindle run time).
        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;

        // Stepper pulse start (to count steps).
        stepper_pulse_start = hal.stepper.pulse_start;
        hal.stepper.pulse_start = stepperPulseStart;

        // Register the $ODOMETERS command.
        system_register_commands(&odometer_commands);
    }
}



#endif // ODOMETER_ENABLE
