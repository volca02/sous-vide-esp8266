/* --------------------------------------------------------
   Sousvide/timer controller project.
   --------------------------------------------------------
*/

#include <Arduino.h>
#include <SH1106Spi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <AutoPID.h>
#include <ClickEncoder.h>

using Display = SH1106Spi;

#define OLED_RST_PIN D3
#define OLED_DC_PIN  D1

#define TEMP_SENSOR_PIN D4

#define ROT_KEY_PIN A0
#define ROT_S1_PIN  D0
#define ROT_S2_PIN  D2
#define ROT_STEPS_PER_NOTCH 4

#define RELAY_PIN   D8

const char *version_str = "0.0.1";

constexpr byte CURSOR_SIZE = 5;

/**

Pinout/connection diagram:

Display (3V3 and GND connected as indicated on display):
  D7 -> SDA
  D5 -> SCK
  D3 -> RST
  D1 -> DC
  GND -> CS

Temp sensor (3V3 and GND connected to red/black respectively):
  D4 -> Yellow wire (onewire data)

Rotary encoder:
  A0 -> Key
  D2 -> S1
  D0 -> S2

Solid state relay:
  D8  -> + port
  GND -> - port
*/

/**
TEST FIXES/IMPROVEMENTS:

TODO:
 * Setup phase screen saving (inactive for too long - blank the screen)
 * Configuration persistence (via IOTWebConf)
 * Temperature offset option - measure 37C with a medical thermometer, remember the diff

 * Setup mode - hold encoder while powering up to enter setup mode
   - a menu item to enable wifi hotspot
   - an option to reset config to defaults
   - temperature offset setting (so basic we need to have it here directly)
   - should listen to programming requests
   - more advanced options done by web page access (IOTWebConf)

 * NTP support, display current time in corner
 * fahrenheit as a second value when setting temp (no conversion needed when cooking by the book)
 * MQTT support
   - report temperature, estimated time(stamp) of end of cooking
   - set mode, disable cooking remotely, set different temperature, timer

DONE:
 X SousVide mode does NOT end after timer ends
 X Re-setting to setup does not put the remaining time back
 X Moving the remaining time should round it down/up as appropriate to the nearest multiple of the 15min interval
 X Ready after initial heatup - require one press of the encoder after preheat before activating timer
   - also after return to setup?
*/


/** This forces ClickEncoder to use A0 as button input
 * (on wemos d1 we don't have a different pin to use!)
 * @note overriden by hand, the original class does not do this right
 */
class AnalogClickEncoder : public ClickEncoder
{
  public:
    explicit AnalogClickEncoder(
            int8_t A, int8_t B, int8_t BTN = -1,
            uint8_t stepsPerNotch = 4, bool active = LOW,
            int16_t rangeLow = 0, int16_t rangeHigh = 512)
        : ClickEncoder(A, B, BTN, stepsPerNotch, active)
    {
        pinMode(pinBTN, INPUT);

        anlogActiveRangeLow = rangeLow;
        anlogActiveRangeHigh = rangeHigh;
        analogInput = true;

        if (anlogActiveRangeLow > anlogActiveRangeHigh) {    // swap values if provided in the wrong order
            int16_t t = anlogActiveRangeLow;
            anlogActiveRangeLow = anlogActiveRangeHigh;
            anlogActiveRangeHigh = t;
        }
    }
};


void draw_cursor_horizonal(Display &display, byte x, byte y, bool cross = false) {
    display.drawLine(x, y,
                     x + CURSOR_SIZE, y + CURSOR_SIZE/2);

    display.drawLine(x, y + CURSOR_SIZE,
                     x + CURSOR_SIZE, y + CURSOR_SIZE/2);

    display.drawLine(x, y,
                     x, y + CURSOR_SIZE);

    if (cross) {
        display.drawLine(x, y + CURSOR_SIZE /2,
                         x + CURSOR_SIZE, y + CURSOR_SIZE / 2);
    }
}

// Handles the cooking time
class Timer {
public:
    Timer(Timer &) = delete;
    Timer() = default;

    static const constexpr unsigned long MAX_TIMER = 24 * 60 * 60 * 1000; // 1 day

    void run() { set_enabled(true); };
    void stop() { set_enabled(false); };

    void reset() { ran = false; }

    void pause() {
        ran = false;
        enabled = false;
        if (target > elapsed) {
            target = target - elapsed;
        } else {
            target = 0;
        }
        elapsed = 0;
    }

    void set_enabled(bool en) {
        if (enabled == en) return;

        enabled = en;

        if (en) {
            prev = millis();
            ran = true;
        }
    }

    void set_target(unsigned long tgt) { target = tgt; }
    unsigned long get_target() const { return target; }

    unsigned long get_remaining() const {
        if (elapsed >= target) return 0;
        return target - elapsed;
    }

    void move_target(int amount, unsigned long interval = 15 * 60 * 1000) {
        // the two weird math ops below guarantee us that we will round up to
        // nearest multiple of interval but move the value at the same time
        if (amount < 0) {
            if (target > interval)
                target = interval * ((target - 1) / interval);
            else
                target = 0;
        } else {
            target = interval * ((target + interval) / interval);
            if (target > MAX_TIMER) target = MAX_TIMER;
        }
    }

    bool is_enabled() const { return enabled; }
    bool done() const { return elapsed >= target && ran; }

    void update() {
        if (enabled && prev) {
            unsigned long now = millis();
            elapsed += now - prev;
            prev = now;
        }
    }

protected:
    bool ran = false; // set after first timer start. Distinguishes done with setup in ui
    bool enabled = false;
    unsigned long prev    = 0;
    unsigned long elapsed = 0;
    unsigned long target  = 0;
};

class TempSensor {
public:
    TempSensor(TempSensor &) = delete;

    TempSensor(uint8_t pin) :
        oneWire(pin),
        dallas(&oneWire)
    {}

    void begin() {
        dallas.begin();
    }

    // update temperature every second
    static const unsigned long UPDATE_INTERVAL = 1000;

    float update() {
        unsigned long time = millis();

        // either time passed or overflowed
        if (time - last_update > UPDATE_INTERVAL) {
            dallas.requestTemperatures();
            temp = dallas.getTempCByIndex(0);

            // only update the interval if succesful
            if (temp != DEVICE_DISCONNECTED_C) last_update = time;

            // TODO: Could record temp history here...
        }

        return temp;
    }

    unsigned long last_update = 0;
    float temp = 0;
    OneWire oneWire;
    DallasTemperature dallas;
};

/*
  Phases:
   1. Configuration after powerup
   2. Heating
   3. Prepared - cooking
   4. Done. Blink finished

 */
class Controller {
public:
    static const constexpr float MIN_TEMP = 0.0;
    static const constexpr float MAX_TEMP = 100.0;
    static const constexpr float DEFAULT_TEMP = 50.0;

    Controller(Controller &) = delete;

    Controller(uint8_t pin, TempSensor &temp, Timer &timer)
            : relay_pin(pin)
            , temp_sensor(temp)
            , timer(timer)
            , pid(&pid_input, &pid_setpoint, &pid_relayState,
                  // TODO: Tune these constants (for now these are copies of values from:
                  // https://github.com/r-downing/wifi-sous-vide/
                  5000, .12, .0003, 0)
    {
    }

    static const constexpr double preheat_threshold = 1.0; // +- this means we're in range

    void begin() {
        // enable pinout of the relay, and switch heating off as default...
        pinMode(relay_pin, OUTPUT);
        heating_off();

        // bangbang is used outside of the 4 degress range of the target
        pid.setBangBang(4);

        // 4000 ms for each pid step... slow cooker is slow!
        pid.setTimeStep(4000);
    }

    // **** this section enables/disables the controller ****
    void set_enabled(bool ena) {
        if (ena == enabled) return;

        enabled = ena;

        if (ena) {
            pid.run();
        } else {
            pid.stop();
            heating_off();
            started = false;
        }
    }

    void enable() {
        set_enabled(true);
    }

    void disable() {
        set_enabled(false);
    }

    bool is_enabled() const { return enabled; }
    bool is_heating() const { return heating; }
    bool is_pid_enabled() const { return pid_mode; }
    double get_target_temp() const { return pid_setpoint; }

    bool is_started() const { return started; }
    void start() { started = true; }

    // **** This section is dedicated to settings ***
    void set_use_pid(bool use_pid) {
        pid_mode = use_pid;
    }

    /// sets target temperature and enables pid mode, or disables pid for negative temps
    void set_target_temp(float temp) {
        pid_mode = true;
        pid_setpoint = temp;

        if (temp <= MIN_TEMP) {
            temp = MIN_TEMP;
            pid_mode = false;
        } else if (temp >= MAX_TEMP) {
            temp = MAX_TEMP;
        }
    }

    void move_target_temp(int steps, float step = 1.0) {
        // each step is 0.1 of a degree. we use acceleration to move faster
        set_target_temp(pid_setpoint + steps * step);
    }

    /** returns true if the controller is in pre-heat mode.
     * this only applies to PID mode, normal mode does not have any pre-heat phase
     */
    bool is_preheating() {
        return pid_mode && preheating;
    }

    void update() {
        // if timer says so, we stop doing what we're doing
        if (timer.done()) {
            disable();
            return;
        }

        if (!pid_mode) {
            if (enabled) started = true; // no need to wait here, just override it...
            set_heating(enabled);
            timer.set_enabled(enabled);
            timer.update();
            return;
        }

        if (!enabled) return;

        // refresh the temperature
        pid_input = temp_sensor.update();

        // pid mode follows:
        pid.run();

        set_heating(pid_relayState);

        bool atPoint = pid.atSetPoint(preheat_threshold);

        // are we preheating or are we set around the target range?
        preheating = !atPoint;

        timer.set_enabled(atPoint && started);
        timer.update();
    }

protected:
    void heating_on() {
        set_heating(true);
    };

    void heating_off() {
        set_heating(false);
    };

    void set_heating(bool ena) {
        if (heating == ena) return;

        heating = ena;
        digitalWrite(relay_pin, ena ? HIGH : LOW);
    }

    uint8_t relay_pin;
    TempSensor &temp_sensor;
    Timer &timer;

    bool pid_mode      = false; // when disabled, simple heating is used
    bool enabled       = false; // after settings get put in, setting this to true will enable the controller
    bool preheating    = true;  // initial phase after controller gets started
    bool heating       = false; // true if the heater is on
    bool started       = false; // after initial pre-heat, we wait for user input once

    // variables used by autopidrelay
    double pid_input = 0;    // this is set by reading value from the temp_sensor
    double pid_setpoint = DEFAULT_TEMP; // this is the desired temperature
    bool   pid_relayState = false;

    AutoPIDRelay pid;
};

class UI;

class UIScreen {
public:
    UIScreen(UIScreen &) = delete;
    UIScreen(UI &ui) : ui(ui) {}

    virtual void draw() {};
    virtual void enter() {};
    virtual void exit() {};

    virtual void on_rotate(int revs) { rot = revs; };
    virtual void on_button(ClickEncoder::Button b) { button = b; };

protected:
    ClickEncoder::Button get_button() {
        ClickEncoder::Button b = button;
        button = ClickEncoder::Open;
        return b;
    }

    int get_rotation() {
        int r = rot;
        rot   = 0;
        return r;
    }

    UI &ui;

private:
    int rot;
    ClickEncoder::Button button; // clear this after using to ClickEncoder::Open!
};

class SetupScreen : public UIScreen {
public:
    SetupScreen(UI &ui) : UIScreen(ui) {}
    void draw() override;

    void enter() override { index = 0; selected = false; }

protected:
    bool selected = false; // true if item is selected
    bool held     = false;
    int index     = 0;
};

class InfoScreen : public UIScreen {
public:
    InfoScreen(UI &ui) : UIScreen(ui) {}

    void draw() override;

    void enter() override { held = false; };

protected:
    // pressed button, as last remembered from the draw call
    ClickEncoder::Button_e last_button = ClickEncoder::Open;
    bool held     = false;
    int counter   = 0;
};

class PreheatScreen : public InfoScreen {
public:
    PreheatScreen(UI &ui) : InfoScreen(ui) {}
    void draw() override;
};

class CookingScreen : public InfoScreen {
public:
    CookingScreen(UI &ui) : InfoScreen(ui) {}
    void draw() override;
};

class DoneScreen : public InfoScreen {
public:
    DoneScreen(UI &ui) : InfoScreen(ui) {}
    void draw() override;
};

class UI {
public:
    UI(UI &) = delete;

    UI(Display &display,
       ClickEncoder &encoder,
       Controller &controller,
       TempSensor &temp_sensor,
       Timer &timer)
            : ctl(controller)
            , temp(temp_sensor)
            , timer(timer)
            , disp(display)
            , encoder(encoder)
            , width(display.getWidth())
            , height(display.getHeight())
            , state(ST_SETUP)
            , screen_setup(*this)
            , screen_preheat(*this)
            , screen_cooking(*this)
            , screen_done(*this)
    {
        screen_setup.enter();
    }

    void begin() {
        disp.init();
        disp.flipScreenVertically();
        disp.setContrast(255);
        disp.clear();
        intro();
    }

    void intro() {
        // this is just a placeholder...
        disp.setFont(ArialMT_Plain_24);
        disp.drawString(10, 10, "SousVide");

        disp.setFont(ArialMT_Plain_10);
        disp.drawString(10, 40, version_str);

        disp.display();

        delay(500);
    }


    void error_screen(const char *text) {
        disp.clear();

        disp.setFont(ArialMT_Plain_24);
        disp.drawString(20, 20, "!");

        disp.setFont(ArialMT_Plain_10);
        disp.drawString(33, 25, text);

        disp.display();
    }

    enum State {
        ST_SETUP = 1,
        ST_PREHEAT,
        ST_COOKING,
        ST_DONE
    };

    void update() {
        // will handle timer and temp_sensor update for us...
        ctl.update();

        // analyze the controller state and change ui accordingly
        State new_state = infer_state();

        if (new_state != state) {
            get_screen(state).exit();
            get_screen(new_state).enter();
            state = new_state;
        }

        // handle encoder input...
        uint32_t ms = micros();

        if (ms - lastService > 1000) {
            lastService = ms;
            encoder.service();
        }

        auto &scr = get_screen(state);

        // send input events to the screen
        int revs = encoder.getValue(); // relative steps that happened this update

        ClickEncoder::Button b = encoder.getButton();

//        bool changed = false;

        if (revs != 0) { scr.on_rotate(revs); input = true; }
        if (b    != ClickEncoder::Open) { scr.on_button(b); input = true; }

        // redraw once every 50th of a second
        static uint32_t lastDraw = 0;

        if (ms - lastDraw > 20000) {
            disp.clear();

            lastDraw = ms;
            // draw the status bar
            draw_status_bar();
            // draw the screen
            scr.draw();
            // screen done it's job, flip
            disp.display();
        }
    }

    Display    &get_display()    { return disp; }

    Controller &get_controller() { return ctl; }
    TempSensor &get_tempsensor() { return temp; }
    Timer      &get_timer()      { return timer; }

protected:
    void draw_status_bar() {
        static int counter = 0;
        counter++;

        // we only display status line if something happened the last N frames
        if (input) {
            input = false;
            counter = 0;
        }

        if (counter > 10 * 50) return;

        char stmp[10];

        // separating line
        disp.drawLine(0, 11, width, 11);

        if (ctl.is_heating()) {
            disp.fillCircle(5,5,3);
        }

        // since the controls are lagging when temp measurement is done,
        // do this only if we don't need controls
        if (ctl.is_pid_enabled() && ctl.is_enabled()) {
            float ftmp = temp.update();
            snprintf(stmp, 10, "%3.1f C", ftmp);
            disp.drawString(10, 0, stmp);
            ftmp = ctl.get_target_temp();
            snprintf(stmp, 10, "^%3.1f C", ftmp);
            disp.drawString(45, 0, stmp);
        } else {
            // NOT NEEDED           disp.drawString(0, 0, "---");
        }

        // and also display timer if enabled
        unsigned long remaining = timer.get_remaining();
        unsigned long secs = (remaining % 60000) / 1000;
        remaining = remaining / 60000; // to minutes

        // a dot means a stopped timer
        char ts = timer.is_enabled() ? ' ' : '.';

        snprintf(stmp, 10, "%c%02ld:%02ld:%02ld", ts, remaining / 60, remaining % 60, secs);
        disp.drawString(83, 0, stmp);
    }

    UIScreen &get_screen(State st) {
        switch (state) {
        case ST_SETUP:   return screen_setup;
        case ST_PREHEAT: return screen_preheat;
        case ST_COOKING: return screen_cooking;
        case ST_DONE:    return screen_done;
        default: return screen_setup;
        }
    }

    State infer_state() {
        if (timer.done()) return ST_DONE;
        if (!ctl.is_enabled()) return ST_SETUP;
        if (ctl.is_preheating()) return ST_PREHEAT;
        return ST_COOKING;
    }

    Controller &ctl;
    TempSensor &temp;
    Timer &timer;

    Display &disp;
    ClickEncoder &encoder;
    uint32_t lastService = 0;
    uint8_t width;
    uint8_t height;

    State state; // curent screen state...

    bool input = false; // set to true if input happened

    // all the screens
    SetupScreen screen_setup;
    PreheatScreen screen_preheat;
    CookingScreen screen_cooking;
    DoneScreen screen_done;
};

/// Screen implementations follow:
void SetupScreen::draw() {
    // inactivity counter
    static int counter = 0;
    counter++;

    auto &disp = ui.get_display();

    auto &ctl = ui.get_controller();
    auto &tmr = ui.get_timer();

    // handle input....
    int rot = get_rotation();
    auto b = get_button();

    // any event will reset the inactivity counter
    if (rot != 0 || b != ClickEncoder::Open) counter = 0;

    if (b == ClickEncoder::Released) {
        held = false;
    }

    bool on_held = false; // the first event of many when button is held

    if (b == ClickEncoder::Clicked) {
        held = false;
        selected = !selected;
    } else if (b == ClickEncoder::Held) {
        if (!held) {
            held = true;
            on_held = true;
        }
    }

    // selected the cook option?
    if (selected && index == 2) {
        ctl.set_enabled(true);
    }

    // non-selected temp held button resets the temp usage to false - non-PID
    if (on_held && !selected && (index == 1)) {
        // set/unset pid control
        bool use = !ctl.is_pid_enabled();
        ctl.set_use_pid(use);
    }

    // handle rotation
    if (rot != 0) {
        if (selected) {
            // move the value
            if (index == 0) {
                tmr.move_target(rot, held ? 5 * 60 * 1000: 15 * 60 * 1000);
            } else if (index == 1) {
                ctl.move_target_temp(rot, held ? 0.1 : 1.0);
            }
        } else {
            index = (3 + index + rot) % 3;
        }
    }

    if (counter > 20 * 50) return; // no rendering if inactive for long

    // screen rendering itself

    // Time setting
    // in 15 minute increments...
    disp.drawString(10, 15, "Timer");
    disp.drawString(10, 30, "Temp");
    disp.drawString(10, 45, "Cook");

    int minutes = tmr.get_target() / 60000;

    // offset of the setting values in X axis
    const int voff = 50;

    char st[16];
    snprintf(st, 16, "%02u:%02u", minutes/60, minutes % 60);

    disp.drawString(voff + 10, 15, st);

    if (ctl.is_pid_enabled()) {
        snprintf(st, 16, "%3.1f C", ctl.get_target_temp());
    } else {
        snprintf(st, 16, "OFF");
    }

    disp.drawString(voff + 10, 30, st);

    draw_cursor_horizonal(disp, 3 + (selected ? voff : 0), 4 + 15 * (index + 1), held);
}


void InfoScreen::draw() {
    auto &disp = ui.get_display();
    auto &timer = ui.get_timer();
    auto &ctl = ui.get_controller();

    last_button = get_button();
    if (last_button == ClickEncoder::Held) held = true;
    if (last_button == ClickEncoder::Released && held) {
        timer.pause();
        ctl.disable();
    }

    if (held) disp.fillCircle(5, 16, 3);

    counter++;
}

void PreheatScreen::draw() {
    InfoScreen::draw();

    auto &disp = ui.get_display();
    auto &ctl = ui.get_controller();
    auto &temp = ui.get_tempsensor();

    char stmp[32];

    float ftmp = temp.update();
    float ftgt = ctl.get_target_temp();

    snprintf(stmp, 32, "Preheat %3.1f > %3.1f C", ftmp, ftgt);

    disp.drawString(10, 15 + (10 * ((counter / 60) % 4)), stmp);
}

void CookingScreen::draw() {
    InfoScreen::draw();

    auto &disp = ui.get_display();
    auto &ctl = ui.get_controller();
    auto &timer = ui.get_timer();
    auto &temp = ui.get_tempsensor();

    char stmp[64];

    unsigned long remaining = timer.get_remaining();
    unsigned long secs = (remaining % 60000) / 1000; // remaining seconds when not rounded up

    if (held) disp.fillCircle(5, 15, 3);

    if ((!ctl.is_started()) && (last_button == ClickEncoder::Clicked)) {
        ctl.start();
    }

    if (!ctl.is_started()) {
        float ftmp = temp.update(); // current temp
        snprintf(stmp, 64, "Press to start... %3.1fC", ftmp);
    } else if (ctl.is_pid_enabled()) {
        float ftmp = temp.update(); // current temp
        unsigned long rem_mins = (remaining + 59999) / 60000; // to minutes, rounded up
        snprintf(stmp, 64, "Cooking... %3.1fC %02ld:%02ld", ftmp, rem_mins / 60, rem_mins % 60);
    } else {
        // when displaying seconds we dont round up
        remaining = remaining / 60000;
        snprintf(stmp, 64, "Cooking... %02ld:%02ld:%02ld", remaining / 60, remaining % 60, secs);
    }

    disp.drawString(10, 15 + (10 * ((counter / 60) % 4)), stmp);
}

void DoneScreen::draw() {
    InfoScreen::draw();

    auto &disp = ui.get_display();
    auto &timer = ui.get_timer();

    if (held)
        disp.drawString(10, 15 + (10 * ((counter / 60) % 4)), "Done...");
    else
        disp.drawString(10, 15 + (10 * ((counter / 60) % 4)), "Done!");
}


AnalogClickEncoder encoder{ROT_S2_PIN, ROT_S1_PIN, ROT_KEY_PIN, ROT_STEPS_PER_NOTCH, false};
Display display(OLED_RST_PIN, OLED_DC_PIN, 0 /*cs is unused*/);

TempSensor temp(TEMP_SENSOR_PIN);
Timer timer;
Controller controller(RELAY_PIN, temp, timer);
UI ui(display, encoder, controller, temp, timer);

void check_temp_sensor()
{
    float t = DEVICE_DISCONNECTED_C;
    do {
        t = temp.update();

        if (t == DEVICE_DISCONNECTED_C) {
            ui.error_screen("Temp. sensor!");
            delay(500);
        }

    } while (t == DEVICE_DISCONNECTED_C);
}

void setup()
{
    encoder.setButtonHeldEnabled(true);
    encoder.setDoubleClickEnabled(false);
    // for esp8266, since pin no. zero is valid
    encoder.setButtonOnPinZeroEnabled(true);
    encoder.setHoldTime(500);

    temp.begin();
    controller.begin();
    ui.begin();

    // check for temp sensor
    check_temp_sensor();
}

void loop()
{
    ui.update();
}
