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

#include <ArduinoOTA.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>
#include <PubSubClient.h>

#include <pidautotuner.h>

static int8_t ICACHE_FLASH_ATTR todigit(char ch) {
    if (ch >= '0' && ch <= '9')
        return ch - '0';

    return -1;
}

static bool validate_dec(const char *buf) {
    for (;*buf;++buf) {
        if (todigit(*buf) < 0)
            return false;
    }
    return true;
}

// ------------ IOT Web Conf Code ------------
#define NUM_LEN 16 // for the regulator constants

const float pid_loop_interval = 5000; // in ms, loop interval of the pid

bool config_mode = false; // enabled by pressing encoder while starting the device

// this is IOTWebconf side config, with conversion methods to get the values
class Config {
public:
    // PID constants
    char kp[NUM_LEN] = "0.12";
    char ki[NUM_LEN] = "0.0003";
    char kd[NUM_LEN] = "0";

    float get_kp() const { return atof(kp); }
    float get_ki() const { return atof(ki); }
    float get_kd() const { return atof(kd); }

    // temperature offset
    char temp_offset[NUM_LEN] = "0";

    float get_temp_offset() const { return atof(temp_offset); }

    void move_temp_offset(int steps, float step = 0.1) {
        float cur = get_temp_offset();
        cur += step * steps;
        snprintf(temp_offset, NUM_LEN, "%f", cur);
    }

    // MQTT Server
    char mqtt_client_id[20];
    char mqtt_server[41] = "";
    char mqtt_port[NUM_LEN] = "1883";
    char mqtt_user[21] = "";
    char mqtt_pass[21] = "";
    char mqtt_topic_prefix[41] = "sousvide";
};

class NetworkHandler {
public:
// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
    const char *thing_name = "SousVide";
    const char *wifi_initial_pwd = "password123456"; // password for initial ap
    const char *config_version = "cv1";

    NetworkHandler(Config &config)
            : config(config)
            , dnsServer()
            , server(80)
            , iotWebConf(thing_name,
                         &dnsServer,
                         &server,
                         wifi_initial_pwd,
                         config_version)

            , controller_group("Controller", "")
            , kp("Kp",
                 "kp",
                 config.kp,
                 NUM_LEN,
                 nullptr,
                 "Kp regulator constant",
                 "step='0.0001'")
            , ki("Ki",
                 "ki",
                 config.ki,
                 NUM_LEN,
                 nullptr,
                 "Ki regulator constant",
                 "step='0.0001'")
            , kd("Kd",
                 "kd",
                 config.kd,
                 NUM_LEN,
                 nullptr,
                 "Kd regulator constant",
                 "step='0.0001'")
            , temp_offset("Temp offset",
                          "temp_offset",
                          config.temp_offset,
                          NUM_LEN,
                          nullptr,
                          "Greater val. means the sensor indicates less than real temp.")
            , mqtt_group("MQTT Connection", "")
            , mqtt_server("MQTT Server", "mqtt_server", config.mqtt_server, 40)
            , mqtt_port("MQTT Port",
                        "mqtt_port",
                        config.mqtt_port,
                        NUM_LEN,
                        nullptr,
                        "MQTT Server Port")
            , mqtt_user("MQTT User", "mqtt_user", config.mqtt_user, 20)
            , mqtt_pass("MQTT Password", "mqtt_pass", config.mqtt_pass, 20)
            , mqtt_topic(
                      "MQTT Topic", "mqtt_topic", config.mqtt_topic_prefix, 40)
    {}

    void setup() {
        controller_group.addItem(&kp);
        controller_group.addItem(&ki);
        controller_group.addItem(&kd);
        controller_group.addItem(&temp_offset);

        mqtt_group.addItem(&mqtt_server);
        mqtt_group.addItem(&mqtt_port);
        mqtt_group.addItem(&mqtt_user);
        mqtt_group.addItem(&mqtt_pass);
        mqtt_group.addItem(&mqtt_topic);

        iotWebConf.addParameterGroup(&controller_group);
        iotWebConf.addParameterGroup(&mqtt_group);

        // TODO: skipAPStartup/forceAPMode
        iotWebConf.init();

        server.on("/", [&] { handle_root(); });
        server.on("/config", [&] { iotWebConf.handleConfig(); });
        server.onNotFound([&] { iotWebConf.handleNotFound(); });
        iotWebConf.setFormValidator([&] (iotwebconf::WebRequestWrapper*) { return validate_config(); });

        server.begin();
    }

    void handle_root() {
        // -- Let IotWebConf test and handle captive portal requests.
        if (iotWebConf.handleCaptivePortal()) {
            // -- Captive portal request were already served.
            return;
        }

        // AP mode config portal
        auto state = iotWebConf.getState();
        if (state == IOTWEBCONF_STATE_AP_MODE
            || state == IOTWEBCONF_STATE_NOT_CONFIGURED)
        {
            iotWebConf.handleConfig();
            return;
        }

        String s =
            "<!DOCTYPE html><html lang=\"en\"><head><meta "
            "name=\"viewport\" content=\"width=device-width, "
            "initial-scale=1, user-scalable=no\"/>"
            "<title>SousVide status</title></head><body> PLACEHOLDER!"
            "<a href='config'>configuration</a>"
            "</body></html>\n";

        server.send(200, "text/html", s);
    }

    void update() {
        iotWebConf.doLoop();
        server.handleClient();
    }

    void saveConfig() {
        iotWebConf.saveConfig();
    }

protected:
    bool validate_config() {
        bool valid = true;

        auto mqtt_p = server.arg(mqtt_port.getId());
        if (!validate_dec(mqtt_p.c_str())) {
            mqtt_port.errorMessage = "MQTT Port expects a number!";
            valid = false;
        }

        return valid;
    }

    Config &config;
    DNSServer dnsServer;
    WebServer server;
    IotWebConf iotWebConf;

    // setup params
    IotWebConfParameterGroup controller_group;
    IotWebConfNumberParameter kp;
    IotWebConfNumberParameter ki;
    IotWebConfNumberParameter kd;
    IotWebConfNumberParameter temp_offset;

    IotWebConfParameterGroup mqtt_group;

    IotWebConfTextParameter mqtt_server;
    IotWebConfNumberParameter mqtt_port;
    IotWebConfTextParameter mqtt_user;
    IotWebConfTextParameter mqtt_pass;
    IotWebConfTextParameter mqtt_topic;
};

// -------------------- HW Config --------------------
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
TODO:
 * NTP support, display current time in corner
 * fahrenheit as a second value when setting temp (no conversion needed when cooking by the book)

DONE:
 X SousVide mode does NOT end after timer ends
 X Re-setting to setup does not put the remaining time back
 X Moving the remaining time should round it down/up as appropriate to the nearest multiple of the 15min interval
 X Ready after initial heatup - require one press of the encoder after preheat before activating timer
   - also after return to setup?
 X Setup phase screen saving (inactive for too long - blank the screen)
 X Config mode
 X MQTT support
   - report temperature, estimated time(stamp) of end of cooking
   - set mode, disable cooking remotely, set different temperature, timer
 X Configuration persistence (via IOTWebConf)
 X Temperature offset option - measure 37C with a medical thermometer, remember the diff
 X FIX PID autotune. Currently does not work
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
        dallas(&oneWire),
        offset(0)
    {}

    void begin(float off) {
        offset = off;
        dallas.begin();

        //
        dallas.getAddress(tempDeviceAddress, 0);
        dallas.setResolution(tempDeviceAddress, resolution);
        dallas.setWaitForConversion(false);
        dallas.requestTemperatures();
        // calculate the delay needed to read the temp
        delayInMillis = 750 / (1 << (12 - resolution));
        last_update = millis();
    }

    float get() {
        unsigned long time = millis();

        // either time passed or overflowed
        if (time - last_update > delayInMillis) {
            dallas.requestTemperatures();
            temp = dallas.getTempCByIndex(0);
            last_update = time;
            dallas.requestTemperatures();
            // TODO: Could record temp history here...
        }

        // in case someone measured and set this, we add offset here
        return temp + offset;
    }

    unsigned long last_update = 0;
    float temp = 0;
    OneWire oneWire;
    DallasTemperature dallas;
    float offset;

    int resolution = 10;
    DeviceAddress tempDeviceAddress;
    unsigned long delayInMillis = 0; //
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

    Controller(Config &conf, uint8_t pin, TempSensor &temp, Timer &timer)
            : conf(conf)
            , relay_pin(pin)
            , temp_sensor(temp)
            , timer(timer)
            , pid(&pid_input, &pid_setpoint, &pid_relayState,
                  pid_loop_interval, conf.get_kp(), conf.get_ki(), conf.get_kd())
    {
    }

    static const constexpr double preheat_threshold = 1.0; // +- this means we're in range

    void begin() {
        // enable pinout of the relay, and switch heating off as default...
        pinMode(relay_pin, OUTPUT);
        heating_off();

        // bangbang is used outside of the 4 degress range of the target
        pid.setBangBang(4);
        pid.setTimeStep(pid_loop_interval);
        set_gains();
    }

    void set_gains() {
        // we set gains here again because network.setup() read the config for us
        pid.setGains(conf.get_kp(), conf.get_ki(), conf.get_kd());
    }

    // **** this section enables/disables the controller ****
    void set_enabled(bool ena) {
        if (ena == enabled) return;

        enabled = ena;

        if (ena) {
            pid.run();
        } else {
            // also handle timer
            timer.pause();
            pid.stop();
            heating_off();
            started = false;
            preheating = true;
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

        // early exit if disabled. This means PIDTuner can override and use
        // set_heater without us immediately switching it off due to disabled
        // controller
        if (!enabled) return;

        // below, all code is with enabled on...
        if (!pid_mode) {
            started = true; // no need to wait here, just override it...
            set_heating(true);
            timer.set_enabled(true);
            timer.update();
            return;
        }

        // refresh the temperature
        pid_input = temp_sensor.get();

        // pid mode follows:
        pid.run();

        set_heating(pid_relayState);

        bool atPoint = pid.atSetPoint(preheat_threshold);

        // are we preheating or are we set around the target range?
        // once we get in the temp range, we dont't ever get out
        // unless we reset the whole process.
        // This mitigates the issue where we insert the food and
        // temp drops down, forcing us to wait for preheat again...
        if (preheating) preheating = !atPoint;
        timer.set_enabled(atPoint && started);
        timer.update();
    }

protected:
    friend class PIDTuner;

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

    Config &conf;

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

class PIDTuner {
public:
    PIDTuner(Config &config, TempSensor &temp, Controller &ctl)
            : config(config), temp_sensor(temp), ctl(ctl)
    {
    }

    bool is_enabled() const { return enabled; }

    float get_kp() { return pidtuner.getKp(); }
    float get_ki() { return pidtuner.getKi(); }
    float get_kd() { return pidtuner.getKd(); }

    bool is_finished() {
        bool finished = pidtuner.isFinished();

        if (enabled && finished) {
            ctl.heating_off();
            enabled = false;
        }

        return finished;
    }

    void setup() {
        // only call update if the loop interval reached the pid loop interval
        pidtuner.setTargetValue(60.0f); // 60 C seems like a good target to tune on
        pidtuner.setLoopInterval(pid_loop_interval * 1000); // same as AutoPID timestep
        pidtuner.setOutputRange(0.0f, 1.0f); // same as AutoPIDrelay
        pidtuner.setZNMode(PIDAutotuner::ZNModeNoOvershoot);
        pidtuner.setTuningCycles(5);
    }

    void start() {
        if (enabled) return;
        enabled = true;
        pidtuner.startTuningLoop(micros());
    }

    void stop() {
        enabled = false;
    }

    void update() {
        if (!enabled) return;

        static long prev_ms = 0;
        auto ms = micros();

        // only handle updates every loop time, the same as AutoPID does
        if ((ms - prev_ms) > (pid_loop_interval * 1000)) {
            prev_ms = ms;

            if (pidtuner.isFinished()) {
                ctl.heating_off();
                return;
            }

            float temp = temp_sensor.get();

            control_value = pidtuner.tune(temp, ms);
        }

        // handle the heating element
        handle_heater();
    }

protected:
    void handle_heater() {
        // just a precaution
        if (!enabled) {
            ctl.heating_off();
            return;
        }

        // this has to align with how the RelayAutoPID works!
        auto pulseWidth = pid_loop_interval;
        while ((millis() - last_pulse_time) > pulseWidth) last_pulse_time += pid_loop_interval;
        bool should_heat = (millis() - last_pulse_time) < (control_value * pid_loop_interval);
        ctl.set_heating(should_heat);
    }

    bool enabled = false;
    Config &config;
    TempSensor &temp_sensor;
    PIDAutotuner pidtuner;
    Controller &ctl;
    double control_value = 0;
    unsigned long last_pulse_time = 0;
};

/// Handles status update messages to mqtt, and listens to mqtt commands back
class MQTTHandler {
public:
    MQTTHandler(Config &conf, Controller &ctl, Timer &t)
            : conf(conf)
            , enabled(false)
            , wifiClient()
            , client(wifiClient)
            , ctl(ctl)
            , timer(t)
    {
        set_temp_topic[0]   = 0;
        set_status_topic[0] = 0;
        set_kp_topic[0]     = 0;
        set_ki_topic[0]     = 0;
        set_kd_topic[0]     = 0;
    }

    bool is_enabled() const { return enabled; }

    void setup() {
        // TODO: Publish device capabilities for homeassistant
        enabled = strlen(conf.mqtt_server) > 0;

        if (enabled) {
            int port = atoi(conf.mqtt_port);
            client.setServer(conf.mqtt_server, port);
            client.setCallback([&](char *topic, byte *payload, unsigned int length){ callback(topic, payload, length); });
            snprintf(set_temp_topic, 128, "%s/temp/set", conf.mqtt_topic_prefix);
            snprintf(set_status_topic, 128, "%s/status/set", conf.mqtt_topic_prefix);
            snprintf(set_kp_topic, 128, "%s/kp/set", conf.mqtt_topic_prefix);
            snprintf(set_ki_topic, 128, "%s/ki/set", conf.mqtt_topic_prefix);
            snprintf(set_kd_topic, 128, "%s/kd/set", conf.mqtt_topic_prefix);
        }
    }

    void loop() { client.loop(); }

    void update(float temp, const char *status) {
        if (!enabled) return;

        if (!client.connected()) connect();

        if (client.connected()) {
            Serial.println("MQTT Publish");
            // report back
            char topic[128];
            if (ctl.is_pid_enabled()) {
                snprintf(topic, 128, "%s/temp", conf.mqtt_topic_prefix);
                client.publish(topic, String(temp).c_str());
            }
            snprintf(topic, 128, "%s/status", conf.mqtt_topic_prefix);
            client.publish(topic, status);
            snprintf(topic, 128, "%s/timer", conf.mqtt_topic_prefix);
            client.publish(topic, String(timer.get_remaining() / 1000).c_str()); // time remaining, in seconds
        }
    }

private:
    enum Status {
        STAT_UNKNOWN = 0,
        STAT_STOP,
        STAT_START
    };

    void callback(char* topic, byte* payload, unsigned int length) {
        if (strcmp(topic, set_temp_topic) == 0) {
            float temp = atof((char*)payload);
            ctl.set_target_temp(temp);
        } else if (strcmp(topic, set_status_topic) == 0) {
            Status s = str_to_status((char *)payload, length);
            switch (s) {
            case STAT_STOP: ctl.disable(); break;
            case STAT_START: ctl.set_enabled(true); break;
            default: ;// ---- ignore
            }
        } else if (strcmp(topic, set_kp_topic) == 0) {
            strncpy(conf.kp, (char *)payload, length < NUM_LEN ? length : NUM_LEN);
            conf.kp[NUM_LEN - 1] = 0; // just to be sure...
            ctl.set_gains();
        } else if (strcmp(topic, set_ki_topic) == 0) {
            strncpy(conf.ki, (char *)payload, length < NUM_LEN ? length : NUM_LEN);
            conf.kp[NUM_LEN - 1] = 0; // just to be sure...
            ctl.set_gains();
        } else if (strcmp(topic, set_kd_topic) == 0) {
            strncpy(conf.kd, (char *)payload, length < NUM_LEN ? length : NUM_LEN);
            conf.kp[NUM_LEN - 1] = 0; // just to be sure...
            ctl.set_gains();
        }
    }


    Status str_to_status(const char *payload, unsigned length) {
        if (length == 0) return STAT_UNKNOWN;

        if (payload[0] == 's') {
            if (strncmp(payload, "start", 5) == 0) return STAT_START;
            if (strncmp(payload, "stop", 4) == 0) return STAT_STOP;
        }

        return STAT_UNKNOWN;
    }

    void connect() {
        if (!client.connected()) {
            static unsigned long last_conn = 0;
            auto now = millis();
            // once every 5s is more than enough
            if ((now - last_conn) <  5000) return;
            last_conn = now;

            Serial.println("MQTT Conn");
            String clientId = "SousVide-";
            clientId += String(random(0xffff), HEX);

            bool conn = false;
            if (strlen(conf.mqtt_user) > 0) {
                conn = client.connect(clientId.c_str(), conf.mqtt_user, conf.mqtt_pass);
            } else {
                conn = client.connect(clientId.c_str());
            }

            if (conn) {
                Serial.println("MQTT Conn OK");
                client.subscribe(set_temp_topic);
                client.subscribe(set_status_topic);
            } else {
                Serial.println("MQTT Conn failed");
            }
        }
    }

    Config &conf;
    bool enabled;
    WiFiClient wifiClient;
    PubSubClient client;
    Controller &ctl;
    Timer &timer;
    char set_temp_topic[128]; // built in setup to avouid doing this repetedly
    char set_status_topic[128];
    char set_kp_topic[128];
    char set_ki_topic[128];
    char set_kd_topic[128];
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

    virtual void format_status(char *ptr, unsigned len) { ptr[0] = 0; }

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

class ConfigScreen : public UIScreen {
public:
    ConfigScreen(UI &ui) : UIScreen(ui) {}

    void draw() override;
    void enter() override { index = 0; selected = false; held = false; }
    void format_status(char *ptr, unsigned len) override;

protected:
    bool selected = false; // true if item is selected
    bool held     = false;
    int index     = 0;
};

class SetupScreen : public UIScreen {
public:
    SetupScreen(UI &ui) : UIScreen(ui) {}
    void draw() override;
    void enter() override { index = 0; selected = false; held = false; }
    void format_status(char *ptr, unsigned len) override;

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
    void format_status(char *ptr, unsigned len) override;

protected:
    // pressed button, as last remembered from the draw call
    ClickEncoder::Button_e last_button = ClickEncoder::Open;
    bool held     = false;
    int counter   = 0;
};

class AutotuneScreen : public InfoScreen {
public:
    AutotuneScreen(UI &ui) : InfoScreen(ui) {}
    void draw() override;
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

    UI(Config &conf,
       Display &display,
       ClickEncoder &encoder,
       Controller &controller,
       PIDTuner &pidtuner,
       TempSensor &temp_sensor,
       Timer &timer,
       NetworkHandler &network)
            : conf(conf)
            , ctl(controller)
            , pidtuner(pidtuner)
            , temp(temp_sensor)
            , timer(timer)
            , network(network)
            , disp(display)
            , encoder(encoder)
            , width(display.getWidth())
            , height(display.getHeight())
            , state(ST_SETUP)
            , screen_config(*this)
            , screen_autotune(*this)
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

        if (config_mode) {
            disp.drawString(70, 40, "CONFIG");
        }

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
        ST_CONFIG = 0,
        ST_AUTOTUNE,
        ST_SETUP,
        ST_PREHEAT,
        ST_COOKING,
        ST_DONE
    };

    void update() {
        if (pidtuner.is_enabled()) {
            pidtuner.update();

            // this will handle disabling the autotuner as well...
            if (pidtuner.is_finished()) {
                snprintf(conf.kp, NUM_LEN, "%f", pidtuner.get_kp());
                snprintf(conf.ki, NUM_LEN, "%f", pidtuner.get_ki());
                snprintf(conf.kd, NUM_LEN, "%f", pidtuner.get_kd());
            }
        } else {
            // will handle timer and temp_sensor update for us...
            ctl.update();
        }

        // analyze the controller state and change ui accordingly
        State new_state = infer_state();

        if (new_state != state) {
            get_screen(state).exit();
            get_screen(new_state).enter();
            state = new_state;
        }

        uint32_t ms = micros();

        auto &scr = get_screen(state);

        // send input events to the screen
        int revs = encoder.getValue(); // relative steps that happened this update

        ClickEncoder::Button b = encoder.getButton();

        if (revs != 0) { scr.on_rotate(revs); input = true; }
        if (b    != ClickEncoder::Open) { scr.on_button(b); input = true; }

        // redraw once every 50th of a second
        static uint32_t lastDraw = 0;

        if (ms - lastDraw > 20000) {
            disp.clear();

            lastDraw = ms;
            // draw the status bar
            draw_status_bar(scr);
            // draw the screen
            scr.draw();
            // screen done it's job, flip
            disp.display();
        }
    }

    Config     &get_config()     { return conf; }
    Display    &get_display()    { return disp; }

    Controller &get_controller() { return ctl; }
    PIDTuner   &get_pidtuner() { return pidtuner; }
    TempSensor &get_tempsensor() { return temp; }
    Timer      &get_timer()      { return timer; }
    NetworkHandler &get_network()    { return network; }

    State infer_state() {
        if (pidtuner.is_enabled()) return ST_AUTOTUNE;
        if (config_mode) return ST_CONFIG;
        if (timer.done()) return ST_DONE;
        if (!ctl.is_enabled()) return ST_SETUP;
        if (ctl.is_preheating()) return ST_PREHEAT;
        return ST_COOKING;
    }

protected:
    void draw_status_bar(UIScreen &scr) {
        static int counter = 0;
        counter++;

        // we only display status line if something happened the last N frames
        if (input) {
            input = false;
            counter = 0;
        }

        if (counter > 10 * 50) return;

        // separating line
        disp.drawLine(0, 11, width, 11);

        if (ctl.is_heating()) {
            disp.fillCircle(5,5,3);
        }

        char stmp[32];
        scr.format_status(stmp, 32);
        disp.drawString(11, 0, stmp);
    }

    UIScreen &get_screen(State st) {
        switch (state) {
        case ST_CONFIG:  return screen_config;
        case ST_AUTOTUNE: return screen_autotune;
        case ST_SETUP:   return screen_setup;
        case ST_PREHEAT: return screen_preheat;
        case ST_COOKING: return screen_cooking;
        case ST_DONE:    return screen_done;
        default: return screen_setup;
        }
    }

    Config &conf;

    Controller &ctl;
    PIDTuner   &pidtuner;
    TempSensor &temp;
    Timer &timer;
    NetworkHandler &network;

    Display &disp;
    ClickEncoder &encoder;
    uint32_t lastService = 0;
    uint8_t width;
    uint8_t height;

    State state; // curent screen state...

    bool input = false; // set to true if input happened

    // all the screens
    ConfigScreen screen_config;
    AutotuneScreen screen_autotune;
    SetupScreen screen_setup;
    PreheatScreen screen_preheat;
    CookingScreen screen_cooking;
    DoneScreen screen_done;
};

/// Screen implementations follow:
void ConfigScreen::draw() {
    // inactivity counter
    static int counter = 0;
    counter++;

    auto &disp = ui.get_display();

    auto &conf = ui.get_config();
    auto &pidtuner = ui.get_pidtuner();
    auto &network = ui.get_network();

    // handle input....
    int rot = get_rotation();
    auto b = get_button();

    // any event will reset the inactivity counter
    if (rot != 0 || b != ClickEncoder::Open) counter = 0;

    bool released = false;
    if (b == ClickEncoder::Released) { released = true; }

    if (b == ClickEncoder::Clicked) {
        held = false;
        selected = !selected;
    } else if (b == ClickEncoder::Held) {
        if (!held) {
            held = true;
        }
    }

    // no select for first item...
    if (selected && index == 0) selected = false;

    if (released && held && index == 0) {
        pidtuner.start();
    }

    if (rot != 0) {
        if (selected) {
            // move the value
            if (index == 1) {
                conf.move_temp_offset(rot, held ? 0.1 : 1.0);
            }
        } else {
            // change selected item
            index = (3 + index + rot) % 3;
        }
    }

    // selected the cook option?
    if (selected && index == 2) {
        // save the new config, and reboot
        network.saveConfig();
        ESP.restart();
    }

    // reset held flag if button was released
    if (released) held = false;

    if (counter > 20 * 50) return; // no rendering if inactive for long

    // screen rendering itself
    const int voff = 50; // offseting of the values...

    disp.drawString(10, 15, "PID autotune");
    disp.drawString(10, 30, "T Delta");
    disp.drawString(10, 45, "[Save & reboot]");

    char st[32];

    disp.drawLine(voff + 40, 15, voff + 40, 56);

    snprintf(st, 16, "P%s", conf.kp);
    disp.drawString(voff + 43, 15, st);
    snprintf(st, 16, "I%s", conf.ki);
    disp.drawString(voff + 43, 30, st);
    snprintf(st, 16, "D%s", conf.kd);
    disp.drawString(voff + 43, 45, st);

    snprintf(st, 16, "%3.1f C", conf.get_temp_offset()); // may be slow due to number conversion...
    disp.drawString(voff + 10, 30, st);

    draw_cursor_horizonal(disp, 3 + (selected ? voff : 0), 4 + 15 * (index + 1), held);
}

void ConfigScreen::format_status(char *ptr, unsigned int len) {
    const auto &ip = WiFi.localIP();
    float ftmp = ui.get_tempsensor().get();
    snprintf(ptr, len, "%d.%d.%d.%d | %3.2f C", ip[0], ip[1], ip[2], ip[3], ftmp);
}

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
    disp.drawString(10, 45, "[Cook]");

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

void SetupScreen::format_status(char *ptr, unsigned int len) {
    auto &temp = ui.get_tempsensor();
    float ftmp = temp.get();
    snprintf(ptr, 10, "%3.2f C", ftmp);
}

void InfoScreen::draw() {
    auto &disp = ui.get_display();
    auto &ctl = ui.get_controller();
    auto &pidtuner = ui.get_pidtuner();

    last_button = get_button();
    if (last_button == ClickEncoder::Held) held = true;
    if (last_button == ClickEncoder::Released && held) {
        ctl.disable();
        pidtuner.stop();
    }

    if (held) disp.fillCircle(5, 16, 3);

    counter++;
}

void InfoScreen::format_status(char *ptr, unsigned int len) {
    auto &temp = ui.get_tempsensor();
    auto &ctl = ui.get_controller();
    auto &timer = ui.get_timer();
    float ftmp = temp.get();

    // and also display timer if enabled
    unsigned long remaining = timer.get_remaining();
    remaining = remaining / 60000; // to minutes

    // a dot means a stopped timer
    char ts = timer.is_enabled() ? ' ' : '.';

    char ctl_tmp[7];

    if (ctl.is_pid_enabled()) {
        snprintf(ctl_tmp, 7, "%3.1f C", ctl.get_target_temp());
    }

    snprintf(ptr, len, "%3.2f C%s %c%02ld:%02ld", ftmp, ctl_tmp, ts, remaining / 60, remaining % 60);
}

void AutotuneScreen::draw() {
    InfoScreen::draw();

    auto &disp = ui.get_display();
    auto &temp = ui.get_tempsensor();

    char stmp[32];

    float ftmp = temp.get();

    // TODO: Hack the autotune library to give us i loop index so we can monitor progress
    snprintf(stmp, 32, "Autotune... %3.1f C", ftmp);

    // could also print current kp, ki and kd
    disp.drawString(10, 15 + (10 * ((counter / 60) % 4)), stmp);
}

void PreheatScreen::draw() {
    InfoScreen::draw();

    auto &disp = ui.get_display();
    auto &ctl = ui.get_controller();
    auto &temp = ui.get_tempsensor();

    char stmp[32];

    float ftmp = temp.get();
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
        float ftmp = temp.get(); // current temp
        snprintf(stmp, 64, "Press to start... %3.1fC", ftmp);
    } else if (ctl.is_pid_enabled()) {
        float ftmp = temp.get(); // current temp
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

    if (held)
        disp.drawString(10, 15 + (10 * ((counter / 60) % 4)), "Done...");
    else
        disp.drawString(10, 15 + (10 * ((counter / 60) % 4)), "Done!");
}


AnalogClickEncoder encoder{ROT_S2_PIN, ROT_S1_PIN, ROT_KEY_PIN, ROT_STEPS_PER_NOTCH, false};
Display display(OLED_RST_PIN, OLED_DC_PIN, 0 /*cs is unused*/);

Config conf;
NetworkHandler network(conf); // loads the config for us
TempSensor temp(TEMP_SENSOR_PIN);
Timer timer;
Controller controller(conf, RELAY_PIN, temp, timer);
PIDTuner pidtuner(conf, temp, controller);
UI ui(conf, display, encoder, controller, pidtuner, temp, timer, network);
MQTTHandler mqtt(conf, controller, timer);

void check_temp_sensor()
{
    float t = DEVICE_DISCONNECTED_C;
    do {
        t = temp.get();

        if (t == DEVICE_DISCONNECTED_C) {
            ui.error_screen("Temp. sensor!");
            delay(500);
        }

    } while (t == DEVICE_DISCONNECTED_C);
}

void setup()
{
    Serial.begin(38400);

    // is setup mode requested? Holding down the encoder on start will cause that
    config_mode = analogRead(ROT_KEY_PIN) < 128; // hardcoded, should do

    // we enable programming over WiFi in setup mode
    if (config_mode) ArduinoOTA.begin();

    // TODO: Display Programming screen while programming over OTA

    // encoder button is digital, we do this by hand instead
    network.setup();

    encoder.setButtonHeldEnabled(true);
    encoder.setDoubleClickEnabled(false);
    // for esp8266, since pin no. zero is valid
    encoder.setButtonOnPinZeroEnabled(true);
    encoder.setHoldTime(500);

    temp.begin(conf.get_temp_offset());
    controller.begin();

    pidtuner.setup();

    ui.begin();

    mqtt.setup();

    if (mqtt.is_enabled()) {
        Serial.println("MQTT is enabled");
    } else {
        Serial.println("MQTT is disabled");
    }

    // check for temp sensor
    check_temp_sensor();
}

void loop()
{
    if (config_mode) ArduinoOTA.handle();
    network.update();

    // handle encoder input...
    static uint32_t last_service = 0;
    uint32_t ms = micros();

    // every millisecond we handle the encoder
    if (ms - last_service > 1000) {
        last_service = ms;
        encoder.service();
    }

    ui.update();

    mqtt.loop();

    // every ~minute we report to mqtt handler
    // - so in case MQTT is enabled, we get updates there and control commands back
    static unsigned long last_mqtt = 0;

    // every half minute is enough?
    if (ms - last_mqtt > 30000000) {
        last_mqtt = ms;

        Serial.println("MQTT update");
        const char *status = "unknown";

        switch (ui.infer_state()) {
        case UI::ST_AUTOTUNE: status = "autotune"; break;
        case UI::ST_CONFIG:   status = "config";   break;
        case UI::ST_DONE:     status = "done";     break;
        case UI::ST_SETUP:    status = "setup";    break;
        case UI::ST_PREHEAT:  status = "preheat";  break;
        case UI::ST_COOKING:  status = "cooking";  break;
        }

        mqtt.update(temp.get(), status);
    }
}
