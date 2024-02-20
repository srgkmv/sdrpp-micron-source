#include "micron.h"
#include <utils/flog.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <gui/style.h>
#include <config.h>
#include <gui/smgui.h>
#include <gui/widgets/stepped_slider.h>
#include <dsp/routing/stream_link.h>
#include <utils/optionlist.h>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "micron_source",
    /* Description:     */ "SDR-Micron source module for SDR++",
    /* Author:          */ "Sergey Komov",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ 1
};

ConfigManager config;

class MicronSourceModule : public ModuleManager::Instance {
public:
    MicronSourceModule(std::string name) {
        this->name = name;

        // Define samplerates
        samplerates.define(48000, "48KHz", micron::MICRON_SAMP_RATE_48KHZ);
        samplerates.define(96000, "96KHz", micron::MICRON_SAMP_RATE_96KHZ);
        samplerates.define(192000, "192KHz", micron::MICRON_SAMP_RATE_192KHZ);
        samplerates.define(240000, "240KHz", micron::MICRON_SAMP_RATE_240KHZ);
        samplerates.define(384000, "384KHz", micron::MICRON_SAMP_RATE_384KHZ);
        samplerates.define(480000, "480KHz", micron::MICRON_SAMP_RATE_480KHZ);
        samplerates.define(640000, "640KHz", micron::MICRON_SAMP_RATE_640KHZ);
        samplerates.define(768000, "768KHz", micron::MICRON_SAMP_RATE_768KHZ);
        samplerates.define(960000, "960KHz", micron::MICRON_SAMP_RATE_960KHZ);
        samplerates.define(1536000, "1536KHz", micron::MICRON_SAMP_RATE_1536KHZ);

        srId = samplerates.keyId(384000);

        lnk.init(NULL, &stream);

        sampleRate = 384000.0;

        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = &stream;

        sigpath::sourceManager.registerSource("Micron", &handler);
    }

    ~MicronSourceModule() {
        stop(this);
        sigpath::sourceManager.unregisterSource("Micron");
    }

    void postInit() {}

    void enable() {
        enabled = true;
    }

    void disable() {
        enabled = false;
    }

    bool isEnabled() {
        return enabled;
    }

    // TODO: Implement select functions

private:
    void refresh() {
        char serial[128];
        char buf[128];
        devices.clear();
        auto devList = micron::discover();
        for (auto& d : devList) {
            sprintf(serial, "%s", d.serial.c_str());
            sprintf(buf, "SDR-Micron [%s]", serial);
            devices.define(serial, buf, d);
        }
    }

    void selectSerial(std::string serial) {
        // If the device list is empty, don't select anything
        if (!devices.size()) {
            selectedSerial.clear();
            return;
        }

        // If the mac doesn't exist, select the first available one instead
        if (!devices.keyExists(serial)) {
            selectSerial(devices.key(0));
            return;
        }

        // Default config
        srId = samplerates.valueId(micron::MICRON_SAMP_RATE_384KHZ);
        att = 0;

        // Load config
        devId = devices.keyId(serial);
        selectedSerial = serial;
        config.acquire();
        if (config.conf["devices"][selectedSerial].contains("samplerate")) {
            int sr = config.conf["devices"][selectedSerial]["samplerate"];
            if (samplerates.keyExists(sr)) { srId = samplerates.keyId(sr); }
        }
        if (config.conf["devices"][selectedSerial].contains("att")) {
            att = config.conf["devices"][selectedSerial]["att"];
        }
        config.release();

        // Update host samplerate
        sampleRate = samplerates.key(srId);
        core::setInputSampleRate(sampleRate);
    }

    static void menuSelected(void* ctx) {
        MicronSourceModule* _this = (MicronSourceModule*)ctx;

        if (_this->firstSelect) {
            _this->firstSelect = false;

            // Refresh
            _this->refresh();

            // Select device
            config.acquire();
            _this->selectedSerial = config.conf["device"];
            config.release();
            _this->selectSerial(_this->selectedSerial);
        }

        core::setInputSampleRate(_this->sampleRate);
        flog::info("MicronSourceModule '{0}': Menu Select!", _this->name);
    }

    static void menuDeselected(void* ctx) {
        MicronSourceModule* _this = (MicronSourceModule*)ctx;
        flog::info("MicronSourceModule '{0}': Menu Deselect!", _this->name);
    }

    static void start(void* ctx) {
        MicronSourceModule* _this = (MicronSourceModule*)ctx;
        if (_this->running || _this->selectedSerial.empty()) { return; }
        
        // TODO: Implement start
        _this->dev = micron::open(_this->devices[_this->devId].serial);

        // TODO: STOP USING A LINK, FIND A BETTER WAY
        _this->lnk.setInput(&_this->dev->out);
        _this->lnk.start();
        _this->dev->start();

        // TODO: Check if the USB commands are accepted before start
        _this->dev->setSamplerate(_this->samplerates[_this->srId]);
        _this->dev->setFrequency(_this->freq);
        _this->dev->setAtt(_this->att);

        _this->running = true;
        flog::info("MicronSourceModule '{0}': Start!", _this->name);
    }

    static void stop(void* ctx) {
        MicronSourceModule* _this = (MicronSourceModule*)ctx;
        if (!_this->running) { return; }
        _this->running = false;
        
        // TODO: Implement stop
        _this->dev->stop();
        _this->dev->close();
        _this->lnk.stop();

        flog::info("MicronSourceModule '{0}': Stop!", _this->name);
    }

    static void tune(double freq, void* ctx) {
        MicronSourceModule* _this = (MicronSourceModule*)ctx;
        if (_this->running) {
            // TODO: Check if dev exists
            _this->dev->setFrequency(freq);
        }
        _this->freq = freq;
        flog::info("MicronSourceModule '{0}': Tune: {1}!", _this->name, freq);
    }

    static void menuHandler(void* ctx) {
        MicronSourceModule* _this = (MicronSourceModule*)ctx;

        if (_this->running) { SmGui::BeginDisabled(); }

        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo(CONCAT("##_micron_dev_sel_", _this->name), &_this->devId, _this->devices.txt)) {
            _this->selectSerial(_this->devices.key(_this->devId));
            if (!_this->selectedSerial.empty()) {
                config.acquire();
                config.conf["device"] = _this->devices.key(_this->devId);
                config.release(true);
            }
        }

        if (SmGui::Combo(CONCAT("##_micron_sr_sel_", _this->name), &_this->srId, _this->samplerates.txt)) {
            _this->sampleRate = _this->samplerates.key(_this->srId);
            core::setInputSampleRate(_this->sampleRate);
            if (!_this->selectedSerial.empty()) {
                config.acquire();
                config.conf["devices"][_this->selectedSerial]["samplerate"] = _this->samplerates.key(_this->srId);
                config.release(true);
            }
        }

        SmGui::SameLine();
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Button(CONCAT("Refresh##_micron_refr_", _this->name))) {
            _this->refresh();
            config.acquire();
            std::string mac = config.conf["device"];
            config.release();
            _this->selectSerial(mac);
        }

        if (_this->running) { SmGui::EndDisabled(); }

        // TODO: Device parameters

        SmGui::LeftLabel("ATT");
        SmGui::FillWidth();
        if (SmGui::SliderInt("##micron_source_att", &_this->att, 0, 31)) {
            if (_this->running) {
                _this->dev->setAtt(_this->att);
            }
            if (!_this->selectedSerial.empty()) {
                config.acquire();
                config.conf["devices"][_this->selectedSerial]["att"] = _this->att;
                config.release(true);
            }
        }
    }

    std::string name;
    bool enabled = true;
    dsp::stream<dsp::complex_t> stream;
    dsp::routing::StreamLink<dsp::complex_t> lnk;
    double sampleRate;
    SourceManager::SourceHandler handler;
    bool running = false;
    std::string selectedSerial = "";

    OptionList<std::string, micron::Info> devices;
    OptionList<int, micron::MicronSamplerate> samplerates;

    double freq;
    int devId = 0;
    int srId = 0;
    int att = 0;

    bool firstSelect = true;

    std::shared_ptr<micron::Client> dev;

};

MOD_EXPORT void _INIT_() {
    json def = json({});
    def["devices"] = json({});
    def["device"] = "";
    config.setPath(core::args["root"].s() + "/micron_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new MicronSourceModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    delete (MicronSourceModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
