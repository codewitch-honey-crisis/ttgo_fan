#pragma once
#include <Arduino.h>
// declare rpm_controller foo(input_pin,output_pin);
// in setup() call foo.initialize();
// in loop() call foo.update();
// call foo.rpm() to check the rpm

class rpm_controller final {
    bool m_initialized;
    uint8_t m_input_pin;
    uint8_t m_output_pin;
    uint32_t m_pulses_per_rev;
    int m_state;
    unsigned int m_change_delay_ms;
    unsigned int m_rpm;
    volatile unsigned long m_micros;
    volatile unsigned long m_micros_old;
    uint8_t m_channel;
    uint8_t m_resolution;
    unsigned int m_frequency;
    unsigned int m_max_rpm;
    unsigned int m_target_rpm;
    int m_target_rpm_adj;
    uint32_t m_target_ts;
    
    rpm_controller(const rpm_controller& rhs)=delete;
    rpm_controller& operator=(const rpm_controller& rhs)=delete;
    void do_move(rpm_controller& rhs) {
        m_initialized = rhs.m_initialized;
        rhs.m_initialized = false;
        m_input_pin = rhs.m_input_pin;
        m_output_pin = rhs.m_output_pin;
        m_state = rhs.m_state;
        m_change_delay_ms = rhs.m_change_delay_ms;
        m_rpm = rhs.m_rpm;
        m_micros = rhs.m_micros;
        m_micros_old = rhs.m_micros_old;
        m_channel = rhs.m_channel;
        m_resolution = rhs.m_resolution;
        m_frequency = rhs.m_frequency;
        m_max_rpm = rhs.m_max_rpm;
        m_target_rpm = rhs.m_target_rpm;
        m_target_rpm_adj = rhs.m_target_rpm_adj;
        m_target_ts = rhs.m_target_ts;
    }
#if ESP32
    IRAM_ATTR
#endif
    static void handler(void* arg) {
        rpm_controller* this_ = (rpm_controller*)arg;
        this_->m_micros_old = this_->m_micros;
        this_->m_micros = micros();
    }
    
public:
    rpm_controller(rpm_controller&& rhs) {
        do_move(rhs);
    }
    rpm_controller& operator=(rpm_controller&& rhs) {
        do_move(rhs);
        return *this;
    }
    rpm_controller(uint8_t input_pin,uint8_t output_pin, uint32_t max_rpm=0,uint32_t pulses_per_rev=2, uint32_t change_delay_ms=500,uint8_t channel = 0,unsigned int frequency=25*1000,uint8_t resolution=8) : m_initialized(false), m_input_pin(input_pin),m_output_pin(output_pin),m_pulses_per_rev(pulses_per_rev), m_state(-1),m_change_delay_ms(change_delay_ms), m_rpm(0), m_micros(0),m_micros_old(0), m_channel(channel),m_resolution(resolution),m_frequency(frequency),m_max_rpm(max_rpm),m_target_rpm(0),m_target_rpm_adj(0) {
    }
    ~rpm_controller() {
        if(initialized()) {
            detachInterrupt(digitalPinToInterrupt(m_input_pin));
            ledcDetachPin(m_output_pin);
        }
    }
    void initialize() {
        if(m_state==-1) {
            m_rpm = 0;
            m_target_rpm = 0;
            m_state = 0;
            m_micros = 0;
            m_micros_old = 0;
            m_target_rpm_adj = 0;
            m_target_ts = 0;
            attachInterruptArg(digitalPinToInterrupt(m_input_pin),handler,this,RISING);
            ledcSetup(m_channel,m_frequency,m_resolution);
            ledcAttachPin(m_output_pin,m_channel);
            if(m_max_rpm==0) {
                m_state = -2;
                m_target_ts = millis();
            } else {
                ledcWrite(m_channel,0);
            }
        }
    }
    bool initialized() const {
        return m_state!=-1;
    }
    unsigned int rpm() const {
        if(initialized()) {
            return m_rpm;
        }
        return 0;
    }
    unsigned int max_rpm() const {
        if(initialized()) {
            return m_max_rpm;
        }
        return 0;
    }
    void rpm(unsigned int rpm) {
        if(initialized()) {
            if(rpm>m_rpm) {
                m_state = 1;
                m_target_rpm = rpm;
                m_target_ts = millis();
            } else if(rpm<m_rpm) {
                m_state = 2;
                m_target_rpm = rpm;
                m_target_ts = millis();
            }
        }
    }
    void update() {
        if(initialized()) {
            // wrap around = initial or bad read:
            if(m_micros<=m_micros_old) return;
            // count of microseconds the last rev took
            unsigned long dur_micros = (m_micros-m_micros_old)*m_pulses_per_rev;
            m_rpm = ((6e+7)/(double)dur_micros+.5);
            uint32_t ms = millis();
            switch(m_state) {
                case -2: // autodetect max rpm
                    if(m_rpm==0) {
                        ledcWrite(m_channel,(1<<m_resolution)-1);
                    }
                    // give time for spinup
                    if(ms-m_target_ts>=(m_change_delay_ms*5) && m_rpm!=0) {
                        m_max_rpm = m_rpm;
                        m_state = 0;
                    }
                    break;
                case 0: // nothing needed
                    break;
                case 1: // changing up (target RPM > RPM)
                    if(ms-m_target_ts>=m_change_delay_ms) {
                        if(m_rpm>m_target_rpm) {
                            --m_target_rpm_adj;
                            if(m_target_rpm_adj<0) {
                                m_target_rpm_adj = 0;
                            }
                            ledcWrite(m_channel,(uint32_t)m_target_rpm_adj);
                            m_state = 0;
                        } else if(m_rpm<m_target_rpm) {
                            ++m_target_rpm_adj;
                            uint32_t max = (1<<m_resolution)-1;
                            if(m_target_rpm_adj>max) {
                                m_target_rpm_adj = max;
                                m_state = 0;
                            }
                            ledcWrite(m_channel,(uint32_t)m_target_rpm_adj);
                        } else {
                            m_state = 0;
                        }
                        m_target_ts = millis();
                    }
                    break;
                case 2: // changing down (target RPM < RPM)
                    if(ms-m_target_ts>=m_change_delay_ms) {
                        if(m_rpm<m_target_rpm) {
                            uint32_t max = (1<<m_resolution)-1;
                            ++m_target_rpm_adj;
                            if(m_target_rpm_adj>=max) {
                                m_target_rpm_adj = max;
                            }
                            ledcWrite(m_channel,(uint32_t)m_target_rpm_adj);
                            m_state = 0;
                        } else if(m_rpm>m_target_rpm) {
                            --m_target_rpm_adj;
                            if(m_target_rpm_adj<0) {
                                m_target_rpm_adj = 0;
                                m_state = 0;
                            }
                            ledcWrite(m_channel,(uint32_t)m_target_rpm_adj);
                        } else {
                            m_state = 0;
                        }
                        m_target_ts = millis();
                    }
                    break;
            }
        
        }
    }
};