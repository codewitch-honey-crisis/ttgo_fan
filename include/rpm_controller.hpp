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
    uint32_t m_ts;
    unsigned int m_change_delay_ms;
    unsigned int m_rpm;
    volatile int m_count;
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
        m_ts = rhs.m_ts;
        m_change_delay_ms = rhs.m_change_delay_ms;
        m_rpm = rhs.m_rpm;
        m_count = rhs.m_count;
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
        ++this_->m_count;
    }
    
public:
    rpm_controller(rpm_controller&& rhs) {
        do_move(rhs);
    }
    rpm_controller& operator=(rpm_controller&& rhs) {
        do_move(rhs);
        return *this;
    }
    rpm_controller(uint8_t input_pin,uint8_t output_pin, uint32_t max_rpm=6500,uint32_t change_delay_ms=500,uint8_t channel = 0,unsigned int frequency=25*1000,uint8_t resolution=8) : m_initialized(false), m_input_pin(input_pin),m_output_pin(output_pin) ,m_ts(0),m_change_delay_ms(change_delay_ms), m_rpm(0), m_count(0),m_channel(channel),m_resolution(resolution),m_frequency(frequency),m_max_rpm(max_rpm),m_target_rpm(0),m_target_rpm_adj(0) {
    }
    ~rpm_controller() {
        if(m_initialized) {
            detachInterrupt(digitalPinToInterrupt(m_input_pin));
            ledcDetachPin(m_output_pin);
        }
    }
    void initialize() {
        if(!m_initialized) {
            m_rpm = 0;
            m_target_rpm = 0;
            m_ts = millis();
            m_count = 0;
            m_target_rpm_adj = 0;
            m_target_ts = 0;
            attachInterruptArg(digitalPinToInterrupt(m_input_pin),handler,this,RISING);
            ledcSetup(m_channel,m_frequency,m_resolution);
            ledcAttachPin(m_output_pin,m_channel);
            ledcWrite(m_channel,0);
            m_initialized = true;
        }
    }
    unsigned int rpm() const {
        if(m_initialized && m_ts!=0) {
            return m_rpm;
        }
        return 0;
    }
    unsigned int max_rpm() const {
        if(m_initialized) {
            return m_max_rpm;
        }
        return 0;
    }
    void rpm(unsigned int rpm) {
        if(m_initialized) {
            if(rpm>m_max_rpm) {
                rpm=m_max_rpm;
                m_target_rpm = rpm;
                m_target_rpm_adj = 0;
                int max = (1<<m_resolution)-1;
                int val = ((float)m_target_rpm/(float)m_max_rpm)*max;
                ledcWrite(m_channel,val);
                m_target_ts = millis();
            }
        }
    }
    void update() {
        if(m_initialized && m_ts>0) {
            uint32_t ms = millis();
            float msecs = (ms-m_ts);
            unsigned int old = m_count;
            int ticks = (int)(m_count/2.0+.5);
            double frac = 60000.0/msecs;
            m_rpm=(int)(ticks*frac);
            if(ms-m_ts>=60000) {
                m_count -=old;
                m_ts += 60000;
            }
            if(m_target_rpm!=m_rpm) {
                if(m_ts+m_change_delay_ms<=ms) {
                    int max = (1<<m_resolution)-1;
                    int rpm_delta = m_max_rpm/(1<<m_resolution);
                    int val = ((float)m_target_rpm/(float)m_max_rpm)*max;
                    if(m_rpm>m_target_rpm) {
                        int cmp = m_rpm+rpm_delta;
                        if(m_target_rpm-cmp<m_rpm-m_target_rpm) {
                            --m_target_rpm_adj;
                        }
                    } else {
                        int cmp = m_rpm-rpm_delta;
                        if(cmp-m_target_rpm<m_target_rpm-m_rpm) {
                            ++m_target_rpm_adj;
                        }
                    }
                    val+= m_target_rpm_adj;    
                    if(val<0) {
                        val = 0;
                    } else if(val>max) {
                        val = max;
                    }
                    ledcWrite(m_channel,val);
                    m_ts = millis();
                }
                

            }
        }
    }
};