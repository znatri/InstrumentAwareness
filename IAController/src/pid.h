#include "../include/ErrorDef.h"

class PID {
public:
    PID(float kp, float kd, float ki, int timePeriod_ms) :
        kp(kp), kd(kd), ki(ki), m_tp(timePeriod_ms * 0.001f) {}

    float step(float measurement, float setPoint, float offset = 0) {
        float e = setPoint - measurement;
        float p = kp * e;
        m_fI += ki * e * m_tp;
        float d = kd * (e - m_fE) / m_tp;
        m_fE = e;
        return offset + p + m_fI + d;
    }

    int32_t step(int32_t measurement, int32_t setPoint, int32_t offset = 0) {
        int32_t e = setPoint - measurement;
        float p = kp * e;
        m_fI += ki * e * m_tp;
        float d = kd * (e - m_iE) / m_tp;
        m_iE = e;
        return offset + (int32_t) roundf(p + m_fI + d);
    }

private:
    float kp, kd, ki;
    float m_tp;
    float m_fE = 0;
    float m_fI = 0;
    int32_t m_iE = 0;
};