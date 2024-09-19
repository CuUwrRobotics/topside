#pragma once

#include <ros/ros.h>

#include <chrono>
#include <cstdint>

enum class LockoutState : std::int8_t
{
    FREE                  = 1,
    ENTER_LOCKED__HOLD    = 2,
    ENTER_LOCKED__RELEASE = -2,
    LOCKED                = -1,
    EXIT_LOCKED__HOLD     = -3,
    EXIT_LOCKED__RELEASE  = 3,
};

class LockoutStateMachine
{
  private:
    bool*           m_Button1;
    bool*           m_Button2;
    LockoutState    m_State;
    struct timespec m_Time;
    double          m_StopTime;
    double          m_HoldTime;

  public:
    LockoutStateMachine(const bool* button1,
                        const bool* button2,
                        const float holdTime = 3)
    {
        this->m_Button1  = button1;
        this->m_Button2  = button2;
        this->m_State    = LockoutState::LOCKED;
        this->m_HoldTime = holdTime;
    }

    bool isLocked()
    {
        static double             now;
        std::chrono::milliseconds ms
            = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());
        now = static_cast<double>(ms.count()) / 1000.0;

        switch (m_State)
        {
        case LockoutState::FREE:
            if (*m_Button1 || *m_Button2)
            {
                m_State    = LockoutState::ENTER_LOCKED__HOLD;
                m_StopTime = now + 0.05; // Give a little m_Time for debounce
            }
            break;
        case LockoutState::ENTER_LOCKED__HOLD:
            if (!(*m_Button1 || *m_Button2))
            {
                // Buttons were released early
                m_State = LockoutState::FREE;
            }
            if (now >= m_StopTime)
            {
                m_State = LockoutState::ENTER_LOCKED__RELEASE;
            }
            break;
        case LockoutState::ENTER_LOCKED__RELEASE:
            if (!(*m_Button1 || *m_Button2))
            {
                m_State = LockoutState::LOCKED;
            }
            break;
        case LockoutState::LOCKED:
            if (*m_Button1 && *m_Button2)
            {
                m_State    = LockoutState::EXIT_LOCKED__HOLD;
                m_StopTime = now + m_HoldTime;
            }
            break;
        case LockoutState::EXIT_LOCKED__HOLD:
            if (!(*m_Button1 && *m_Button2))
            {
                m_State = LockoutState::LOCKED;
                break;
            }
            if (now >= m_StopTime)
            {
                m_State = LockoutState::EXIT_LOCKED__RELEASE;
            }
            break;

        case LockoutState::EXIT_LOCKED__RELEASE:
            if (!(*m_Button1 && *m_Button2))
            {
                m_State = LockoutState::FREE;
            }
            break;
        default:
            assert(false);
            break;
        }

        return m_State < 0;
    }
};
