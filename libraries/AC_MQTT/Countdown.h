/*******************************************************************************
 * Copyright (c) 2014, 2017 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *    Ian Craggs - change Timer member initialization to avoid copy constructor
 *******************************************************************************/

#if !defined(COUNTDOWN_H)
#define COUNTDOWN_H

class Countdown
{
public:
    Countdown()
    {
		interval_end_ms = 0;
    }

    Countdown(uint32_t ms)
    {
        countdown_ms(ms);
    }

    bool expired()
    {
        return (interval_end_ms > 0) && (AP_HAL::millis() >= interval_end_ms);
    }

    void countdown_ms(uint32_t ms)
    {
        interval_end_ms = AP_HAL::millis() + ms;
    }

    void countdown(uint32_t seconds)
    {
        countdown_ms((uint32_t)seconds * 1000);
    }

    uint32_t left_ms()
    {
        return interval_end_ms - AP_HAL::millis();
    }

private:
    uint32_t interval_end_ms;
};

#endif

