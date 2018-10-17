/*
 * AC_Flowermeter.h
 *
 *  Created on: 2016-4-12
 *      Author: liwh
 */

#ifndef AC_FLOWERMETER_H_
#define AC_FLOWERMETER_H_

class AC_Flowermeter
{
public:
    // constructor
	AC_Flowermeter();

    // destructor
    ~AC_Flowermeter(void);

    // static detection function
    static bool detect(void);

    // update state
    void update(void);
    void init();

    // get count
    uint64_t get_flowerVel(){ return _vel_flower; }
    uint64_t get_pwmCount(){ return _count; }
    uint64_t get_totallyCount() { return _count_totally; }
    uint64_t last_update() { return _last_timestamp; }
    uint64_t get_period() { return _period; }

    void set_valid(bool value){
    	_valid = value;
    }

    bool get_valid(){
    	return _valid;
    }
private:
    int _fd;
    uint64_t _count;
    uint64_t _last_count;
    uint64_t _last_timestamp;
    uint64_t _count_totally;
    uint64_t _period;

    uint16_t _vel_flower;

    bool _valid;
    // uint64_t _last_pulse_time_ms;
    // uint32_t _disable_time_ms;
    // uint32_t _good_sample_count;
    // float _last_sample_distance_cm;
};


#endif /* AC_FLOWERMETER_H_ */
