#ifndef MovementCommander_HPP
#define MovementCommander_HPP

#include <cmath>
#include <px4_platform_common/px4_config.h>

#include <uORB/uORB.h>

class MovementCommander {



public:
    MovementCommander();
    ~MovementCommander();

    int Movingtest();
    int activation1(int expectedn);
    void repostest();
    int gototest();
    int repos(float x,float y,float z);


};


#endif /* GPS_CONTROLLER_HPP */
