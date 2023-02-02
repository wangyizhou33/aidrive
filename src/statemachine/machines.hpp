#ifndef MACHINES_HPP_
#define MACHINES_HPP_

#include "tinyfsm.hpp"
#include "states.hpp"

class Main : public tinyfsm::Fsm<Main>
{


private:
    MainState state_;
};


class Lon : public tinyfsm::Fsm<Lon>
{


private:
    LonState state_;
};


class Lat : public tinyfsm::Fsm<Lat>
{


private:
    LatState state_;
};

class HDMap : public tinyfsm::Fsm<HDMap>
{


private:
    HDMapState state_;
};

class AEB : public tinyfsm::Fsm<AEB>
{

private:
    AEBState state_;
};


#endif // MACHINES_HPP_