#ifndef STATES_HPP_
#define STATES_HPP_


enum class MainState
{
    INVALID,
    OFF,
    ACC,
    ICA,
    NDA,
    MRM,
    HPA,
    APA
};

enum class LonState
{
    INVALID,
    OFF_NOT_READY,
    OFF_READY,
    CONTROLLING,
    OVERRIDEN,
    WAITING,
    HOLDING
};

enum class LatState
{
    INVALID,
    OFF_NOT_READY,
    OFF_READY,
    CONTROLLING,
    OVERRIDEN,
};

enum class HDMapState
{
    INVALID,
    ON,
    OFF
};

enum class AEBState
{
    INVALID,
    OFF,
    MONITORING,
    ACTUATING
};

#endif // STATES_HPP_