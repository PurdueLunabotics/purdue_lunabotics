#ifndef __FSM__
#define __FSM__

enum class DepState
{
    STORED,
    FULL_EXT,
    CNT,
    STOPPED
};

enum class BinState
{
    EMPTY,
    FILLING,
    FULL,
};

enum class ExcState
{
    NOMINAL,
    OVERCURRENT,
};

enum class LinActState
{
    STORED = -1,
    DRIVING,  // move lin act
    FULL_EXT, 
    CNT,
    STOPPED
};

enum class LeadScrewState
{
    STORED,
    FULL_EXT,
    CNT,
    STOPPED
};

#endif