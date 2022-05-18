enum class DepState
{
    STORED,
    FULL_EXT,
    CNT
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
    INIT = -1,
    STORED,    // move lin act
    START_EXC, // start excavating slowly
    FULL_EXT,  // excavate max speed
    STOP_EXC,  // stop lin act
    CNT,
};

enum class LeadScrewState
{
    STORED,
    FULL_EXT,
    CNT
};