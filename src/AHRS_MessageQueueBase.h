#include <cstdint>

struct xyz_t;

/*!
AHRS_MessageQueueBase, used to signal that data is available for Blackbox.
*/
class AHRS_MessageQueueBase {
public:
    virtual uint32_t append(uint32_t timeMicroSeconds, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc) = 0;
};
