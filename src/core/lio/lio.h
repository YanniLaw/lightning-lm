#ifndef LIGHTNING_LIO_H
#define LIGHTNING_LIO_H

#include <functional>
#include <memory>
#include <string>

#include "common/imu.h"
#include "common/odom.h"
#include "common/sensor_data.h"
#include "core/lio/lio_data.h"
#include "core/lio/lio_result.h"

namespace lightning {

enum class LIOInputStatus {
    kAccepted,
    kInvalidData,
    kNotRunning,
    kUnsupported,
    kTimeDiscontinuity,
};

enum class LIOUsage {
    kMapping,
    kLocalization,
};

struct LIOOptions {
    LIOUsage usage = LIOUsage::kMapping;
};

/// Transport-independent interface shared by mapping and localization.
class LIO {
   public:
    using ResultCallback = std::function<void(const LIOResult&)>;
    using PredictionCallback = std::function<void(const LIOState&)>;
    using DataCallback = std::function<void(const LIOData&)>;

    virtual ~LIO() = default;

    virtual bool Init(const std::string& config_path, const LIOOptions& options) = 0;

    virtual void SetResultCallback(ResultCallback callback) = 0;
    virtual void SetPredictionCallback(PredictionCallback callback) = 0;
    virtual void SetDataCallback(DataCallback callback) = 0;

    virtual LIOInputStatus AddImu(const IMUPtr& imu) = 0;
    // Points are in the actual IMU frame; header.stamp is scan start in us.
    // After submission the caller must not modify the cloud; LIO is its writer.
    virtual LIOInputStatus AddPointCloud(const CloudPtr& scan) = 0;
    virtual LIOInputStatus AddWheelOdometry(const OdomPtr& data) = 0;

    virtual void Stop() = 0;
};

/// Creates a frontend implementation by its configuration name.
std::unique_ptr<LIO> CreateLIO(const std::string& type);

}  // namespace lightning

#endif  // LIGHTNING_LIO_H
