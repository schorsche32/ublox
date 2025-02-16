#ifndef UBLOX_GPS_RAWX_DATA_PRODUCT_HPP
#define UBLOX_GPS_RAWX_DATA_PRODUCT_HPP

#include <memory>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/rxm_rawx.hpp>
#include <ublox_msgs/msg/rxm_sfrbx.hpp>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gps.hpp>
#include <ublox_gps/ublox_topic_diagnostic.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for Raw Data products.
 */
class RawXDataProduct final : public virtual ComponentInterface {
 public:
  const double kRtcmFreqTol = 0.15;
  const int kRtcmFreqWindow = 25;

  explicit RawXDataProduct(uint16_t nav_rate, uint16_t meas_rate, std::shared_ptr<diagnostic_updater::Updater> updater, rclcpp::Node* node);

  /**
   * @brief Does nothing since there are no RawX Data product specific settings.
   */
  void getRosParams() override {}

  /**
   * @brief Does nothing since there are no RawX Data product specific settings.
   * @return always returns true
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override {
    (void)gps;
    return true;
  }

  /**
   * @brief Adds frequency diagnostics for RTCM topics.
   */
  void initializeRosDiagnostics() override;

  /**
   * @brief Subscribe to RawX Data Product messages and set up ROS publishers.
   *
   * @details Subscribe to RxmRAWX
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 private:
  //! Topic diagnostic updaters
  std::vector<std::shared_ptr<UbloxTopicDiagnostic> > freq_diagnostics_;

  rclcpp::Publisher<ublox_msgs::msg::RxmRAWX>::SharedPtr rxm_rawx_pub_;
  rclcpp::Publisher<ublox_msgs::msg::RxmSFRBX>::SharedPtr rxm_sfrbx_pub_;

  uint16_t nav_rate_;
  uint16_t meas_rate_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  rclcpp::Node* node_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_RAWX_DATA_PRODUCT_HPP