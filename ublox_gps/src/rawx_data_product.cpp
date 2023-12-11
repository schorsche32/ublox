#include <memory>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_gps/gps.hpp>
#include <ublox_gps/rawx_data_product.hpp>
#include <ublox_gps/ublox_topic_diagnostic.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// RawX Data Products
//
RawXDataProduct::RawXDataProduct(uint16_t nav_rate, uint16_t meas_rate, std::shared_ptr<diagnostic_updater::Updater> updater, rclcpp::Node* node)
  : nav_rate_(nav_rate), meas_rate_(meas_rate), updater_(updater), node_(node) {
  if (getRosBoolean(node_, "publish.rxm.raw")) {
    rxm_rawx_pub_ = node_->create_publisher<ublox_msgs::msg::RxmRAWX>("rxmraw", 1);
  }
  if (getRosBoolean(node_, "publish.rxm.sfrb")) {
    rxm_sfrbx_pub_ = node_->create_publisher<ublox_msgs::msg::RxmSFRBX>("rxmsfrbx", 1);
  }
}

void RawXDataProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to RXM RawX
  if (getRosBoolean(node_, "publish.rxm.raw")) {
    gps->subscribe<ublox_msgs::msg::RxmRAWX>([this](const ublox_msgs::msg::RxmRAWX &m) { rxm_rawx_pub_->publish(m); },
                                       1);
  }
  // Subscribe to RXM SFRBX
  if (getRosBoolean(node_, "publish.rxm.sfrbx")) {
    gps->subscribe<ublox_msgs::msg::RxmSFRBX>([this](const ublox_msgs::msg::RxmSFRBX &m) { rxm_sfrbx_pub_->publish(m); },
                                        1);
  }
}

void RawXDataProduct::initializeRosDiagnostics() {
  if (getRosBoolean(node_, "publish.rxm.raw")) {
    freq_diagnostics_.push_back(std::make_shared<UbloxTopicDiagnostic>(
      "rxmraw", kRtcmFreqTol, kRtcmFreqWindow, nav_rate_, meas_rate_, updater_));
  }
  if (getRosBoolean(node_, "publish.rxm.sfrb")) {
    freq_diagnostics_.push_back(std::make_shared<UbloxTopicDiagnostic>(
      "rxmsfrb", kRtcmFreqTol, kRtcmFreqWindow, nav_rate_, meas_rate_, updater_));
  }
}

}  // namespace ublox_node