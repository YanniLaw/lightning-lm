#include "ui/pangolin_window.h"

namespace lightning::ui {

PangolinWindow::PangolinWindow() = default;
PangolinWindow::~PangolinWindow() = default;

bool PangolinWindow::Init() { return true; }

void PangolinWindow::Reset(const std::vector<Keyframe::Ptr>&) {}

void PangolinWindow::UpdatePointCloudGlobal(const std::map<int, CloudPtr>&) {}

void PangolinWindow::UpdatePointCloudDynamic(const std::map<int, CloudPtr>&) {}

void PangolinWindow::UpdateNavState(const NavState&) {}

void PangolinWindow::UpdateRecentPose(const SE3&) {}

void PangolinWindow::UpdatePredictPose(const SE3&) {}

void PangolinWindow::UpdateScan(CloudPtr, const SE3&) {}

void PangolinWindow::UpdateKF(std::shared_ptr<Keyframe>) {}

void PangolinWindow::Quit() {}

bool PangolinWindow::ShouldQuit() { return false; }

void PangolinWindow::SetTImuLidar(const SE3&) {}

void PangolinWindow::SetCurrentScanSize(int) {}

}  // namespace lightning::ui
