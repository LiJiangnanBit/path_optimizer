//
// Created by ljn on 20-3-23.
//
#include "path_optimizer/data_struct/reference_path_impl.hpp"
#include "path_optimizer/data_struct/reference_path.hpp"
#include "path_optimizer/data_struct/data_struct.hpp"
#include "path_optimizer/tools/spline.h"
#include "path_optimizer/tools/tools.hpp"
#include "path_optimizer/tools/Map.hpp"

namespace PathOptimizationNS {

ReferencePath::ReferencePath() : reference_path_impl_(std::make_shared<ReferencePathImpl>()) {

}

const tk::spline &ReferencePath::getXS() const {
    return reference_path_impl_->getXS();
}

const tk::spline &ReferencePath::getYS() const {
    return reference_path_impl_->getYS();
}

double ReferencePath::getXS(double s) const {
    return reference_path_impl_->getXS()(s);
}

double ReferencePath::getYS(double s) const {
    return reference_path_impl_->getYS()(s);
}

void ReferencePath::clear() {
    reference_path_impl_->clear();
}

std::size_t ReferencePath::getSize() const {
    return reference_path_impl_->getSize();
}

double ReferencePath::getLength() const {
    return reference_path_impl_->getLength();
}

void ReferencePath::setLength(double s) {
    reference_path_impl_->setLength(s);
}

const std::vector<State> &ReferencePath::getReferenceStates() const {
    return reference_path_impl_->getReferenceStates();
}

const std::vector<CoveringCircleBounds> &ReferencePath::getBounds() const {
    return reference_path_impl_->getBounds();
}

const std::vector<double> &ReferencePath::getMaxKList() const {
    return reference_path_impl_->getMaxKList();
}

const std::vector<double> &ReferencePath::getMaxKpList() const {
    return reference_path_impl_->getMaxKpList();
}

std::vector<std::tuple<State, double, double>> ReferencePath::display_abnormal_bounds() const {
    return reference_path_impl_->display_abnormal_bounds();
}

void ReferencePath::setReference(const std::vector<PathOptimizationNS::State> &reference) {
    reference_path_impl_->setReference(reference);
}

void ReferencePath::setReference(const std::vector<PathOptimizationNS::State> &&reference) {
    reference_path_impl_->setReference(reference);
}

void ReferencePath::updateBounds(const Map &map) {
    reference_path_impl_->updateBoundsImproved(map);
}

void ReferencePath::updateLimits() {
    reference_path_impl_->updateLimits();
}

bool ReferencePath::buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger) {
    reference_path_impl_->buildReferenceFromSpline(delta_s_smaller, delta_s_larger);
}

void ReferencePath::setSpline(const PathOptimizationNS::tk::spline &x_s,
                              const PathOptimizationNS::tk::spline &y_s,
                              double max_s) {
    reference_path_impl_->setSpline(x_s, y_s, max_s);
}

void ReferencePath::setOriginalSpline(const PathOptimizationNS::tk::spline &x_s,
                                      const PathOptimizationNS::tk::spline &y_s,
                                      double max_s) {
    reference_path_impl_->setOriginalSpline(x_s, y_s, max_s);
}

}
