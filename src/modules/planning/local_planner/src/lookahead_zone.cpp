/* 
 * lookahead_zone.cpp
 * 
 * Created on: Nov 14, 2018 07:35
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "local_planner/lookahead_zone.hpp"

using namespace ivnav;

LookaheadZone::LookaheadZone(ReferenceTrajectory curve, double s_step) : CurvilinearGridBase<ThreatField::ThreatComponent, ReferenceTrajectory>(curve, s_step, DStep, DNum, SOffset),
                                                                         trajectory_(curve)
{
}

void LookaheadZone::SetupLookaheadZone(ReferenceTrajectory curve, double s_step)
{
    trajectory_ = curve;
    SetupGrid(curve, s_step, DStep, DNum, SOffset);
}
