#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"
#include "connectivity/threat_evaluation.hpp"
#include "local_planner/lattice_graph.hpp"
#include "stopwatch/stopwatch.h"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "lightviz/navviz.hpp"
#endif

using namespace librav;

int main()
{
    // load map
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    // 

    /****************************************************************************/

    // ------------------- vehicle 1 ---------------------- //

    CovarMatrix2d pos_covar1;
    pos_covar1 << 2, 0,
        0, 2;
    VehicleEstimation veh1({35, 59, -7 / 180.0 * M_PI}, pos_covar1, 10, 3 * 3);

    // ------------------- vehicle 2 ---------------------- //

    CovarMatrix2d pos_covar2;
    pos_covar2 << 1, 0,
        0, 1;
    VehicleEstimation veh2({89, 52, -7 / 180.0 * M_PI}, pos_covar2, 10, 2 * 2);

    // ------------------- vehicle 3 ---------------------- //

    CovarMatrix2d pos_covar3;
    pos_covar3 << 0.25, 0,
        0, 0.25;
    VehicleEstimation veh3({80, 59, 170 / 180.0 * M_PI}, pos_covar3, 15, 1 * 1);

    // ------------------- vehicle 4 ---------------------- //

    CovarMatrix2d pos_covar4;
    pos_covar4 << 1, 0,
        0, 1;
    VehicleEstimation veh4({52, 35, -95 / 180.0 * M_PI}, pos_covar4, 10, 2 * 2);

    // ------------------- vehicle 5 ---------------------- //

    CovarMatrix2d pos_covar5;
    pos_covar5 << 1, 0,
        0, 1;
    VehicleEstimation veh5({40, 64, 171 / 180.0 * M_PI}, pos_covar5, 10, 2 * 2);

    /****************************************************************************/

    // trajectory planning for ego vehicle
    auto all_channels = loader.traffic_map->GetAllTrafficChannels();
    auto ego_chn = loader.traffic_map->GetAllTrafficChannels()[2];
    ego_chn->DiscretizeChannel(2, 0.74, 3);

    VehicleEstimation ego_veh({56.5, 36, 85.0 / 180.0 * M_PI}, 10);

    stopwatch::StopWatch timer;
    std::vector<StateLattice> path;
    // auto graph = LatticeGraph::Search(path, ego_chn, {2, 0}, 8);
    auto graph = LatticeGraph::Search(path, ego_chn, ego_veh.GetPose(), 8);
    std::cout << "search finished in " << timer.toc() << " seconds" << std::endl;

    if (path.empty())
    {
        std::cout << "failed to find a path" << std::endl;
        return -1;
    }

    ReferenceTrajectory traj(path);
    traj.GenerateConstantSpeedProfile(ego_veh.GetSpeed());

    LookaheadZone zone(traj);

    // UGVNavViz::ShowLatticePathWithLookaheadZone(path, zone, *ego_chn.get(), "path with lookahead");

    /****************************************************************************/

    const int32_t eval_horizon = 8;

    timer.tic();
    ThreatEvaluation ranker(loader.road_map, loader.traffic_map);
    ranker.SetTrafficConfiguration(ego_veh, ego_chn, zone, {veh1, veh2, veh3, veh4, veh5});
    ranker.Evaluate(eval_horizon);
    std::cout << "evaluation finished in " << timer.toc() << " seconds" << std::endl;

#ifdef ENABLE_VIZ
    // UGVNavViz::ShowTrafficChannelWithThreatField(*ego_chn.get(), ranker.field_, 4, true, "lattice_in_threat_field", true);

    // for (int i = 0; i <= eval_horizon; i++)
    //     UGVNavViz::ShowThreatField(ranker.field_, i, true, "occupancy_estimation_new" + std::to_string(i), false);

    for (int32_t i = 0; i <= eval_horizon; ++i)
    {
        std::cout << "passing in: " << i << std::endl;
        UGVNavViz::ShowPathWithThreatField(loader.road_map, path, zone, ranker.field_, i, 0.5, "threat_exposure" + std::to_string(i), true);
    }
#endif

    return 0;
}