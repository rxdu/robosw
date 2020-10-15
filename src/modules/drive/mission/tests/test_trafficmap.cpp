#include <iostream>

// #include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"
#include "stopwatch/stopwatch.h"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "lightviz/navviz.hpp"
#endif

using namespace ivnav;

int main()
{
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    // RoadMapViz::SetupRoadMapViz(loader.road_map);

    auto ids = loader.road_map->FindOccupiedLanelet(CartCooridnate(55, 56));
    std::cout << "occupied laneles: " << ids.size() << std::endl;

    for (auto &chn : loader.traffic_map->GetAllTrafficChannels())
    {
        chn->PrintInfo();
#ifdef ENABLE_VIZ
        UGVNavViz::ShowTrafficChannel(loader.road_map, chn.get());
#endif
    }

    // RoadMapViz::ShowTrafficChannel(map->traffic_map_->GetAllTrafficChannels().front());

    return 0;
}