/*
 * quad_planner.h
 *
 *  Created on: Aug 1, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PLANNER_QUAD_PLANNER_H_
#define PLANNING_SRC_PLANNER_QUAD_PLANNER_H_

#include <memory>
#include "opencv2/opencv.hpp"

// headers for lcm
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/comm.hpp"

#include "common/planning_types.h"
#include "planner/graph_planner.h"
#include "planner/rrts_planner.h"
#include "map/map_info.h"

namespace srcl_ctrl {

enum class GraphPlannerType {
	SQUAREGRID_PLANNER,
	QUADTREE_PLANNER,
	NOT_SPECIFIED
};

class QuadPlanner{
public:
	QuadPlanner();
	QuadPlanner(std::shared_ptr<lcm::LCM> lcm);
	~QuadPlanner();

private:
	// lcm
	std::shared_ptr<lcm::LCM> lcm_;

	// planners
	GraphPlanner<QuadTree> qtree_planner_;
	GraphPlanner<SquareGrid> sgrid_planner_;

	RRTStarPlanner local_planner_;

	// planning parameters
	Position2D start_pos_;
	Position2D goal_pos_;

	bool world_size_set_;

public:
	GraphPlannerType active_graph_planner_;

public:
	void ConfigGraphPlanner(MapConfig config);
	void SetStartMapPosition(Position2D pos);
	void SetGoalMapPosition(Position2D pos);

	void ConfigRRTSOccupancyMap(cv::Mat map, MapInfo info);
	void SetRealWorldSize(double x, double y);

	std::vector<uint64_t> SearchForGlobalPath();
	bool SearchForLocalPath(Position2Dd start, Position2Dd goal, double time_limit, std::vector<Position2Dd>& path2d);

	// for visualization
	cv::Mat GetActiveMap();
	MapInfo GetActiveMapInfo();
	std::shared_ptr<Graph_t<RRTNode>> GetLocalPlannerVisGraph();

	// lcm
	void LcmTransformHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::QuadrotorTransform* msg);
};

}


#endif /* PLANNING_SRC_PLANNER_QUAD_PLANNER_H_ */
