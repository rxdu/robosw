/*
 * quad_path_repair.cpp
 *
 *  Created on: Sep 9, 2016
 *      Author: rdu
 */

#include <iostream>
#include <ctime>
#include <cmath>
#include <limits>

#include "path_repair/quad_path_repair.h"
#include "map/map_utils.h"
#include "geometry/cube_array/cube_array.h"
#include "geometry/cube_array_builder.h"
#include "geometry/graph_builder.h"

using namespace srcl_ctrl;

QuadPathRepair::QuadPathRepair(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		octomap_server_(OctomapServer(lcm_)),
		active_graph_planner_(GraphPlannerType::NOT_SPECIFIED),
		current_sys_time_(0),
		gstart_set_(false),
		ggoal_set_(false),
		world_size_set_(false),
		auto_update_pos_(true),
		update_global_plan_(false),
		init_plan_found_(false),
		est_dist2goal_(0)
{
	if(!lcm_->good())
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
	else {
		traj_gen_ = std::make_shared<TrajectoryGenerator>(lcm_);

		lcm_->subscribe("quad_data/quad_transform",&QuadPathRepair::LcmTransformHandler, this);
		lcm_->subscribe("quad_planner/new_octomap_ready",&QuadPathRepair::LcmOctomapHandler, this);
		lcm_->subscribe("quad_ctrl/mission_info",&QuadPathRepair::LcmMissionInfoHandler, this);
		lcm_->subscribe("quad_data/system_time", &QuadPathRepair::LcmSysTimeHandler, this);
	}
}

QuadPathRepair::~QuadPathRepair()
{

}

void QuadPathRepair::ConfigGraphPlanner(MapConfig config, double world_size_x, double world_size_y)
{
	if(config.GetMapType().data_model == MapDataModel::QUAD_TREE)
	{
		bool result = qtree_planner_.UpdateMapConfig(config);

		if(result)
		{
			std::cout << "quad tree planner activated" << std::endl;
			active_graph_planner_ = GraphPlannerType::QUADTREE_PLANNER;

			//gcombiner_.SetBaseGraph(qtree_planner_.graph_, qtree_planner_.map_.data_model, qtree_planner_.map_.data_model->leaf_nodes_.size(), qtree_planner_.map_.info);
		}
	}
	else if(config.GetMapType().data_model == MapDataModel::SQUARE_GRID)
	{
		bool result = sgrid_planner_.UpdateMapConfig(config);

		if(result)
		{
			std::cout << "square grid planner activated" << std::endl;
			active_graph_planner_ = GraphPlannerType::SQUAREGRID_PLANNER;
		}
	}

	// the world size must be set after the planner is updated, otherwise the configuration will be override
	qtree_planner_.map_.info.SetWorldSize(world_size_x, world_size_y);
	sgrid_planner_.map_.info.SetWorldSize(world_size_x, world_size_y);
	world_size_set_ = true;

	if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
		gcombiner_.SetBaseGraph(sgrid_planner_.graph_, sgrid_planner_.map_.data_model, sgrid_planner_.map_.data_model->cells_.size(), sgrid_planner_.map_.info);

	srcl_lcm_msgs::Graph_t graph_msg = GenerateLcmGraphMsg();
	lcm_->publish("quad_planner/quad_planner_graph", &graph_msg);
}

void QuadPathRepair::SetStartMapPosition(Position2D pos)
{
	if(pos == start_pos_)
		return;

	start_pos_.x = pos.x;
	start_pos_.y = pos.y;

	gstart_set_ = true;

	est_dist2goal_ = std::numeric_limits<double>::infinity();
	init_plan_found_ = false;

	if(gstart_set_ && ggoal_set_)
		update_global_plan_ = true;
}

void QuadPathRepair::SetGoalMapPosition(Position2D pos)
{
	goal_pos_.x = pos.x;
	goal_pos_.y = pos.y;

	ggoal_set_ = true;

	if(gstart_set_ && ggoal_set_)
		update_global_plan_ = true;
}

void QuadPathRepair::SetStartRefWorldPosition(Position2Dd pos)
{
	Position2Dd mpos;
	mpos = MapUtils::CoordinatesFromRefWorldToMapWorld(pos, GetActiveMapInfo());

	Position2D map_pos;
	map_pos = MapUtils::CoordinatesFromMapWorldToMap(mpos, GetActiveMapInfo());

	Position2D map_padded_pos;
	map_padded_pos = MapUtils::CoordinatesFromOriginalToPadded(map_pos, GetActiveMapInfo());

	SetStartMapPosition(map_padded_pos);
}

void QuadPathRepair::SetGoalRefWorldPosition(Position2Dd pos)
{
	Position2Dd mpos;
	mpos = MapUtils::CoordinatesFromRefWorldToMapWorld(pos, GetActiveMapInfo());

	Position2D map_pos;
	map_pos = MapUtils::CoordinatesFromMapWorldToMap(mpos, GetActiveMapInfo());

	Position2D map_padded_pos;
	map_padded_pos = MapUtils::CoordinatesFromOriginalToPadded(map_pos, GetActiveMapInfo());

	SetGoalMapPosition(map_padded_pos);
}

std::vector<Position2D> QuadPathRepair::UpdateGlobalPath()
{
	std::vector<Position2D> waypoints;

//	std::cout << "----> start: " << start_pos_.x << " , " << start_pos_.y << std::endl;
//	std::cout << "----> goal: " << goal_pos_.x << " , " << goal_pos_.y << std::endl;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		auto traj_vtx = qtree_planner_.Search(start_pos_, goal_pos_);
		for(auto& wp:traj_vtx)
			waypoints.push_back(wp->bundled_data_->location_);
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		auto traj_vtx = sgrid_planner_.Search(start_pos_, goal_pos_);
		for(auto& wp:traj_vtx)
			waypoints.push_back(wp->bundled_data_->location_);
	}

	srcl_lcm_msgs::Path_t path_msg = GenerateLcmPathMsg(waypoints);
	lcm_->publish("quad_planner/quad_planner_graph_path", &path_msg);

	update_global_plan_ = false;

	return waypoints;
}

std::vector<uint64_t> QuadPathRepair::UpdateGlobalPathID()
{
	std::vector<uint64_t> waypoints;

//		std::cout << "----> start: " << start_pos_.x << " , " << start_pos_.y << std::endl;
//		std::cout << "----> goal: " << goal_pos_.x << " , " << goal_pos_.y << std::endl;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		auto traj_vtx = qtree_planner_.Search(start_pos_, goal_pos_);
		for(auto& wp:traj_vtx)
			waypoints.push_back(wp->vertex_id_);
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		auto traj_vtx = sgrid_planner_.Search(start_pos_, goal_pos_);
		for(auto& wp:traj_vtx)
			waypoints.push_back(wp->vertex_id_);
	}

	update_global_plan_ = false;

	return waypoints;
}

cv::Mat QuadPathRepair::GetActiveMap()
{
	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		return qtree_planner_.map_.padded_image;
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		return sgrid_planner_.map_.padded_image;
	}
	else
		return cv::Mat::zeros(10, 10, CV_8UC1);
}

MapInfo QuadPathRepair::GetActiveMapInfo()
{
	MapInfo empty_info;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		return qtree_planner_.map_.info;
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		return sgrid_planner_.map_.info;
	}
	else
		return empty_info;
}

void QuadPathRepair::LcmTransformHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_lcm_msgs::QuadrotorTransform* msg)
{
	Position2Dd rpos;
	rpos.x = msg->base_to_world.position[0];
	rpos.y = msg->base_to_world.position[1];

	if(auto_update_pos_)
		SetStartRefWorldPosition(rpos);

	gcombiner_.UpdateVehiclePose(Position3Dd(msg->base_to_world.position[0],msg->base_to_world.position[1],msg->base_to_world.position[2]),
					Eigen::Quaterniond(msg->base_to_world.quaternion[0] , msg->base_to_world.quaternion[1] , msg->base_to_world.quaternion[2] , msg->base_to_world.quaternion[3]));
	mission_tracker_.UpdateCurrentPosition(Position3Dd(msg->base_to_world.position[0],msg->base_to_world.position[1],msg->base_to_world.position[2]));
}

void QuadPathRepair::LcmMissionInfoHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_lcm_msgs::MissionInfo_t* msg)
{
	est_dist2goal_ = msg->dist_to_goal;
}

void QuadPathRepair::LcmSysTimeHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_lcm_msgs::TimeStamp_t* msg)
{
	current_sys_time_ = msg->time_stamp;
}

void QuadPathRepair::LcmOctomapHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_lcm_msgs::NewDataReady_t* msg)
{
	static int count = 0;
	//std::cout << " test reading: " << octomap_server_.octree_->getResolution() << std::endl;
	srcl_lcm_msgs::KeyframeSet_t kf_cmd;

	// record the planning time
	kf_cmd.sys_time.time_stamp = current_sys_time_;

	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(octomap_server_.octree_);
	std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

	std::cout << "cube graph size: " << cubegraph->GetGraphVertices().size() << std::endl;

//	bool combine_success = gcombiner_.CombineBaseWithCubeArrayGraph(cubearray, cubegraph);
//	if(!combine_success)
//		return;

	uint64_t geo_start_id_astar = gcombiner_.CombineBaseWithCubeArrayGraph(cubearray, cubegraph);

	//uint64_t map_start_id = sgrid_planner_.map_.data_model->GetIDFromPosition(start_pos_.x, start_pos_.y);
	//uint64_t geo_start_id_astar = sgrid_planner_.graph_->GetVertexFromID(map_start_id)->bundled_data_->geo_mark_id_;

	uint64_t map_goal_id = sgrid_planner_.map_.data_model->GetIDFromPosition(goal_pos_.x, goal_pos_.y);
	uint64_t geo_goal_id_astar = sgrid_planner_.graph_->GetVertexFromID(map_goal_id)->bundled_data_->geo_mark_id_;
//	uint64_t map_goal_id = sgrid_planner_.map_.data_model->GetIDFromPosition(goal_pos_.x, goal_pos_.y);
//	uint64_t geo_goal_id_astar = sgrid_planner_.graph_->GetVertexFromID(map_goal_id)->bundled_data_->geo_mark_id_;

	clock_t exec_time;
	exec_time = clock();
	auto comb_path = gcombiner_.combined_graph_.AStarSearch(geo_start_id_astar, geo_goal_id_astar);
	exec_time = clock() - exec_time;
	std::cout << "Search in 3D finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

//	if(!mission_tracker_.mission_started_) {
//		mission_tracker_.UpdateActivePathWaypoints(comb_path);
//	}

	std::vector<Position3Dd> raw_wps;
	for(auto& wp:comb_path)
		raw_wps.push_back(wp->bundled_data_.position);

	// if failed to find a 3d path, terminate this iteration
	if(raw_wps.size() <= 1)
		return;

	std::vector<Position3Dd> selected_wps = MissionUtils::GetKeyTurningWaypoints(raw_wps);

	double est_dist = 0;
	for(int i = 0; i < selected_wps.size() - 1; i++)
		est_dist += std::sqrt(std::pow(selected_wps[i].x - selected_wps[i + 1].x,2) +
				std::pow(selected_wps[i].y - selected_wps[i + 1].y,2) +
				std::pow(selected_wps[i].z - selected_wps[i + 1].z,2));

	if(!init_plan_found_ || (selected_wps.size()>0 && est_dist < est_dist2goal_ * (1 - 0.3)))
	{
		if(init_plan_found_) {
			std::cout << "-------- found better solution ---------" << std::endl;
			std::cout << "current path: " << est_dist2goal_ << " , new path: " <<  est_dist << std::endl;
		}
		else {
			init_plan_found_ = true;
		}

		// TODO this line is only for debugging, should be removed later !!!!!!!!!!!!!!!!!!!!!!!!!!!!
		est_dist2goal_ = est_dist;

		// send data for visualization
		Send3DSearchPathToVis(selected_wps);

		Eigen::Vector3d goal_vec(selected_wps.back().x, selected_wps.back().y, 0);

		kf_cmd.kf_num = selected_wps.size();
		for(auto& wp:selected_wps)
		{
			srcl_lcm_msgs::Keyframe_t kf;
			kf.vel_constr = false;

			kf.positions[0] = wp.x;
			kf.positions[1] = wp.y;
			kf.positions[2] = wp.z;

			Eigen::Vector3d pos_vec(wp.x, wp.y, 0);
			Eigen::Vector3d dir_vec = goal_vec - pos_vec;
			Eigen::Vector3d x_vec(1,0,0);
			double angle = - std::acos(dir_vec.normalized().dot(x_vec));
			kf.yaw = angle;

			kf_cmd.kfs.push_back(kf);
		}
		kf_cmd.kfs.front().yaw = 0;
		kf_cmd.kfs.back().yaw = -M_PI/4;

		//lcm_->publish("quad_planner/goal_keyframe_set", &kf_cmd);
	}

	if(count++ == 20)
	{
//		count = 0;
//		srcl_lcm_msgs::Graph_t graph_msg;
//
//		graph_msg.vertex_num = gcombiner_.combined_graph_.GetGraphVertices().size();
//		for(auto& vtx : gcombiner_.combined_graph_.GetGraphVertices())
//		{
//			srcl_lcm_msgs::Vertex_t vertex;
//			vertex.id = vtx->vertex_id_;
//
//			vertex.position[0] = vtx->bundled_data_.position.x;
//			vertex.position[1] = vtx->bundled_data_.position.y;
//			vertex.position[2] = vtx->bundled_data_.position.z;
//
//			graph_msg.vertices.push_back(vertex);
//		}
//
//		graph_msg.edge_num = gcombiner_.combined_graph_.GetGraphUndirectedEdges().size();
//		for(auto& eg : gcombiner_.combined_graph_.GetGraphUndirectedEdges())
//		{
//			srcl_lcm_msgs::Edge_t edge;
//			edge.id_start = eg.src_->vertex_id_;
//			edge.id_end = eg.dst_->vertex_id_;
//
//			graph_msg.edges.push_back(edge);
//		}
//
//		lcm_->publish("quad_planner/geo_mark_graph", &graph_msg);
	}
}

template<typename PlannerType>
srcl_lcm_msgs::Graph_t QuadPathRepair::GetLcmGraphFromPlanner(const PlannerType& planner)
{
	srcl_lcm_msgs::Graph_t graph_msg;

	graph_msg.vertex_num = planner.graph_->GetGraphVertices().size();
	for(auto& vtx : planner.graph_->GetGraphVertices())
	{
		srcl_lcm_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapPaddedToRefWorld(vtx->bundled_data_->location_, planner.map_.info);
		vertex.position[0] = ref_world_pos.x;
		vertex.position[1] = ref_world_pos.y;

		graph_msg.vertices.push_back(vertex);
	}

	graph_msg.edge_num = planner.graph_->GetGraphUndirectedEdges().size();
	for(auto& eg : planner.graph_->GetGraphUndirectedEdges())
	{
		srcl_lcm_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg.edges.push_back(edge);
	}

	return graph_msg;
}

srcl_lcm_msgs::Graph_t QuadPathRepair::GenerateLcmGraphMsg()
{
	srcl_lcm_msgs::Graph_t graph_msg;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		graph_msg = GetLcmGraphFromPlanner(this->qtree_planner_);
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		graph_msg = GetLcmGraphFromPlanner(this->sgrid_planner_);
	}

	return graph_msg;
}

srcl_lcm_msgs::Path_t QuadPathRepair::GenerateLcmPathMsg(std::vector<Position2D> waypoints)
{
	srcl_lcm_msgs::Path_t path_msg;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		path_msg.waypoint_num = waypoints.size();
		for(auto& wp : waypoints)
		{
			Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapPaddedToRefWorld(wp, this->qtree_planner_.map_.info);
			srcl_lcm_msgs::WayPoint_t waypoint;
			waypoint.positions[0] = ref_world_pos.x;
			waypoint.positions[1] = ref_world_pos.y;

			path_msg.waypoints.push_back(waypoint);
		}
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		path_msg.waypoint_num = waypoints.size();
		for(auto& wp : waypoints)
		{
			Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapPaddedToRefWorld(wp, this->sgrid_planner_.map_.info);
			srcl_lcm_msgs::WayPoint_t waypoint;
			waypoint.positions[0] = ref_world_pos.x;
			waypoint.positions[1] = ref_world_pos.y;

			path_msg.waypoints.push_back(waypoint);
		}
	}

	return path_msg;
}

void QuadPathRepair::Send3DSearchPathToVis(std::vector<Position3Dd> path)
{
	if(path.size() > 0)
	{
		srcl_lcm_msgs::Path_t path_msg;

		path_msg.waypoint_num = path.size();
		for(auto& wp : path)
		{
			srcl_lcm_msgs::WayPoint_t waypoint;
			waypoint.positions[0] = wp.x;
			waypoint.positions[1] = wp.y;
			waypoint.positions[2] = wp.z;

			path_msg.waypoints.push_back(waypoint);
		}

		lcm_->publish("quad_planner/geo_mark_graph_path", &path_msg);
	}
}
