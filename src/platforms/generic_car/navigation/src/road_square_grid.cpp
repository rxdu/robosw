/* 
 * road_square_grid.cpp
 * 
 * Created on: Apr 12, 2018 17:43
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "navigation/road_square_grid.hpp"

using namespace librav;

// #define ALLOW_DIAGONAL_MOVE

GetRoadSquareGridNeighbour::GetRoadSquareGridNeighbour(std::shared_ptr<RoadSquareGrid> sg, std::shared_ptr<RoadSquareGrid> mask) : sgrid_(sg), mask_(mask)
{
}

std::vector<std::tuple<RoadSquareCell *, double>> GetRoadSquareGridNeighbour::operator()(RoadSquareCell *cell)
{
    std::vector<std::tuple<RoadSquareCell *, double>> adjacent_cells;

    if (cell->label != SquareCellLabel::FREE)
        return adjacent_cells;

#ifdef ALLOW_DIAGONAL_MOVE
    auto neighbours = sgrid_->GetNeighbours(cell->id, true);
#else
    auto neighbours = sgrid_->GetNeighbours(cell->id, false);
#endif

    if (mask_ == nullptr)
    {
        for (auto nb : neighbours)
        {
            if (nb->label == SquareCellLabel::FREE)
            {
                double x_err = cell->center.x - nb->center.x;
                double y_err = cell->center.y - nb->center.y;
                double dist = std::sqrt(x_err * x_err + y_err * y_err);
                adjacent_cells.push_back(std::make_tuple(nb, dist));
            }
        }
    }
    else
    {
        for (auto nb : neighbours)
        {
            // std::cout << "checking: " << nb->x << " , " << nb->y << std::endl;
            // std::cout << "mask: " << mask_->GetValueAtCoordinate(nb->y, nb->x) << std::endl;
            if (mask_->GetCell(nb->x, nb->y)->label == SquareCellLabel::FREE)
            {
                double x_err = cell->center.x - nb->center.x;
                double y_err = cell->center.y - nb->center.y;
                double dist = std::sqrt(x_err * x_err + y_err * y_err);
                adjacent_cells.push_back(std::make_tuple(nb, dist));
            }
            // std::cout << "neighbour size: " << adjacent_cells.size() << std::endl;
        }
    }

    return adjacent_cells;
}