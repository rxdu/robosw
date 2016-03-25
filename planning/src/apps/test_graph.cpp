/*
 * test_graph.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: rdu
 */

// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>

// opencv
#include "opencv2/opencv.hpp"

// quad_tree
#include "qtree_builder.h"
#include "graph_builder.h"
#include "astar.h"
#include "graph_vis.h"
#include "image_utils.h"

using namespace cv;
using namespace srcl_ctrl;

int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    Mat image_raw;
    Mat image_disp;
    image_raw = imread( argv[1], IMREAD_GRAYSCALE );

    if ( !image_raw.data )
    {
        printf("No image data \n");
        return -1;
    }

    // example to use quadtree builder
//    QTreeBuilder builder;
//    QuadTree* tree = builder.BuildQuadTree(image_raw, 6);
    QuadTree* tree = QTreeBuilder::BuildQuadTree(image_raw, 6);

    Mat image_tree, image_nodes;
    GraphVis vis;
    Mat bin_map, pad_map, vis_map;
    ImageUtils::BinarizeImage(image_raw, bin_map, 200);
    ImageUtils::PadImageToSquared(bin_map, pad_map);
    vis.DrawQuadTree(tree, pad_map, image_tree, TreeVisType::ALL_SPACE);
//    TreeNode* node = tree->leaf_nodes_.at(0);
//    vis.DrawQTreeSingleNode(node, image_tree, image_nodes);
    std::vector<QuadTreeNode*> free_leaves;
    std::vector<QuadTreeNode*>::iterator it;
    for(it = tree->leaf_nodes_.begin(); it != tree->leaf_nodes_.end(); it++)
    {
    	if((*it)->occupancy_ == OccupancyType::FREE)
    		free_leaves.push_back((*it));
    }
    vis.DrawQTreeNodes(free_leaves, image_tree, image_nodes);

//    Mat image_dummy;
//    vis.DrawQTreeWithDummies(tree,builder.padded_img_, image_dummy);

    // build a graph from quadtree
    Graph<QuadTreeNode>* graph;

	graph = GraphBuilder::BuildFromQuadTree(tree);
	Mat image_graph;
	vis.DrawQTreeGraph(graph, tree, image_tree, image_graph,true, false);
//	vis.DrawQTreeGraph(graph, tree, image_tree, image_disp);

	// try a* search
	std::vector<Vertex<QuadTreeNode>*> vertices = graph->GetGraphVertices();
	std::cout<<"vertex number: "<<vertices.size()<<std::endl;

	Vertex<QuadTreeNode>* start_vertex;
	Vertex<QuadTreeNode>* end_vertex;
	std::vector<Vertex<QuadTreeNode>*> traj;

//	start_vertex = vertices[0];
//	end_vertex = vertices[15];
	start_vertex = graph->GetVertexFromID(172);
	end_vertex = graph->GetVertexFromID(20);

	clock_t		exec_time;

	std::cout<<"Start from "<< start_vertex->vertex_id_<<" and finish at "<< end_vertex->vertex_id_<< std::endl;
	exec_time = clock();
	traj = graph->AStarSearch(start_vertex, end_vertex);
	exec_time = clock() - exec_time;
	std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	Mat path_img;
	vis.DrawQTreeGraphPath(traj,image_graph,path_img);

	image_disp = path_img;

//    imwrite( "new_map_path_cmp1.jpg", image_disp);

    namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
    imshow("Processed Image", image_disp);

    waitKey(0);

    delete graph;
    delete tree;

    return 0;
}



