#include "route_planner.h"
#include <algorithm>
#include <unordered_map>

using namespace std;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

  
    this->start_node = &model.FindClosestNode(start_x,start_y);
    this->end_node = &model.FindClosestNode(end_x,end_y);

    
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

auto other = *(this->end_node);

auto dist=node->distance(other);
//cout <<"dist " << dist << endl;
return dist;

}



// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {


current_node->FindNeighbors();

//iterate through the current_node->neighbours vector

int len=current_node->neighbors.size();

//update the expansion value
auto g_value = current_node->g_value+1;
auto h_value = 0.0;
//cout <<"g val: " <<g_value<<endl;

for (int i=0 ; i < len ; i++){


    // calculate  the heuristics 'h' which is the distance to the end node or goal
    h_value=CalculateHValue(current_node->neighbors[i]);
    current_node->neighbors[i]->h_value=h_value;
    current_node->neighbors[i]->g_value=g_value;
    current_node->neighbors[i]->parent=current_node;
    current_node->neighbors[i]->visited=true;
    open_list.push_back(current_node->neighbors[i]);

}



}



bool CompareHplusG(RouteModel::Node const *node1, RouteModel::Node const *node2)
{

auto H1= node1->g_value + node1->h_value;
auto H2= node2->g_value + node2->h_value;

if(H1>=H2) return true;
else return false;

//return(H1>H2);

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {


  //iterate through the current_node->neighbours vector


  std::sort( open_list.begin(), open_list.end(),CompareHplusG);


//after sort we should get the node with the least g+h value
//this node needs to be returned as the node to be traversed next 
//remove this node from the list of open nodes to be examined

auto * Next_Node = new RouteModel:: Node; //where to delete this node ?

Next_Node=open_list.back();

open_list.pop_back();

return (Next_Node);

}



// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    //do until parent node is null (=> start node)
    // when parent node is found compute distance from parent node and update local distance variable
    // make current node as parent node and continue with the loop
   auto *next_node = current_node;

    int count =0;
    while((next_node->parent->x != this->start_node->x) && (next_node->parent->y != this->start_node->y))
    {
        auto other = *(next_node->parent);
        path_found.insert(path_found.begin(), other);
        distance += next_node->distance(other);
        next_node = next_node->parent;
     }



    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
   auto *current_node = this->start_node;

    auto x =   this->end_node->x;
    auto y =   this->end_node->y;


  //  cout << "endx,endy" << this->end_node->x <<"," << this->end_node->y<< endl;

    // TODO: Implement your solution here.
    //while the node is not the end node 
    int count=0;
    while((x != current_node->x) && (y != current_node->y))
    {

     //   if((x == current_node->x) && (y == current_node->y)) break;
       
            AddNeighbors(current_node);
            //delete current_node; ??
            current_node=NextNode();
         //   cout << "endx,endy,currx,curry: " << this->end_node->x <<"," << this->end_node->y<<","<<current_node->x<<","<<current_node->y<<endl;
    
     }
  //   cout << "end of search" <<endl;

    //search ended 
    //construct the path
    if (current_node != nullptr)
        m_Model.path =  ConstructFinalPath(current_node);
}