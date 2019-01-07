/* 
 * tree_vertex_impl.hpp
 * 
 * Created on: Jan 07, 2019 09:20
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef TREE_VERTEX_IMPL_HPP
#define TREE_VERTEX_IMPL_HPP

namespace librav
{
template <typename State, typename Transition, typename StateIndexer>
bool Tree<State, Transition, StateIndexer>::Vertex::operator==(const Tree<State, Transition, StateIndexer>::Vertex &other)
{
    if (vertex_id_ == other.vertex_id_)
        return true;
    return false;
}

template <typename State, typename Transition, typename StateIndexer>
typename Tree<State, Transition, StateIndexer>::Vertex::edge_iterator Tree<State, Transition, StateIndexer>::Vertex::FindEdge(int64_t dst_id)
{
    typename Tree<State, Transition, StateIndexer>::Vertex::edge_iterator it;
    for (it = edge_begin(); it != edge_end(); ++it)
    {
        if (it->dst_->vertex_id_ == dst_id)
            return it;
    }
    return it;
}

template <typename State, typename Transition, typename StateIndexer>
template <class T = State, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
typename Tree<State, Transition, StateIndexer>::Vertex::edge_iterator Tree<State, Transition, StateIndexer>::Vertex::FindEdge(T dst_state)
{
    typename Tree<State, Transition, StateIndexer>::Vertex::edge_iterator it;
    for (it = edge_begin(); it != edge_end(); ++it)
    {
        if(this->GetStateIndex(it->dst_->state_) == this->GetStateIndex(dst_state))
            return it;
    }
    return it;
}

template <typename State, typename Transition, typename StateIndexer>
template <typename T>
bool Tree<State, Transition, StateIndexer>::Vertex::CheckNeighbour(T dst)
{
    auto res = FindEdge(dst);
    if (res != edge_end())
        return true;
    return false;
}

template <typename State, typename Transition, typename StateIndexer>
std::vector<typename Tree<State, Transition, StateIndexer>::vertex_iterator> Tree<State, Transition, StateIndexer>::Vertex::GetNeighbours()
{
    std::vector<typename Tree<State, Transition, StateIndexer>::vertex_iterator> nbs;
    for (auto it = edge_begin(); it != edge_end(); ++it)
        nbs.push_back(it->dst_);
    return nbs;
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::Vertex::ClearVertexSearchInfo()
{
    is_checked_ = false;
    is_in_openlist_ = false;
    search_parent_ = vertex_iterator();

    f_cost_ = 0.0;
    g_cost_ = 0.0;
    h_cost_ = 0.0;
}
} // namespace librav

#endif /* TREE_VERTEX_IMPL_HPP */
