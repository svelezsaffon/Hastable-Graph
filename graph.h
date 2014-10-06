/* 
 * File:   graph.h
 * Author: Santiago
 *
 * Created on 5 de octubre de 2014, 18:58
 */




#ifndef GRAPH_H
#define	GRAPH_H


#include <unordered_map>
#include <queue>
#include <stack>
#include <iostream>
#include <vector>
#include <bits/stl_tree.h>



template<class V, class L>
class graph {
    //stl unordered_map has average constant time insert and find.
    //this will create a graph that is O(1) insertion and retrieval. Adjacency matrix like
    //The worst case scenario is that we have to rehash. THe rehash occurs when 
    // we run out of buckets
    
    

public:
    std::unordered_map< V, std::unordered_map < V, L > > central;

    //this will help when inserting and retrieving a new link
    typedef typename std::unordered_map < V, L > vertex_link;

    //This will be a data type that will help me iterate all the neighbors from a given node
    typedef typename std::pair < V, std::unordered_map < V, L > > neighbors;

    //this is just an iterator of the previous data structure, this is like a template
    typedef typename graph::neighbors::second_type::iterator neighbors_iter;

    //so much time saved when inserting and retrieving :)
    typedef typename std::unordered_map< V, std::unordered_map < V, L > >::iterator graph_iterator;


    //this variable will tell us if the graph is undirected
    bool directed;



    void insert_vertex(V);
    void insert_edge(V, V, L);
    bool contains_node(V);
    graph < V, L > BFS(V);
    graph < V, L > DFS(V);
    std::vector < V > path(V,V);
    graph();
    graph(bool);

    void print();

    neighbors_iter end_of(V node) {
        if (graph::contains_node(node)) {
            return (*graph::central.find(node)).second.end();
        }
    };

    neighbors_iter begin_of(V node) {
        if (graph::contains_node(node)) {
            return (*graph::central.find(node)).second.begin();
        }
    };

};

template<class V, class L>
graph<V, L>::graph() {
    directed = false;
};

template<class V, class L>
graph<V, L>::graph(bool dir) {
    directed = dir;
};

template<class V, class L>
void graph<V, L>::print() {
    graph::graph_iterator it = graph::central.begin();
    while (it != graph::central.end()) {
        std::cout << (*it).first;

        graph::neighbors_iter it_neihg = graph::begin_of((*it).first);
        graph::neighbors_iter it_end = graph::end_of((*it).first);

        std::cout << "{";
        while (it_neihg != it_end) {

            std::cout << (*it_neihg).first << "->[" << (*it_neihg).second << "];";

            it_neihg++;
        }
        std::cout << "}" << std::endl;
        it++;

    }
};

template <class V, class L>
bool graph<V, L>::contains_node(V node) {
    return (graph::central.find(node) != graph::central.end());
};

template<class V, class L>
void graph<V, L>::insert_vertex(V vertex) {
    graph_iterator it = central.find(vertex);
    if (it == central.end()) {
        //the node does not exist :) lets give memory to a new map

        graph::vertex_link link;

        graph::neighbors dest(vertex, link);

        //This has a constant average running time
        graph::central.insert(dest);

    }
};

template<class V, class L>
void graph<V, L>::insert_edge(V from, V to, L value) {

    if (graph::contains_node(from) == true && graph::contains_node(to) == true) {

        //In this operation we will be doing a double insert, but since both of them are O(1)
        graph::graph_iterator it = graph::central.find(from);

        //we already have this node, lets look for the second node
        graph::neighbors_iter it_neigh = (*it).second.find(to);
        if (it_neigh == (*it).second.end()) {
            //we dont have the to element, so lets put it inside FROM's neighbprs maps
            std::pair < V, L > link(to, value);
            (*it).second.insert(link);

        } else {
            //this is an update we are making the link value to -> from
            (*it_neigh).second = value;
        }


        //making the graph undirected            
        if (!graph::directed) {
            graph::graph_iterator it2nd = graph::central.find(to);

            //we already have this node, lets look for the second node
            graph::neighbors_iter it_neigh2nd = (*it2nd).second.find(from);
            if (it_neigh2nd == (*it2nd).second.end()) {
                //we dont have the to element, so lets put it inside FROM's neighbprs maps
                std::pair < V, L > link(from, value);
                (*it2nd).second.insert(link);

            } else {
                //this is an update we are making the link value to -> from
                (*it_neigh2nd).second = value;
            }

        }



    };
};
/**
 * This is the Breadth deprth serach algorithm. It runs in O(V+E). starting from the node sent as parameter
 * Please reffer to Jeff Ericksons algorithm description, found on 
 * http://web.engr.illinois.edu/~jeffe/teaching/algorithms/notes/18-graphs.pdf
 * @param node
 * @return Graph with Breath first spanning tree
 */
template<class V, class L>
graph<V, L> graph<V, L>::BFS(V node) {
    graph<V, L> bst(false);
    std::queue < V > rec;
    rec.push(node);
    bst.insert_vertex(node);

    while (rec.empty() == false) {

        V current = rec.back();
        
        rec.pop();
        
        graph::neighbors_iter begin = begin_of(current);
        graph::neighbors_iter end = end_of(current);
        while (begin != end) {

            if (!bst.contains_node((*begin).first)) {
                //we haven't visited this node :) lets use it Yayaaa
                bst.insert_vertex((*begin).first);
                bst.insert_edge(current, (*begin).first, (*begin).second);
                rec.push((*begin).first);
            }

            begin++;
        }
    }
    return bst;
};

template<class V, class L>
graph<V, L> graph<V, L>::DFS(V node) {
    graph<V, L> bst(false);
    std::stack < V > rec;
    rec.push(node);
    bst.insert_vertex(node);

    while (rec.empty() == false) {

        V current = rec.top();
        
        rec.pop();
        
        graph::neighbors_iter begin = begin_of(current);
        graph::neighbors_iter end = end_of(current);
        while (begin != end) {

            if (!bst.contains_node((*begin).first)) {
                //we haven't visited this node :) lets use it Yayaaa
                bst.insert_vertex((*begin).first);
                bst.insert_edge(current, (*begin).first, (*begin).second);
                rec.push((*begin).first);
            }

            begin++;
        }
    }
    return bst;
};



#endif	/* GRAPH_H */

