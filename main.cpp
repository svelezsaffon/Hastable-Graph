/* 
 * File:   main.cpp
 * Author: Santiago
 *
 * Created on 5 de octubre de 2014, 18:56
 */

#include <cstdlib>
#include "graph.h"
using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {

    graph<char,float> gra;
    
    gra.insert_vertex('a');
    gra.insert_vertex('b');
    gra.insert_vertex('c');
    gra.insert_vertex('d');
    gra.insert_vertex('e');
    gra.insert_vertex('f');
    
    gra.insert_edge('a','b',1.0);
    gra.insert_edge('a','c',1.0);
    
    gra.insert_edge('b','c',1.0);    
    gra.insert_edge('b','d',1.0);
    gra.insert_edge('b','e',1.0);
        
    gra.insert_edge('c','d',1.0);
    gra.insert_edge('c','e',1.0);
    
    gra.insert_edge('d','f',1.0);    
    gra.insert_edge('e','f',1.0);    
    
    
    
    
    
    
    
    
    
    //gra.print();
    gra.DFS('a').print();
    
    
    return 0;
}


