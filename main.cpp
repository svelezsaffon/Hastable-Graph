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

    gra.insert_edge('a','b',1.0);
    gra.insert_edge('a','c',1000.0);    
    gra.insert_edge('b','c',1.0);    
    
    
    
    
    
    
    
    
    
    //gra.print();
    gra.close_friends('a').print();
    
    
    return 0;
}


