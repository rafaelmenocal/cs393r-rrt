## Kinematically Constrained Rapidly Exploring Random Trees (RRTs)

Abstract: In this paper we provide a solution to the global navigation planning problem using a sampling based algorithm called rapidly exploring random trees (RRTs). Our particular implementation provides a kinematically feasible solution during forwards and backwards motion, assuming constant curvature paths between nodes. Using this implementation, we can demonstrate how a global navigation planner can be used to plan more complicated maneuvers such as back-in or parallel parking.

Steps to Run:
1) Run `roscore`.
2) Change directory `roscd ut_automata`.
3) Run `./bin/simulator --localize`.
4) Run `./bin/websocket`.
5) Open visualization `google-chrome web_rviz.html`.
6) Change directory to this repo. 
7) Compile binaries `make`.
8) Run navigation `./bin/navigation`.

Try modifying the following variables to see additional characteristics and debugging information: 
1) float_t max_truncation_dist = 1.5; (line 97: navigation.cc)
2) bool view_sampled_debug = false; (line 79: navigation.cc)
3) float min_turn_radius_ = 0.45; (line 70: navigation.h)
