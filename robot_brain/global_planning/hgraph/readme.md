## The Hypothesis Graph (HGraph) is the conductor that orgestrates planning, executing and storing of actions (edges in the HGraph).

After initialisation and setup of the HGraph (functions _init()_ and _setup(objects, task)_) The only function that should be called is _respond()_. 

#### Main functions:
- _respond()_:

The only interface into the HGraph, _respond()_ does one of the 2 things, if variable _in_loop_ is THINKING it calls _search_hypothesis()_, if variable _in_loop_ is EXECUTING it calls the current_edge for a response that will then send input toward the robot.
- _search_hypothesis()_:

For the current subtask, find the nodes and edges to connect the starting node to the target node. To connect the nodes, functions _create_drive_edge()_ and _create_push_edge()_ are called. When a path between the start and target node is found, variable _in_loop_ is set to EXECUTING, and _respond()_ is called.
- _create_drive_edge()_, _create_push_edge()_:

Create a drive or push edge, both are in class _ActionEdge()_ and can have the following status: INITIALISED, PATH_EXISTS, HAS_SYSTEM_MODEL, PATH_IS_PLANNED, EXECUTING, COMPLETED, FAILED. Depending on the edge's status certain actions must be taken (e.g. path estimation, motion planning) until the edge has status PATH_IS_PLANNED. Then it is ready for execution. To progress the edge's status a system model is required, obtaining a system model can be performed by an _IdentificationEdge()_. An exception is raised if all possible edges are on the blacklist.

- _estimate_path(edge)_:

Estimate a path for an edge. A path found will perform as a 'warm start' for motion planning. If no path can be found the target node is laballed UNFEASIBLE. 
- _search_path(edge)_:

Perform motion planning for an edge. A blocking object is detected and should be relocated to free the path. When no path can be found, an exception is raised. 

#### Important variables:

- _in_loop_: 

Flag indicating if currently we are in the SEARCH or EXECUTION loop.
- _current_subtask_:

Keep track of the currently active subtask, when it is completed or concluded to be impossible, move to the next subtask in the task.
- _current_node_ and _current_edge_:

Keep track of the currently active node and edge. The _current_node_ has one outgoing edge, the _current_edge_, newly generated nodes and edges are added after the _current_node_ and before the _current_edge_. Then the _current_edge_ is updated to make it the first outgoing edge of the _current_node_ again.
- _hypothesis_:

List of edges, that if all completed successfully, complete the _current_subtask_.

- _blacklist_:

List of edges that triggered an exception and should not be generated again during the lifetime of the HGraph.

### Flowchart structure of the Hypothesis Graph:
<p align="center">
  <img width="760" height="auto" src="../../../images/hypothesis_graph_flowchart.png">
</p>
# TODO: put the pdf file, not the png for some sweet quality

