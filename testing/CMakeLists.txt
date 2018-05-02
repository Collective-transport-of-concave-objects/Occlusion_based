add_library(collective_transport MODULE 
	collective_transport.h 
	collective_transport.cpp
	cmd.h
	cmd.cpp
)

target_link_libraries(collective_transport
  argos3core_simulator
  argos3plugin_simulator_kheperaiv
  argos3plugin_simulator_genericrobot)

add_library(trajectory_loop_functions MODULE 
  trajectory_loop_functions.h
  trajectory_qtuser_functions.h
  trajectory_loop_functions.cpp
  trajectory_qtuser_functions.cpp
	GMP.h
	GMP.cpp
	cmd.h
	cmd.cpp
)

target_link_libraries(trajectory_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_kheperaiv
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
