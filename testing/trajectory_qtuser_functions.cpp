#include "trajectory_qtuser_functions.h"
#include "trajectory_loop_functions.h"
#include "GMP.h"

/****************************************/
/****************************************/

CTrajectoryQTUserFunctions::CTrajectoryQTUserFunctions() :
   m_cTrajLF(dynamic_cast<CTrajectoryLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
}

/****************************************/
/****************************************/

/**
* Drawing hook executed after the floor is drawn.
*/
//void CTrajectoryQTUserFunctions::Draw(CFloorEntity& c_entity) {
//}

/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawInWorld() {
/* Go through all the robot waypoints and draw them */
   for(CTrajectoryLoopFunctions::TWaypointMap::const_iterator it = m_cTrajLF.GetWaypoints().begin();
       it != m_cTrajLF.GetWaypoints().end();
       ++it) {
      DrawWaypoints(it->second);
   }
   //DrawSearchpoints(m_cTrajLF.GetSearchpoints());
   // DEBUG
   //DrawParentspoints(m_cTrajLF.GetParents(), m_cTrajLF.GetChildren());
   DrawSolutionpoints(m_cTrajLF.GetSolution());
}

/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawParentspoints(const std::vector<CVector3>& c_parents, const std::vector<CVector3>& c_children) {
   /* Start drawing segments when you have at least two points */
        // DEBUG LOG << "Sizes - Parents:" << c_parents.size() << " Children:" << c_children.size() << std::endl;
        // DEBUG LOG << "P1:" << c_parents[1] << " C1:" << c_children[1] << std::endl;
      size_t unEnd = 0;
      while(unEnd < c_parents.size()) {
         DrawRay(CRay3(c_parents[unEnd],
                       c_children[unEnd]), CColor::GREEN, 5.0f);
         ++unEnd;
   }
}

/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawSolutionpoints(const std::vector<CVector3>& c_solution) {
   /* Start drawing segments when you have at least two points */
        // DEBUG LOG << "Size Solution:" << c_solution.size() << std::endl;
        // DEBUG LOG << "P1:" << c_parents[1] << " C1:" << c_children[1] << std::endl;
   if(c_solution.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_solution.size()) {
         DrawRay(CRay3(c_solution[unEnd],
                       c_solution[unStart]), CColor::BLUE, 5.0f);
         ++unStart;
         ++unEnd;
      }
   }
}

/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawSearchpoints(const std::vector<CVector3>& c_searchpoints) {
   /* Start drawing segments when you have at least two points */
   if(c_searchpoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_searchpoints.size()) {
         DrawRay(CRay3(c_searchpoints[unEnd],
                       c_searchpoints[unStart]), CColor::BLUE, 5.0f);
         ++unStart;
         ++unEnd;
      }
   }
}

/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
   /* Start drawing segments when you have at least two points */
   if(c_waypoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_waypoints.size()) {
         DrawRay(CRay3(c_waypoints[unEnd],
                       c_waypoints[unStart]), CColor::RED, 5.0f);
         ++unStart;
         ++unEnd;
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTrajectoryQTUserFunctions, "trajectory_qtuser_functions")
