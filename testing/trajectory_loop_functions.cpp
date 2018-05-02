#include "trajectory_loop_functions.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/angles.h>
#include "collective_transport.h"
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <string>

static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

void CTrajectoryLoopFunctions::Init(TConfigurationNode& t_tree) {

//  Search for boxes in the environment and check if any one of them is a object

    CSpace::TMapPerType& tBoxMap = GetSpace().GetEntitiesByType("box");
    for(CSpace::TMapPerType::iterator it = tBoxMap.begin();
        it != tBoxMap.end();
        ++it) {
        /* Create a pointer to the current box */
        CBoxEntity* pcBox = any_cast<CBoxEntity*>(it->second);
        if (pcBox->GetId() == "object") {
            CVector3 objSize = pcBox->GetSize();
            CVector3 objPos = pcBox->GetEmbodiedEntity().GetOriginAnchor().Position;
            LOG << "object found! size X:" << objSize.GetX() << " Y:" << objSize.GetY() << " position X:" << objPos.GetX() << " Y:" << objPos.GetY() << std::endl;
            gmp.SetStart(objPos.GetX(), objPos.GetY());

            CSpace::TMapPerType& m_cKheperaiv = GetSpace().GetEntitiesByType("kheperaiv");

            for(CSpace::TMapPerType::iterator it = m_cKheperaiv.begin();
                it != m_cKheperaiv.end();
                ++it) {
                /* Get handle to Kheperaiv entity and controller */
                CKheperaIVEntity& cKheperaiv = *any_cast<CKheperaIVEntity*>(it->second);
                CKheperaIVDiffusion& cController = dynamic_cast<CKheperaIVDiffusion&>(cKheperaiv.GetControllableEntity().GetController());
                
                cController.setBoxLocation(objPos.GetX(), objPos.GetY());
            };
}
}
}
void CTrajectoryLoopFunctions::Reset() {
}


void CTrajectoryLoopFunctions::Destroy() {}

/****************************************/
/****************************************/
 void CTrajectoryLoopFunctions::PreStep() {
}

void CTrajectoryLoopFunctions::PostStep() {
}

bool CTrajectoryLoopFunctions::IsExperimentFinished() {
        return false;
    }

/****************************************/
/****************************************/
    void CTrajectoryLoopFunctions::PostExperiment() {}

/****************************************/
/****************************************/
    CColor CTrajectoryLoopFunctions::GetFloorColor(const CVector2& c_pos_on_floor) {
     return CColor::WHITE;
  }

void CTrajectoryLoopFunctions::MoveEntity(CPositionalEntity& c_entity,
                          const CVector3& c_position,
                          const CQuaternion& c_orientation) {}

/****************************************/
/****************************************/
  /**
   * Moves the entity to the wanted position and orientation.
   * The movement is allowed only if the
   * object does not collide with anything once in the new position.
   * @param c_entity The positional or embodied component of the entity to move.
   * @param c_position The wanted position.
   * @param c_orientation The wanted orientation.
   * @param b_check_only If <tt>false</tt>, the movement is executed; otherwise, the object is not actually moved.
   * @return <tt>true</tt> if no collisions were detected, <tt>false</tt> otherwise.
   */
    bool CTrajectoryLoopFunctions::MoveEntity(CEmbodiedEntity& c_entity,
                          const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false) {
    return true;
    }

/****************************************/
/****************************************/
  /**
   * Adds the passed entity to the simulation.
   * Important: the entity must be created with a <tt>new</tt> statement or a CFactory::New() statement.
   * In other words, the entity must be stored in the heap.
   * If this is not the case, memory corruption will occur.
   * @param c_entity A reference to the entity to add.
   * @throws CARGoSException if an error occurs.
   */
    void CTrajectoryLoopFunctions::AddEntity(CEntity& c_entity) {}

/****************************************/
/****************************************/
  /**
   * Removes an entity from the simulation.
   * @param str_entity_id The id of the entity to remove.
   * @throws CARGoSException If an entity with the specified id was not found.
   */
    void CTrajectoryLoopFunctions::RemoveEntity(const std::string& str_entity_id) {}

/****************************************/
/****************************************/
  /**
   * Removes an entity from the simulation.
   * @param c_entity A reference to the entity to remove.
   */
    void CTrajectoryLoopFunctions::RemoveEntity(CEntity& c_entity) {}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "trajectory_loop_functions")


