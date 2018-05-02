#ifndef TRAJECTORY_LOOP_FUNCTIONS_H
#define TRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
//#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include "GMP.h"

using namespace argos;

class CTrajectoryLoopFunctions : public CLoopFunctions {

public:

   typedef std::map<CKheperaIVEntity*, std::vector<CVector3> > TWaypointMap;
   TWaypointMap m_tWaypoints;

   typedef std::vector<CVector3> TSearchpointMap;
   TSearchpointMap m_tSearchpoints;

   typedef std::vector<CVector3> TParentMap;
   TParentMap m_tParents;

   typedef std::vector<CVector3> TChildrenMap;
   TChildrenMap m_tChildren;

   typedef std::vector<CVector3> TSolutionMap;
   TSolutionMap m_tSolution;

public:

      /**
       * Class constructor.
       */
      CTrajectoryLoopFunctions() :
         m_cSimulator(CSimulator::GetInstance()),
         m_cSpace(m_cSimulator.GetSpace()) {
         }

   virtual ~CTrajectoryLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();
   
   virtual void Destroy();
   
   virtual void PreStep();

   virtual void PostStep();
   
   virtual bool IsExperimentFinished();
   
   virtual void PostExperiment();
   
  virtual CColor GetFloorColor(const CVector2& c_pos_on_floor);
  

  /**
   * Returns the simulator instance.
   * @return The simulator instance.
   */
  inline CSimulator& GetSimulator() {
     return m_cSimulator;
  }

  /**
   * Returns a reference to the space state.
   * @return A reference to the space state.
   */
  inline CSpace& GetSpace() {
     return m_cSpace;
  }

  /**
   * Moves the entity to the wanted position and orientation.
   * @param c_entity The positional component of the entity to move.
   * @param c_position The wanted position.
   * @param c_orientation The wanted orientation.
   */
  virtual void MoveEntity(CPositionalEntity& c_entity,
                          const CVector3& c_position,
                          const CQuaternion& c_orientation);

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
  virtual bool MoveEntity(CEmbodiedEntity& c_entity,
                          const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only);

  /**
   * Adds the passed entity to the simulation.
   * Important: the entity must be created with a <tt>new</tt> statement or a CFactory::New() statement.
   * In other words, the entity must be stored in the heap.
   * If this is not the case, memory corruption will occur.
   * @param c_entity A reference to the entity to add.
   * @throws CARGoSException if an error occurs.
   */
  virtual void AddEntity(CEntity& c_entity);

  /**
   * Removes an entity from the simulation.
   * @param str_entity_id The id of the entity to remove.
   * @throws CARGoSException If an entity with the specified id was not found.
   */
  virtual void RemoveEntity(const std::string& str_entity_id);

  /**
   * Removes an entity from the simulation.
   * @param c_entity A reference to the entity to remove.
   */
  virtual void RemoveEntity(CEntity& c_entity);

  inline const TWaypointMap& GetWaypoints() const {
        return m_tWaypoints;
        }

  inline const TSearchpointMap& GetSearchpoints() const {
        return m_tSearchpoints;
        }

  inline const TParentMap& GetParents() const {
        return m_tParents;
        }

  inline const TChildrenMap& GetChildren() const {
        return m_tChildren;
        }

  inline const TSolutionMap& GetSolution() const {
        return m_tSolution;
        }

private:

    /** A reference to the CSimulator instance */
    CSimulator& m_cSimulator;

    /** A reference to the CSpace instance */
    CSpace& m_cSpace;
  
    CVector3 start;
    GMP gmp;

};

#endif
