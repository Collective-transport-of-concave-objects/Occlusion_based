#ifndef COLLECTIVE_TRANSPORT_H
#define COLLECTIVE_TRANSPORT_H

/*
 * Include some necessary headers.
 */
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_light_sensor.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include "cmd.h"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CKheperaIVDiffusion : public CCI_Controller {

public:

   /* Class constructor. */
   CKheperaIVDiffusion();

   /* Class destructor. */
   virtual ~CKheperaIVDiffusion() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   /*
    * Returns true if the robot needs new command.
    */
   inline STAT GetStatus() {
      return status;
   }
   
   void SetCommand(cmd c);

   void setBoxLocation(float x, float y);

   float getBoxX();
   float getBoxY();

   bool inRange(float currOrientation, float desiredOrientation);
   bool Light();
   bool inObjectRange(float getBoxX(), float getBoxY(),float my_x,float my_y);

private:

//    cmd GetCmd();
    
   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the Khepera IV proximity sensor */
   CCI_KheperaIVProximitySensor* m_pcProximity;

   CCI_KheperaIVLightSensor* m_pcLight;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

   /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   Real m_fRotVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;
   
   CCI_PositioningSensor* m_pcPosition;
   //CCI_PositioningSensor::SReading m_sPosition;
   CVector3 m_sPosition;

    int loop_count = 0;
    CVector3 destination;
    CVector2 cAccumulator1;
    float orientation;
    float m_sOrientation;
    bool adjusting_orientation;
    cmd command;    // current command
    STAT status;    // current status

    float boxXY[2];
    bool oriented;

};

#endif
